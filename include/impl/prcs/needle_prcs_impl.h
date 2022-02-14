// BSD 3-Clause License

// Copyright (c) 2021, The University of North Carolina at Chapel Hill
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.

// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

//! @author Mengyu Fu

#pragma once
#ifndef SNP_NEEDLE_PRCS_IMPL_H
#define SNP_NEEDLE_PRCS_IMPL_H

#include "prcs_base.hpp"
#include "priority_queue.hpp"

namespace unc::robotics::mpt::impl::prcs {
template <typename Scenario, int maxThreads, bool reportStats, typename NNStrategy>
class NeedlePRCS : public PlannerBase<NeedlePRCS<Scenario, maxThreads, reportStats, NNStrategy>> {
  public:
    using Space = scenario_space_t<Scenario>;
    using State = typename Space::Type;

  private:
    using Planner = NeedlePRCS;
    using Base = PlannerBase<Planner>;
    using Distance = typename Space::Distance;
    using Link = scenario_link_t<Scenario>;
    using Traj = link_trajectory_t<Link>;
    using Node = prcs::Node<State, Traj>;
    using Edge = prcs::Edge<State, Traj>;
    using PriorityQueue = prcs::PriorityQueue<State, Traj>;
    using RNG = scenario_rng_t<Scenario, Distance>;
    using Sampler = scenario_sampler_t<Scenario, RNG>;
    using Point = typename Scenario::Position;
    using Propagator = typename Scenario::Propagator;

    Distance maxDistance_{std::numeric_limits<Distance>::infinity()};

    static constexpr bool concurrent = maxThreads != 1;
    using NNConcurrency = std::conditional_t<concurrent, nigh::Concurrent, nigh::NoThreadSafety>;

    struct NNNode {
        Node* node;
        State state;

        NNNode(Node* n, const State& s)
            : node(n), state(s) {
        }
    };

    struct NNNodeKey {
        const State& operator() (const NNNode& n) const {
            return n.state;
        }
    };

    nigh::Nigh<NNNode, Space, NNNodeKey, NNConcurrency, NNStrategy> nn_;
    nigh::Nigh<StateNode<State>, Space, StateNodeKey, NNConcurrency, NNStrategy> ic_nn_;
    nigh::Nigh<StateNode<State>, Space, StateNodeKey, NNConcurrency, NNStrategy> ic_invalid_nn_;

    Propagator propagator_;

    PriorityQueue queue_;

    std::mutex mutex_;
    std::mutex startMutex_;
    std::mutex terminationMutex_;
    std::mutex activeMutex_;
    std::forward_list<Node*> goals_;
    Distance bestCost_{std::numeric_limits<Distance>::infinity()};
    snp::TimePoint start_time_;
    using ResultSeq = std::vector<std::pair<float, Distance>>;
    ResultSeq resultWithTime_;
    unsigned minValidateRank_{10};

    Node* approxRes_{nullptr};
    Distance bestDist_{std::numeric_limits<Distance>::infinity()};

    Atom<std::size_t, concurrent> goalCount_{0};

    int numActivateWorkers_{0};

    ObjectPool<Node, false> startNodes_;

    struct Worker;

    WorkerPool<Worker, maxThreads> workers_;

    void foundGoal(Node* node) {
        if constexpr (reportStats) {
            MPT_LOG(INFO) << "found solution with cost " << node->cost();
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            goals_.push_front(node);
            bestCost_ = std::fmin(bestCost_, node->cost());
            resultWithTime_.push_back({std::chrono::duration_cast<std::chrono::duration<float>>(snp::Clock::now() - start_time_).count(), node->cost()});
        }

        ++goalCount_;

        bestDist_ = 0.0;
    }

    std::optional<Node*> foundApproxGoal(Node* node, const State& goalState, ObjectPool<Node>& nodePool,
                                         Distance* dist) {
        {
            std::lock_guard<std::mutex> lock(mutex_);

            if (*dist < bestDist_) {
                MPT_LOG(INFO) << "update approximate solution with dist " << *dist;
                bestDist_ = *dist;
                approxRes_ = nodePool.allocate(linkTrajectory(true), node, goalState);
                return approxRes_;
            }
            else if (*dist > bestDist_) {
                *dist = bestDist_;
            }
        }

        return {};
    }

    void addActivateWorker() {
        std::lock_guard<std::mutex> lock(activeMutex_);
        numActivateWorkers_++;
    }

    void removeActivateWorker() {
        std::lock_guard<std::mutex> lock(activeMutex_);
        numActivateWorkers_--;
    }

  public:
    template <typename RNGSeed = RandomDeviceSeed<>>
    explicit NeedlePRCS(const Scenario& scenario = Scenario(), const RNGSeed& seed = RNGSeed())
        : nn_(scenario.space())
        , workers_(scenario, seed)
        , propagator_(scenario.Config()) {
        MPT_LOG(TRACE) << "Using nearest: " << log::type_name<NNStrategy>();
        MPT_LOG(TRACE) << "Using concurrency: " << log::type_name<NNConcurrency>();
        MPT_LOG(TRACE) << "Using sampler: " << log::type_name<Sampler>();

        MPT_LOG(INFO) << "Using planner: " << "NeedlePRCS";
    }

    void setRange(Distance range) {
        assert(range > 0);
        maxDistance_ = range;
    }

    Distance getRange() const {
        return maxDistance_;
    }

    std::size_t iterations() const {
        std::size_t sum = 0;

        for (const Worker& w : workers_) {
            sum += w.numIterations();
        }

        return sum;
    }

    std::size_t size() const {
        std::size_t sumSize = 0;

        for (const Worker& w : workers_) {
            sumSize += w.nodes().size();
        }

        return sumSize;
    }

    template <typename ... Args>
    void addStart(Args&& ... args) {
        std::lock_guard<std::mutex> lock(mutex_);
        Node* node = startNodes_.allocate(Traj{}, nullptr, std::forward<Args>(args)...);
        node->valid() = true;
        node->state().rotation().normalize();
        nn_.insert(NNNode(node, node->state()));
        ic_nn_.insert(StateNode(node->state()));
        queue_.push(node);
    }

    using Base::solveFor;
    using Base::solveUntil;

    template <typename DoneFn>
    std::enable_if_t<std::is_same_v<bool, std::result_of_t<DoneFn()>>>
    solve(DoneFn doneFn) {
        if (nn_.size() == 0) {
            throw std::runtime_error("there are no valid initial states");
        }

        start_time_ = snp::Clock::now();
        workers_.solve(*this, doneFn);
    }

    bool solved() const {
        return goalCount_.load(std::memory_order_relaxed);
    }

    bool approxSolved() const {
        return (approxRes_ != nullptr);
    }

    bool exhausted() const {
        return (numActivateWorkers_ == 0) && (queue_.empty());
    }

    std::size_t numPlansFound() const {
        return goalCount_.load(std::memory_order_relaxed);
    }

  private:
    std::pair<Distance, std::size_t> pathCost(const Node* n) const {
        Distance cost = 0;
        std::size_t size = 0;

        if (n) {
            cost += workers_[0].scenario().FinalStateCost(n->state());
            ++size;

            for (const Node *p ; (p = n->parent()) != nullptr ; n = p) {
                cost += workers_[0].scenario().CurveCost(p->state(), n->state());
                ++size;
            }
        }

        return {cost, size};
    }

    std::tuple<Distance, std::size_t, const Node*> bestSolution() const {
        Distance bestCost = std::numeric_limits<Distance>::infinity();
        std::size_t bestSize = 0;
        const Node* bestGoal = nullptr;

        if (goals_.empty() && approxRes_) {
            const Node* goal = approxRes_;
            auto [cost, size] = pathCost(goal);
            bestCost = cost;
            bestSize = size;
            bestGoal = goal;
            return {bestCost, bestSize, bestGoal};
        }

        for (const Node* goal : goals_) {
            auto [cost, size] = pathCost(goal);

            if (cost < bestCost || bestGoal == nullptr) {
                bestCost = cost;
                bestSize = size;
                bestGoal = goal;
            }
        }

        return {bestCost, bestSize, bestGoal};
    }

    template <typename Fn>
    std::enable_if_t< is_trajectory_callback_v< Fn, State, Traj> >
    solutionRecur(const Node* node, Fn& fn) const {
        if (const Node* p = node->parent()) {
            solutionRecur(p, fn);
            fn(p->state(), node->edge().link(), node->state(), true);
        }
    }

    template <typename Fn>
    std::enable_if_t< is_trajectory_reference_callback_v< Fn, State, Traj > >
    solutionRecur(const Node* node, Fn& fn) const {
        if (const Node* p = node->parent()) {
            solutionRecur(p, fn);
            fn(p->state(), *node->edge().link(), node->state(), true);
        }
    }

    template <typename Fn>
    std::enable_if_t< is_waypoint_callback_v< Fn, State, Traj > >
    solutionRecur(const Node* node, Fn& fn) const {
        if (const Node* p = node->parent()) {
            solutionRecur(p, fn);
        }

        fn(node->state());
    }

  public:
    std::vector<State> solution() const {
        auto [cost, size, n] = bestSolution();
        std::vector<State> path;

        if (n) {
            path.reserve(size);

            do {
                path.push_back(n->state());
            }
            while ((n = n->parent()) != nullptr);

            std::reverse(path.begin(), path.end());
        }

        return path;
    }

    std::vector<std::vector<State>> allSolutions () const {
        std::vector<std::vector<State>> paths;

        for (const Node* n : goals_) {
            std::vector<State> path;

            if (n) {
                auto [cost, size] = pathCost(n);
                path.reserve(size);

                do {
                    path.push_back(n->state());
                }
                while ((n = n->parent()) != nullptr);

                std::reverse(path.begin(), path.end());
            }

            paths.push_back(path);
        }

        return paths;
    }

    template <typename Fn>
    void solution(Fn fn) const {
        auto [cost, size, goal] = bestSolution();

        if (goal) {
            solutionRecur(goal, fn);
        }
    }

    void printStats() const {
        MPT_LOG(INFO) << "nodes in graph: " << nn_.size();
        auto [cost, size, goal] = bestSolution();
        MPT_LOG(INFO) << "solutions: " << goalCount_.load() << ", best cost=" << cost
                      << " over " << size << " waypoints";

        if constexpr (reportStats) {
            WorkerStats<true> stats;

            for (unsigned i=0 ; i<workers_.size() ; ++i) {
                stats += workers_[i];
            }

            stats.print();
        }
    }

    Distance cost() const {
        auto [cost, size, n] = bestSolution();
        return cost;
    }

    std::vector<Distance> allCosts() const {
        std::vector<Distance> costs;

        for (const Node* n : goals_) {
            auto [cost, size] = pathCost(n);
            costs.push_back(cost);
        }

        return costs;
    }

    const ResultSeq& resultWithTime() const {
        return resultWithTime_;
    }

  private:
    template <typename Visitor, typename Nodes>
    void visitNodes(Visitor&& visitor, const Nodes& nodes) const {
        for (const auto& n : nodes) {
            visitor.vertex(n->state());

            if (n->parent()) {
                visitor.edge(n->parent()->state());
            }
        }
    }

  public:
    template <typename Visitor>
    void visitGraph(Visitor&& visitor) const {
        for (const Worker& w : workers_) {
            visitNodes(std::forward<Visitor>(visitor), w.nodes());
        }
    }
};

template <typename Scenario, int maxThreads, bool reportStats, typename NNStrategy>
class NeedlePRCS<Scenario, maxThreads, reportStats, NNStrategy>::Worker
    : public WorkerStats<reportStats> {
    using Stats = WorkerStats<reportStats>;

    unsigned no_;
    Scenario scenario_;
    RNG rng_;

    ObjectPool<Node> nodePool_;
    std::queue<Node*> bin_;
    Distance bestDist_{std::numeric_limits<Distance>::infinity()};
    Distance configTolerance_{0};
    unsigned initNum_{1};
    std::vector<Distance> radList_;
    std::size_t numIterations_{0};
    std::vector<const Node*> closed_;

    enum RefineType {
        SHORTER=0,
        LONGER,
        LEFT,
        RIGHT
    };

  public:
    Worker(Worker&& other)
        : no_(other.no_)
        , scenario_(other.scenario_)
        , rng_(other.rng_)
        , nodePool_(std::move(other.nodePool_)) {
    }

    template <typename RNGSeed>
    Worker(unsigned no, const Scenario& scenario, const RNGSeed& seed)
        : no_(no)
        , scenario_(scenario)
        , rng_(seed) {
    }

    decltype(auto) space() const {
        return scenario_.space();
    }

    decltype(auto) scenario() const {
        return scenario_;
    }

    const auto& nodes() const {
        return closed_;
    }

    const auto& numIterations() const {
        return numIterations_;
    }

    template <typename DoneFn>
    void solve(Planner& planner, DoneFn done) {
        MPT_LOG(TRACE) << "worker running";

        configTolerance_ = scenario_.validator().ConfigTolerance();
        initNum_ = planner.propagator_.InitialNumberofOrientations();
        radList_ = planner.propagator_.RadCurvList();

        if (no_ > 0) {
            auto const startQueueSize = no_;
            while (!done() && planner.queue_.size() < startQueueSize) {
                std::lock_guard<std::mutex> lock(planner.startMutex_);
            }
        }

        while (!done()) {
            Node* popped_node;
            {
                std::lock_guard<std::mutex> lock(planner.terminationMutex_);
                popped_node = planner.queue_.pop();

                if (popped_node == nullptr) {
                    if (planner.exhausted()) {
                        MPT_LOG(TRACE) << "planner exhausted";
                        break;
                    }

                    continue;
                }

                planner.addActivateWorker();
            }

            Stats::countIteration();
            ++numIterations_;
            process(planner, popped_node, done);

            planner.removeActivateWorker();
        }

        MPT_LOG(TRACE) << "worker done";
    }

    bool checkTerminateCondition(Planner& planner, Node* node) {
        auto [isGoal, goalDist, goalStates] = scenario_goal<Scenario>::check(scenario_, node->state());

        if (isGoal) {
            if (goalStates.size() < 2) {
                auto const& goalState = goalStates[0];
                auto const goalLength = node->length() + snp::CurveLength(node->state(), goalState);
                if (scenario_.valid(goalLength)) {
                    auto const goalCost = node->cost() + scenario_.CurveCost(node->state(), goalState)
                                         + scenario_.FinalStateCost(goalState);
                    if (goalCost < planner.bestCost_) {
                        Node* goalNode = nodePool_.allocate(linkTrajectory(true), node, goalState);
                        goalNode->length() = goalLength;
                        goalNode->cost() = goalCost;
                        planner.foundGoal(goalNode);
                    }

                    return true;
                }
            }
            else {
                auto const goalLength0 = node->length() + snp::CurveLength(node->state(), goalStates[0]);
                auto const goalLength1 = goalLength0 + snp::CurveLength(goalStates[0], goalStates[1]);

                if (scenario_.valid(goalLength1)) {
                    auto const goalCost0 = node->cost() + scenario_.CurveCost(node->state(), goalStates[0]);
                    auto const goalCost1 = goalCost0 + scenario_.CurveCost(goalStates[0], goalStates[1])
                                         + scenario_.FinalStateCost(goalStates[1]);

                    if (goalCost1 < planner.bestCost_) {
                        Node* transNode = nodePool_.allocate(linkTrajectory(true), node, goalStates[0]);
                        transNode->length() = goalLength0;
                        transNode->cost() = goalCost0;

                        Node* goalNode = nodePool_.allocate(linkTrajectory(true), transNode, goalStates[1]);
                        goalNode->length() = goalLength1;
                        goalNode->cost() = goalCost1;
                        planner.foundGoal(goalNode);
                    }

                }
            }
        }
        else if (!planner.solved() && goalDist < bestDist_) {
            auto goalState = goalStates[0];
            auto const& goalLength = node->length() + snp::CurveLength(node->state(), goalState);

            if (scenario_.valid(goalLength)) {
                bestDist_ = goalDist;
                auto goalNode = planner.foundApproxGoal(node, goalState, nodePool_, &bestDist_);
                if (goalNode) {
                    (*goalNode)->length() = goalLength;
                    (*goalNode)->cost() = node->cost() + scenario_.CurveCost(node->state(), goalState)
                                          + scenario_.FinalStateCost(goalState);
                }
            }
        }

        return false;
    }

    template <typename DoneFn>
    void process(Planner& planner, Node* node, DoneFn done) {
        State from = node->state();

        if (node->parent()) {
            from = planner.propagator_.ComputeStartPose(node->parent()->state(), node->angleIndex());
            auto duplicatedStart = similarState(planner, node->parent(), from);

            if (duplicatedStart) {
                recycle(node);
                return;
            }

            auto propagated = planner.propagator_(from, node->radIndex(), node->lengthIndex());

            if (!propagated) {
                recycle(node);
                return;
            }

            node->state() = *propagated;
            node->length() = node->parent()->length() + planner.propagator_.Length(node->lengthIndex());
            node->cost() = node->parent()->cost() + scenario_.CurveCost(node->parent()->state(), node->state());
        }

        const bool inheritValidation = node->valid();
        bool inevitableCollision = similarNode(planner.ic_invalid_nn_, node->state(), configTolerance_);

        if (!inevitableCollision && validNode(planner, node)) {
            if (auto traj = validMotion(planner, node, from)) {
                auto const validResult = checkTerminateCondition(planner, node);

                if (done()) {
                    return;
                }

                if (!validResult && node->parent() && node->rank() >= planner.minValidateRank_
                    && !similarNode(planner.ic_nn_, node->state(), 1.0))
                {
                    if (!scenario_.validReachableSpace(node->state())) {
                        inevitableCollision = true;
                        planner.ic_invalid_nn_.insert(StateNode(node->state()));
                    }
                    else {
                        planner.ic_nn_.insert(StateNode(node->state()));
                    }
                }

                if (!inevitableCollision) {
                    expand(planner, node);
                    closed_.push_back(node);
                }
            }
        }

        if (!node->parent()) {
            return;
        }

        auto shorter = refine(planner, node, SHORTER);

        if (node->valid()) {
            if (shorter) {
                shorter->valid() = true;
            }

            if (!inevitableCollision) {
                auto longer = refine(planner, node, LONGER);
                if (inheritValidation && longer) {
                    longer->valid() = true;
                }
            }
        }

        refine(planner, node, LEFT);
        refine(planner, node, RIGHT);

        if (!node->valid()) {
            recycle(node);
        }
    }

    bool similarState(Planner& planner, Node* refNode, State& state) {
        Timer timer(Stats::nearest());
        state.rotation().normalize();
        auto neig = planner.nn_.nearest(state);

        if (neig && neig->second < configTolerance_) {
            if (neig->first.node != refNode) {
                return true;
            }
        }
        else {
            planner.nn_.insert(NNNode(refNode, state));
        }

        return false;
    }

    template<typename NN>
    bool similarNode(NN& nn, State state, const Distance rad) {
        Timer timer(Stats::nearest());
        Vec3 tang = (state.rotation().normalized()*Vec3::UnitZ()).normalized();
        state.rotation() = Quat::FromTwoVectors(Vec3::UnitZ(), tang).normalized();
        auto neig = nn.nearest(state);
        if (neig && neig->second < rad) {
            return true;
        }

        return false;
    }

    decltype(auto) validNode(Planner& planner, Node* node) {
        if (!scenario_.valid(node->length())) {
            node->valid() = false;
            return false;
        }
        else if (node->valid()) {
            return true;
        }

        if (node->parent()
            && node->angleIndex() == 0
            && node->radIndex() == node->parent()->radIndex()
            && node->parent()->lengthIndex() > 0)
        {
            return false;
        }

        if (!scenario_.valid(node->state())) {
            return false;
        }

        return true;
    }

    decltype(auto) validMotion(Planner& planner, Node* node, const State& from) {
        if (node->valid()) {
            return true;
        }

        Timer timer(Stats::validMotion());
        auto const& lengthIdx = node->lengthIndex();
        auto const& baseMotion = planner.propagator_.BaseMotion(node->radIndex(), lengthIdx);

        unsigned offset = 0;
        if (lengthIdx > 0 && lengthIdx % 2 == 0) {
            offset = planner.propagator_.BaseMotion(node->radIndex(), lengthIdx/2).size();
        }

        if (scenario_.validator().ValidMotion(from, baseMotion, offset)) {
            node->valid() = true;
            return true;
        }

        return false;
    }

    void expand(Planner& planner, Node* node) {
        for (auto r_i = 0; r_i < radList_.size(); ++ r_i) {
            if (radList_[r_i] == std::numeric_limits<Distance>::infinity()) {
                addNewNode(planner, node, r_i, 0, 0);
            }
            else {
                for (auto a_idx = 0; a_idx < initNum_; ++a_idx) {
                    addNewNode(planner, node, r_i, 0, 0, 0, a_idx);
                }
            }
        }
    }

    Node* refine(Planner& planner, Node* node, const RefineType& type) {
        if (!node->parent()) {
            throw std::runtime_error("[ERROR] Cannot compute finer motion for the root!");
        }

        if (type == SHORTER || type == LONGER) {
            if (node->lengthLevel() == planner.propagator_.MaxLengthLevel()) {
                return nullptr;
            }
        }
        else {
            if (radList_[node->radIndex()] == std::numeric_limits<Distance>::infinity()) {
                return nullptr;
            }

            if (node->angleLevel() == planner.propagator_.MaxAngleLevel()) {
                return nullptr;
            }
        }

        ResLevels newLevels = node->levels();
        NodeIndices newIndices = node->indices();

        switch (type) {
        case SHORTER: {
            if (node->lengthLevel() == 0) {
                newIndices[1] += 1;
            }
            else {
                newIndices[1] = newIndices[1] * 2 + 1;
            }

            newLevels[0]++;
            break;
        }

        case LONGER: {
            if (node->lengthLevel() == 0) {
                return nullptr;
            }

            newIndices[1] *= 2;
            newLevels[0]++;
            break;
        }

        case LEFT: {
            if (node->angleLevel() == 0) {
                newIndices[2] += initNum_;
            }
            else {
                newIndices[2] *= 2;
            }

            newLevels[1]++;
            break;
        }

        case RIGHT: {
            if (node->angleLevel() == 0) {
                return nullptr;
            }

            newIndices[2] = newIndices[2] * 2 + 1;
            newLevels[1]++;
            break;
        }
        }

        if (node->parent()->explored(newIndices)) {
            return nullptr;
        }

        return addNewNode(planner, node->parent(), newIndices[0], newLevels[0], newLevels[1],
                          newIndices[1], newIndices[2]);
    }

    Node* addNewNode(Planner& planner, Node* parent, const unsigned& radIndex, const unsigned& lengthLevel,
                     const unsigned& angleLevel, const unsigned& lengthIndex=0, const unsigned& angleIndex=0) {
        Node* node;

        if (bin_.empty()) {
            node = nodePool_.allocate(linkTrajectory(true), parent, parent->state());
        }
        else {
            node = bin_.front();
            bin_.pop();
            node->reset(linkTrajectory(true), parent, parent->state());
        }

        node->setResolution({lengthLevel, angleLevel}, {radIndex, lengthIndex, angleIndex});
        planner.queue_.push(node);
        return node;
    }

    void recycle(Node* node) {
        bin_.push(node);
    }
};

} // namespace unc::robotics::mpt::impl::prcs

#endif // SNP_NEEDLE_PRCS_IMPL_H