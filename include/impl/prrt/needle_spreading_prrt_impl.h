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

//! @author Jeff Ichnowski

#pragma once
#ifndef SNP_NEEDLE_SPREADING_PRRT_IMPL_H
#define SNP_NEEDLE_SPREADING_PRRT_IMPL_H

#include "prrt_base.hpp"

namespace unc::robotics::mpt::impl::prrt {
template <typename Scenario, int maxThreads, bool reportStats, typename NNStrategy>
class NeedleSpreadingPRRT : public
    PlannerBase<NeedleSpreadingPRRT<Scenario, maxThreads, reportStats, NNStrategy>> {
  public:
    using Space = scenario_space_t<Scenario>;
    using State = typename Space::Type;

  private:
    using Planner = NeedleSpreadingPRRT;
    using Base = PlannerBase<Planner>;
    using Distance = typename Space::Distance;
    using Link = scenario_link_t<Scenario>;
    using Traj = link_trajectory_t<Link>;
    using Node = prrt::Node<State, Traj>;
    using Edge = prrt::Edge<State, Traj>;
    using RNG = scenario_rng_t<Scenario, Distance>;
    using Sampler = scenario_sampler_t<Scenario, RNG>;
    using Point = typename Scenario::Position;

    Distance maxDistance_{std::numeric_limits<Distance>::infinity()};
    Distance goalBias_{0.01};

    static constexpr bool concurrent = maxThreads != 1;
    using NNConcurrency = std::conditional_t<concurrent, nigh::Concurrent, nigh::NoThreadSafety>;
    nigh::Nigh<Node*, Space, NodeKey, NNConcurrency, NNStrategy> nn_;

    std::mutex mutex_;
    std::forward_list<Node*> goals_;

    Node* approxRes_{nullptr};
    Distance bestDist_{std::numeric_limits<Distance>::infinity()};

    Atom<std::size_t, concurrent> goalCount_{0};

    ObjectPool<Node, false> startNodes_;

    struct Worker;

    WorkerPool<Worker, maxThreads> workers_;

    void foundGoal(Node* node) {
        if constexpr (reportStats) {
            MPT_LOG(INFO) << "found solution";
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            goals_.push_front(node);
        }

        ++goalCount_;
    }

    void foundApproxGoal(Node* node, Distance* dist) {
        {
            std::lock_guard<std::mutex> lock(mutex_);

            if (*dist < bestDist_) {
                MPT_LOG(INFO) << "update approximate solution with dist " << *dist;
                bestDist_ = *dist;
                approxRes_ = node;
            }
            else if (*dist > bestDist_) {
                *dist = bestDist_;
            }
        }
    }

  public:
    template <typename RNGSeed = RandomDeviceSeed<>>
    explicit NeedleSpreadingPRRT(const Scenario& scenario = Scenario(), const RNGSeed& seed = RNGSeed())
        : nn_(scenario.space())
        , workers_(scenario, seed) {
        MPT_LOG(TRACE) << "Using nearest: " << log::type_name<NNStrategy>();
        MPT_LOG(TRACE) << "Using concurrency: " << log::type_name<NNConcurrency>();
        MPT_LOG(TRACE) << "Using sampler: " << log::type_name<Sampler>();
    }

    void setGoalBias(Distance bias) {
        assert(0 <= bias && bias <= 1);
        goalBias_ = bias;
    }

    Distance getGoalBias() const {
        return goalBias_;
    }

    void setRange(Distance range) {
        assert(range > 0);
        maxDistance_ = range;
    }

    Distance getRange() const {
        return maxDistance_;
    }

    std::size_t size() const {
        return nn_.size();
    }

    template <typename ... Args>
    void addStart(Args&& ... args) {
        std::lock_guard<std::mutex> lock(mutex_);
        Node* node = startNodes_.allocate(Traj{}, nullptr, std::forward<Args>(args)...);
        nn_.insert(node);
    }

    using Base::solveFor;
    using Base::solveUntil;

    template <typename DoneFn>
    std::enable_if_t<std::is_same_v<bool, std::result_of_t<DoneFn()>>>
    solve(DoneFn doneFn) {
        if (size() == 0) {
            throw std::runtime_error("there are no valid initial states");
        }

        workers_.solve(*this, doneFn);
    }

    bool solved() const {
        return goalCount_.load(std::memory_order_relaxed);
    }

    bool approxSolved() const {
        return (approxRes_ != nullptr);
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

  private:
    template <typename Visitor, typename Nodes>
    void visitNodes(Visitor&& visitor, const Nodes& nodes) const {
        for (const Node& n : nodes) {
            visitor.vertex(n.state());

            if (n.parent()) {
                visitor.edge(n.parent()->state());
            }
        }
    }

  public:
    template <typename Visitor>
    void visitGraph(Visitor&& visitor) const {
        visitNodes(std::forward<Visitor>(visitor), startNodes_);

        for (const Worker& w : workers_) {
            visitNodes(std::forward<Visitor>(visitor), w.nodes());
        }
    }
};

template <typename Scenario, int maxThreads, bool reportStats, typename NNStrategy>
class NeedleSpreadingPRRT<Scenario, maxThreads, reportStats, NNStrategy>::Worker
    : public WorkerStats<reportStats> {
    using Stats = WorkerStats<reportStats>;
    using CSampler = typename Scenario::CSampler;
    using Propagator = typename Scenario::Propagator;

    unsigned no_;
    Scenario scenario_;
    RNG rng_;
    std::uniform_real_distribution<Distance> uniform01_;

    ObjectPool<Node> nodePool_;

    CSampler csampler_;
    Propagator propagator_;
    Distance bestDist_{std::numeric_limits<Distance>::infinity()};

    State start_;

  public:
    Worker(Worker&& other)
        : no_(other.no_)
        , scenario_(other.scenario_)
        , rng_(other.rng_)
        , nodePool_(std::move(other.nodePool_))
        , csampler_(scenario_.Config(), scenario_.StartState())
        , propagator_(scenario_.Config()) {

    }

    template <typename RNGSeed>
    Worker(unsigned no, const Scenario& scenario, const RNGSeed& seed)
        : no_(no)
        , scenario_(scenario)
        , rng_(seed)
        , csampler_(scenario.Config(), scenario.StartState())
        , propagator_(scenario_.Config()) {
    }

    decltype(auto) space() const {
        return scenario_.space();
    }

    decltype(auto) scenario() const {
        return scenario_;
    }

    const auto& nodes() const {
        return nodePool_;
    }

    template <typename DoneFn>
    void solve(Planner& planner, DoneFn done) {
        MPT_LOG(TRACE) << "worker running";

        if constexpr (scenario_has_goal_sampler_v<Scenario, RNG>) {
            if (no_ == 0 && planner.goalBias_ > 0) {
                scenario_goal_sampler_t<Scenario, RNG> goalSampler(scenario_);
                Distance scaledBias = planner.goalBias_ * planner.workers_.size();

                MPT_LOG(TRACE) << "using scaled goal bias of " << scaledBias;

                while (!done()) {
                    Stats::countIteration();

                    if (planner.goalCount_.load(std::memory_order_relaxed) >= 1) {
                        goto unbiasedSamplingLoop;
                    }

                    if (uniform01_(rng_) < scaledBias) {
                        Stats::countBiasedSample();
                        addSample(planner, goalSampler(rng_));
                    }
                    else {
                        addSample(planner, csampler_(rng_));
                    }
                }

                return;
            }
        }

unbiasedSamplingLoop:

        while (!done()) {
            Stats::countIteration();
            addSample(planner, csampler_(rng_));
        }

        MPT_LOG(TRACE) << "worker done";
    }

    void addSample(Planner& planner, std::optional<State>&& sample) {
        if (sample) {
            addSample(planner, *sample);
        }
    }

    decltype(auto) nearest(Planner& planner, const State& state) {
        Timer timer(Stats::nearest());
        return planner.nn_.nearest(state);
    }

    void addSample(Planner& planner, State& randState) {
        if (scenario_.collision(randState)) {
            return;
        }

        auto [nearNode, d] = nearest(planner, randState).value();

        State newState = randState;

        if (scenario_.PositionDist(nearNode->state(), randState) < snp::EPS) {
            return;
        }

        if (uniform01_(rng_) < scenario_.Config()->start_connect_ratio) {
            auto startState = scenario_.DirectConnectingStart(randState);

            if (startState) {
                planner.addStart(*startState);
            }
        }

        auto propagated = propagator_(nearNode->state(), randState, rng_);

        if (!propagated) {
            return;
        }

        newState = *propagated;

        auto const& newLength = nearNode->length() + snp::CurveLength(nearNode->state(), newState);

        if (!scenario_.valid(newState, newLength)) {
            return;
        }

        if (auto traj = validMotion(nearNode->state(), newState)) {
            auto [isGoal, goalDist, goalState] = scenario_goal<Scenario>::check(scenario_, newState);

            Node* newNode = nodePool_.allocate(linkTrajectory(traj), nearNode, newState);
            newNode->length() = newLength;
            planner.nn_.insert(newNode);

            if (isGoal) {
                auto const& goalLength = newLength + snp::CurveLength(newState, goalState);

                if (!scenario_.valid(goalLength)) {
                    return;
                }

                if (snp::IsTheSameState(goalState, newState)) {
                    planner.foundGoal(newNode);
                }
                else {
                    Node* goalNode = nodePool_.allocate(linkTrajectory(traj), newNode, goalState);
                    goalNode->length() = goalLength;
                    planner.foundGoal(goalNode);
                }
            }
            else if (!planner.solved() && goalDist < bestDist_) {
                auto const& goalLength = newLength + snp::CurveLength(newState, goalState);

                if (!scenario_.valid(goalLength)) {
                    return;
                }

                bestDist_ = goalDist;
                planner.foundApproxGoal(newNode, &bestDist_);
            }
        }
    }

    decltype(auto) validMotion(const State& a, const State& b) {
        Timer timer(Stats::validMotion());
        return scenario_.link(a, b);
    }
};

} // namespace unc::robotics::mpt::impl::prrt

#endif // SNP_NEEDLE_SPREADING_PRRT_IMPL_H