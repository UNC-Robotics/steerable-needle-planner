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
#ifndef SNP_NEEDLE_PLANNING_SCENARIO_H
#define SNP_NEEDLE_PLANNING_SCENARIO_H

#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <vector>

#include <Eigen/Dense>

#include <mpt/box_bounds.hpp>
#include <mpt/log.hpp>
#include <mpt/uniform_sampler.hpp>

#include "../problem_config.h"
#include "goals/needle_goal.h"
#include "goals/needle_spreading_goal.h"
#include "needle_space.h"
#include "needle_sampler.h"
#include "needle_validator.h"
#include "needle_propagator.h"

namespace unc::robotics::snp {

template <typename DistanceSpace, typename PoseSampler, typename StatePropagator, typename StateValidator, unsigned Mode>
class NeedlePlanningScenario {};

template <typename DistanceSpace, typename PoseSampler, typename StatePropagator, typename StateValidator>
class NeedlePlanningScenario<DistanceSpace, PoseSampler, StatePropagator, StateValidator, 0> {
  public:
    using Space = DistanceSpace;
    using CSampler = PoseSampler;
    using Propagator = StatePropagator;
    using Validator = StateValidator;
    using Scalar = typename Space::Distance;
    using Distance = typename Space::Distance;
    using State = typename Space::Type;
    using Goal = mpt::NeedleGoalState<Space>;
    using Bounds = std::tuple<mpt::Unbounded, mpt::BoxBounds<Scalar, 3>>;
    using Position = Eigen::Matrix<Scalar, 3, 1>;
    using Quaternion = Eigen::Quaternion<Scalar>;

  private:
    Space space_;
    Bounds bounds_;
    const unsigned radius_status_;
    Validator validator_;
    Goal goal_;

    State start_;
    State goal_state_;
    ConfigPtr cfg_;

  public:
    NeedlePlanningScenario(ConfigPtr cfg, const State& start, const State& goal)
        : bounds_(this->MakeBounds(start, goal, cfg->ins_length))
        , radius_status_(this->CheckCurvatureStatus(start, goal, cfg))
        , validator_(cfg, start, goal, radius_status_)
        , goal_(cfg, goal)
        , start_(start)
        , goal_state_(goal)
        , cfg_(cfg) {
        if (cfg->env == nullptr) {
            throw std::runtime_error("Construction of scenario failed! Config class doesn't have a valid environment!");
        }

        start_.rotation().normalize();
    }

    Bounds MakeBounds(const State& start, const State& goal, const Scalar& length) const {
        const Vec3& start_p = start.translation();
        const Vec3& goal_p = goal.translation();

        Position min, max;

        for (unsigned i = 0; i < 3; ++i) {
            min[i] = std::fmax(start_p[i], start_p[i]) - length;
            max[i] = std::fmin(start_p[i], start_p[i]) + length;
        }

        return Bounds(mpt::Unbounded{}, mpt::BoxBounds<Scalar, 3>(min, max));
    }

    unsigned CheckCurvatureStatus(const State& start, const State& goal, const ConfigPtr cfg) {
        const Vec3& start_p = start.translation();
        const Vec3& goal_p = goal.translation();

        const RealNum d = (start_p - goal_p).norm();

        unsigned status;

        if (d < cfg->dist_threshold_0) {
            status = 0;
        }
        else if (d < cfg->dist_threshold_1) {
            status = 1;
        }
        else {
            status = 2;
        }

        return status;
    }

    bool ValidProblem() {
        return validator_.ValidProblem();
    }

    ConfigPtr Config() const {
        return cfg_;
    }

    State StartState() const {
        return start_;
    }

    State GoalState() const {
        return goal_.state();
    }

    bool valid(const State& s) const {
        return validator_.Valid(s);
    }

    bool valid(const Distance& length) const {
        return validator_.ValidLength(length);
    }

    bool valid(const State& s, const Distance& length) const {
        return (validator_.ValidLength(length) && validator_.Valid(s));
    }

    bool validReachableSpace(const State& s) {
        return validator_.ValidReachableSpace(s);
    }

    bool collision(const State& s) const {
        return validator_.InCollision(s);
    }

    bool link(const State& from, const State& to) const {
        return validator_.ValidMotion(from, to);
    }

    Scalar PositionDist(const State& s1, const State& s2) const {
        return (s1.translation() - s2.translation()).norm();
    }

    Scalar CurveCost(const State& s1, const State& s2) const {
        return cfg_->env->CurveCost(s1.translation(), s1.rotation(), s2.translation(), s2.rotation(),
                                    cfg_->rad_curv, cfg_->cost_res);
    }

    Scalar FinalStateCost(const State& s) const {
        return cfg_->env->FinalStateCost(s.translation(), s.rotation(),
                                         goal_state_.translation(), goal_state_.rotation());
    }

    const Space& space() const {
        return space_;
    }

    const Bounds& bounds() const {
        return bounds_;
    }

    const Goal& goal() const {
        return goal_;
    }

    const Validator& validator() const {
        return validator_;
    }
};

template <typename DistanceSpace, typename PoseSampler, typename StatePropagator, typename StateValidator>
class NeedlePlanningScenario<DistanceSpace, PoseSampler, StatePropagator, StateValidator, 1> {
  public:
    using Space = DistanceSpace;
    using CSampler = PoseSampler;
    using Propagator = StatePropagator;
    using Validator = StateValidator;
    using Scalar = typename Space::Distance;
    using Distance = typename Space::Distance;
    using State = typename Space::Type;
    using Goal = mpt::NeedleSpreadingGoal<Space>;
    using Bounds = std::tuple<mpt::Unbounded, mpt::BoxBounds<Scalar, 3>>;
    using Position = Eigen::Matrix<Scalar, 3, 1>;
    using Quaternion = Eigen::Quaternion<Scalar>;

  private:
    Space space_;
    Bounds bounds_;
    Validator validator_;
    Goal goal_;

    State start_;
    ConfigPtr cfg_;

  public:
    NeedlePlanningScenario(ConfigPtr cfg, const State& start)
        : bounds_(this->MakeBounds(start.translation(), cfg->ins_length))
        , validator_(cfg, start)
        , goal_(cfg, start.translation())
        , start_(start)
        , cfg_(cfg) {
        if (cfg->env == nullptr) {
            throw std::runtime_error("Construction of scenario failed! Config class doesn't have a valid environment!");
        }

        start_.rotation().normalize();
    }

    Bounds MakeBounds(const Position& start, const Scalar& length) const {
        Eigen::Matrix<Scalar, 3, 1> min, max;

        for (unsigned i = 0; i < 3; ++i) {
            min[i] = start[i] - length;
            max[i] = start[i] + length;
        }

        return Bounds(mpt::Unbounded{}, mpt::BoxBounds<Scalar, 3>(min, max));
    }

    bool ValidProblem() const {
        return validator_.ValidProblem();
    }

    ConfigPtr Config() const {
        return cfg_;
    }

    State StartState() const {
        return start_;
    }

    std::optional<State> DirectConnectingStart(const State& s) const {
        return validator_.DirectConnectingStart(s);
    }

    bool valid(const State& s) const {
        return validator_.Valid(s);
    }

    bool valid(const Distance& length) const {
        return validator_.ValidLength(length);
    }

    bool valid(const State& s, const Distance& length) const {
        return validator_.Valid(s, length);
    }

    bool collision(const State& s) const {
        return validator_.InCollision(s);
    }

    bool link(const State& from, const State& to) const {
        return validator_.ValidMotion(from, to);
    }

    Scalar PositionDist(const State& s1, const State& s2) const {
        return (s1.translation() - s2.translation()).norm();
    }

    Scalar CurveCost(const State& s1, const State& s2) const {
        return cfg_->env->CurveCost(s1.translation(), s1.rotation(), s2.translation(), s2.rotation(),
                                    cfg_->rad_curv, cfg_->cost_res);
    }

    Scalar FinalStateCost(const State& s) const {
        return 0;
    }

    const Space& space() const {
        return space_;
    }

    const Bounds& bounds() const {
        return bounds_;
    }

    Goal& goal() {
        return goal_;
    }

    const Goal& goal() const {
        return goal_;
    }

    Validator& validator() {
        return validator_;
    }

    const Validator& validator() const {
        return validator_;
    }
};

} // namespace unc::robotics::snp

#endif // SNP_NEEDLE_PLANNING_SCENARIO_H