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
#ifndef SNP_NEEDLE_GOAL_STATE_H
#define SNP_NEEDLE_GOAL_STATE_H

#include <mpt/goal_sampler.hpp>
#include <utility>

#include "../utils.h"

namespace unc::robotics::snp {

struct RandomNumber {
    std::mt19937_64 rng;
    RealUniformDist uniform;

    RandomNumber() {
        rng.seed(1);
        uniform = RealUniformDist(0, 1);
    }

    RealNum Generate(const RealNum low=0, const RealNum high=1) {
        return uniform(rng)*(high - low) + low;
    }
};

using RandPtr = std::shared_ptr<RandomNumber>;

}

namespace unc::robotics::mpt {
using namespace snp;

template <typename Space>
class NeedleGoalState {
    using State = typename Space::Type;
    using Distance = typename Space::Distance;
    using Scalar = typename Space::Distance;

    State goal_;
    Eigen::Matrix<Scalar, 3, 1> goal_p_;
    Eigen::Quaternion<Scalar> goal_q_;
    ConfigPtr cfg_;
    RandPtr rand_;

  public:
    template <typename ... Args>
    NeedleGoalState(ConfigPtr cfg, Args&& ... args)
        : goal_(std::forward<Args>(args)...),
          cfg_(cfg) {
        goal_p_ = goal_.translation();
        goal_q_ = goal_.rotation().normalized();
        rand_.reset(new RandomNumber());
    }

    const State& state() const {
        return goal_;
    }

    std::tuple<bool, Distance, State> operator() (const Space& space, const State& s) const {
        if (cfg_->constrain_goal_orientation) {
            return this->GoalCheckWithOrientation(space, s);
        }

        return this->GoalCheck(space, s);
    }

  private:
    bool Valid(const State& s) const {
        return cfg_->env->CollisionFree(s.translation());
    }

    std::tuple<bool, Distance, State> GoalCheck(const Space& space, const State& s) const {
        State goal_state;
        const Vec3& p = s.translation();
        const RealNum d = (p - goal_p_).norm();

        if (d > cfg_->goal_connecting_rad && rand_->Generate() > cfg_->direct_connect_ratio) {
            return {false, R_INF, goal_state};
        }

        goal_state = ForwardTo(s, goal_, cfg_->rad_curv);
        const Distance dist = (goal_state.translation() - goal_p_).norm();

        if (dist < cfg_->goal_pos_tolerance) {
            std::vector<State> states = Interpolate(s, goal_state, cfg_->rad_curv,
                                                    cfg_->validity_res);
            states.push_back(goal_state);

            State last_valid_state = s;

            for (const State& state : states) {
                if (!this->Valid(state)) {
                    const Distance valid_dist = (last_valid_state.translation() - goal_p_).norm();

                    if (valid_dist < cfg_->goal_pos_tolerance) {
                        return {true, valid_dist, last_valid_state};
                    }
                    else {
                        return {false, R_INF, goal_state};
                    }
                }

                last_valid_state = state;
            }

            return {true, dist, goal_state};
        }

        return {false, R_INF, goal_state};
    }

    std::tuple<bool, Distance, State> GoalCheckWithOrientation(const Space& space,
            const State& s) const {
        State goal_state;
        const Vec3& p = s.translation();
        const RealNum d = (p - goal_p_).norm();

        if (d > cfg_->goal_connecting_rad && rand_->Generate() > cfg_->direct_connect_ratio) {
            return {false, R_INF, goal_state};
        }

        goal_state = ForwardTo(s, goal_, cfg_->rad_curv);
        const Distance dist = (goal_state.translation() - goal_p_).norm();
        const Distance ang_diff = DirectionDifference(goal_state.rotation(), goal_q_);

        if (dist < cfg_->goal_pos_tolerance) {
            std::vector<State> states = Interpolate(s, goal_state, cfg_->rad_curv,
                                                    cfg_->validity_res);
            states.push_back(goal_state);

            State last_valid_state = s;

            for (const State& state : states) {
                if (!this->Valid(state)) {
                    const Distance valid_dist = (last_valid_state.translation() - goal_p_).norm();

                    if (valid_dist < cfg_->goal_pos_tolerance) {
                        const Distance valid_ang_diff = DirectionDifference(last_valid_state.rotation(), goal_q_);

                        if (valid_ang_diff < cfg_->goal_ang_tolerance) {
                            return {true, valid_dist + valid_ang_diff, last_valid_state};
                        }
                        else {
                            return {false, valid_dist + valid_ang_diff, last_valid_state};
                        }
                    }
                    else {
                        return {false, R_INF, goal_state};
                    }
                }

                last_valid_state = state;
            }

            if (ang_diff < cfg_->goal_ang_tolerance) {
                return {true, dist + ang_diff, goal_state};
            }
            else {
                return {false, dist + ang_diff, goal_state};
            }
        }

        return {false, R_INF, goal_state};
    }
};

template <typename Space>
class GoalSampler<NeedleGoalState<Space>> {
    const NeedleGoalState<Space>& goal_;

  public:
    using Type = typename Space::Type;

    GoalSampler(const NeedleGoalState<Space>& goal)
        : goal_(goal) {
    }

    template <typename RNG>
    Type operator() (RNG&) const {
        return goal_.state();
    }
};

}

#endif
