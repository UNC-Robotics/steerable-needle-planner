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
#ifndef SNP_NEEDLE_SPREADING_GOAL_H
#define SNP_NEEDLE_SPREADING_GOAL_H

#include <mpt/goal_sampler.hpp>
#include <utility>

#include "../../needle_utils.h"

namespace unc::robotics::mpt {
using namespace snp;

template <typename Space>
class NeedleSpreadingGoal {
    using State = typename Space::Type;
    using Distance = typename Space::Distance;
    using Scalar = typename Space::Distance;

    ConfigPtr cfg_;
    Vec3 start_p_;
    std::vector<Vec3> goals_;
    Distance min_dist_{60.0};

  public:
    template <typename ... Args>
    NeedleSpreadingGoal(ConfigPtr cfg, const Vec3& start_p, Args&& ... args)
        : cfg_(cfg), start_p_(start_p), min_dist_{cfg->spreading_min_dist} {
    }

    void ProvideGoalPoints(const std::vector<Vec3>& goals) {
        goals_ = goals;
    }

    unsigned size() const {
        return goals_.size();
    }

    const Vec3& goal(const unsigned& idx) const {
        return goals_[idx];
    }

    const Distance& length() const {
        return cfg_->ins_length;
    }

    std::tuple<bool, Distance, State> operator() (const Space& space, const State& s) const {
        const Vec3& pos = s.translation();

        if (goals_.size() > 0) {
            Distance dist = R_INF;

            for (auto const& p : goals_) {
                dist = std::min((pos - p).norm(), dist);
            }

            if (dist < cfg_->goal_pos_tolerance) {
                return {true, dist, s};
            }

            return {false, dist, s};
        }

        if ((pos - start_p_).norm() > min_dist_) {
            return {true, 1/((pos - start_p_).norm() + 1e-3), s};
        }

        return {false, 1/((pos - start_p_).norm() + 1e-3), s};
    }
};

template <typename Space>
class GoalSampler<NeedleSpreadingGoal<Space>> {
    const NeedleSpreadingGoal<Space>& goal_;
    std::uniform_real_distribution<typename Space::Distance> uniform_;
    std::normal_distribution<typename Space::Distance> normal_;

  public:
    using Type = typename Space::Type;

    GoalSampler(const NeedleSpreadingGoal<Space>& goal)
        : goal_(goal) {
    }

    template <typename RNG>
    Type operator() (RNG& rng) {
        Type goal;

        if (goal_.size() == 0) {
            Vec3 result;
            result[0] = normal_(rng);
            result[1] = normal_(rng);
            result[2] = normal_(rng);
            result *= goal_.length()/result.norm();

            goal.translation() = result;
        }
        else {
            const unsigned& rand_idx = goal_.size() * uniform_(rng);
            goal.translation() = goal_.goal(rand_idx);
        }

        return goal;
    }
};

} // namespace unc::robotics::mpt

#endif // SNP_NEEDLE_SPREADING_GOAL_H