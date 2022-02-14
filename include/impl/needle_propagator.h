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
#ifndef SNP_NEEDLE_PROPAGATOR_H
#define SNP_NEEDLE_PROPAGATOR_H

#include <cmath>

#include "motion_primitive.h"
#include "../problem_config.h"
#include "../needle_utils.h"

namespace unc::robotics::snp {

namespace utils {

template <typename State, typename RNG, typename Normal>
std::optional<State> ConnectPointWithCurveDirectly(const State& from, const State& to, RNG& rng,
        Normal& normal_dist, const RealNum& rad_curv, const RealNum& steer_step) {
    State result;

    const Vec3& sp = from.translation();
    const Quat sq = from.rotation().normalized();
    const Vec3 st = (sq*Vec3::UnitZ()).normalized();
    const Vec3& gp = to.translation();

    const Vec3 sg = gp - sp;
    const RealNum d = sg.norm();
    const RealNum cos_theta = (sg.normalized()).dot(st);

    if (d < EPS || cos_theta > 1 - EPS) {
        result.translation() = from.translation() + st*d;
        result.rotation() = sq;
        return result;
    }

    RealNum r = rad_curv;

    if (cos_theta > 0) {
        r = std::fmax(rad_curv, 0.5*d/std::sin(std::acos(cos_theta)));
    }

    const Vec3 normal = (st.cross(sg)).normalized();
    const Vec3 center = sp + r*(normal.cross(st));
    const RealNum max_ang = std::acos(((gp - center).normalized()).dot((sp - center).normalized()));

    RealNum proceed_ang = max_ang;

    if (steer_step > 0) {
        RealNum portion = 1 - abs(normal_dist(rng));

        while (portion < EPS) {
            portion = 1 - abs(normal_dist(rng));
        }

        proceed_ang = std::fmin(steer_step*portion/r, max_ang);
    }

    const Quat proceed_quat(AngleAxis(proceed_ang, normal));

    result.translation() = proceed_quat*(sp - center) + center;
    result.rotation() = (proceed_quat*sq).normalized();

    if (std::isnan(result.rotation().w())) {
        throw std::runtime_error("[ConnectPointWithCurveDirectly] Get nan result quaternion!");
    }

    return result;
}

template<typename State>
State TransformToNewBase(const State& state, const State& new_base) {
    const Vec3& p = new_base.translation();
    const Quat q = new_base.rotation().normalized();

    State transformed;
    transformed.translation() = q * state.translation() + p;
    transformed.rotation() = (q * state.rotation()).normalized();

    return transformed;
}

Quat RotateAroundZ(Quat init_q, const RealNum& angle) {
    init_q.normalize();
    Quat proceed_quat(AngleAxis(angle, (init_q*Vec3::UnitZ()).normalized()));

    return (proceed_quat * init_q).normalized();
}

template <typename State, typename RNG, typename Uniform>
std::optional<State> RandomForward(const State& from, const State& to, RNG& rng, Uniform& uniform_dist,
                                   const RealNum& rad_curv, const RealNum& steer_step, const Idx& num_attempt,
                                   const bool& return_first) {
    Vec3 p = from.translation();
    Quat q = from.rotation().normalized();

    State result(q, p);

    RealNum theta = 2 * M_PI * uniform_dist(rng);
    RealNum ell = steer_step * uniform_dist(rng);

    q = RotateAroundZ(q, theta);

    const Vec3 center = p + rad_curv*(q*Vec3::UnitX());
    const RealNum max_ang = ell/rad_curv;
    const Vec3 normal_vec = q*Vec3::UnitY();

    Quat proceed_quat(AngleAxis(max_ang, normal_vec));
    result.translation() = proceed_quat*(p - center) + center;
    result.rotation() = (proceed_quat*q).normalized();

    return result;
}

} // namespace utils

template <typename State>
class CurvePropagator {
  public:
    CurvePropagator(const ConfigPtr cfg)
        : rad_curv_(cfg->rad_curv)
        , steer_step_(cfg->steer_step) {
    }

    template <typename RNG>
    std::optional<State> operator()(const State& from, const State& to, RNG& rng) {
        return utils::ConnectPointWithCurveDirectly(from, to, rng, normal_, rad_curv_, steer_step_);
    }

  private:
    const RealNum rad_curv_;
    const RealNum steer_step_;

    RealNormalDist normal_;
};

template <typename State>
class RandomForwardPropagator {
  public:
    RandomForwardPropagator(const ConfigPtr cfg)
        : rad_curv_(cfg->rad_curv)
        , steer_step_(cfg->steer_step) {
    }

    template <typename RNG>
    std::optional<State> operator()(const State& from, const State& to, RNG& rng) {
        return utils::RandomForward(from, to, rng, uniform_, rad_curv_, steer_step_, num_attempt_, false);
    }

  private:
    const RealNum rad_curv_;
    const RealNum steer_step_;
    const Idx num_attempt_{10};

    RealUniformDist uniform_;
};

template<typename State>
class MotionPrimitivePropagator {
  public:
    MotionPrimitivePropagator(const ConfigPtr cfg)
        : rad_curv_(cfg->rad_curv)
        , delta_ell_max_(cfg->delta_ell_max)
        , delta_theta_max_(cfg->delta_theta_max)
        , delta_ell_min_(cfg->delta_ell_min)
        , delta_theta_min_(cfg->delta_theta_min) {
        rad_sequence_.assign({rad_curv_, R_INF});

        max_length_i_ = std::ceil(std::log2(delta_ell_max_/delta_ell_min_));
        max_angle_i_ = std::ceil(std::log2(delta_theta_max_/delta_theta_min_));
        min_length_step_ = delta_ell_max_/(std::pow(2, max_length_i_));
        min_angle_step_ = delta_theta_max_/(std::pow(2, max_angle_i_));
        max_depth_ = std::ceil(cfg->ins_length/min_length_step_);

        length_sequence_.resize(std::pow(2, max_length_i_));
        length_sequence_[0] = delta_ell_max_;
        length_sequence_[1] = 0.5 * length_sequence_[0];

        for (unsigned i = 2; i < length_sequence_.size(); i += 2) {
            const unsigned parent_i = i/2;
            const RealNum d_length = delta_ell_max_/std::pow(2, std::floor(std::log2(i)) + 1);
            length_sequence_[i] = length_sequence_[parent_i] + d_length;
            length_sequence_[i+1] = length_sequence_[parent_i] - d_length;
        }

        init_num_orientations_ = std::round(2 * M_PI/delta_theta_max_);
        angle_sequence_.resize(std::pow(2, max_angle_i_) * init_num_orientations_);

        RealNum ang = 0;

        for (unsigned i = 0; i < init_num_orientations_; ++i) {
            angle_sequence_[i] = ang;
            angle_sequence_[i + init_num_orientations_] = ang + 0.5 * delta_theta_max_;
            ang += delta_theta_max_;
        }

        for (unsigned i = 2*init_num_orientations_; i < angle_sequence_.size(); i += 2) {
            unsigned parent_i = i/2;
            RealNum d_angle = delta_theta_max_/std::pow(2, std::floor(std::log2(i)) - 1);
            angle_sequence_[i] = angle_sequence_[parent_i] + d_angle;
            angle_sequence_[i+1] = angle_sequence_[parent_i] - d_angle;
        }

        motion_primitives_.resize(rad_sequence_.size());

        for (unsigned r = 0; r < rad_sequence_.size(); ++r) {
            const RealNum rad = rad_sequence_[r];

            for (unsigned i = motion_primitives_[r].size(); i < length_sequence_.size(); ++i) {
                motion_primitives_[r].emplace_back(length_sequence_[i], rad, cfg->validity_res);
            }
        }
    }

    std::optional<State> operator()(const State& from, const std::array<unsigned, 3>& indices) const {
        auto const& base_state = motion_primitives_[indices[0]][indices[1]].FinalState();

        return utils::TransformToNewBase(base_state, this->ComputeStartPose(from, indices[2]));
    }

    std::optional<State> operator()(const State& from, const unsigned& rad_idx,
                                    const unsigned& length_idx) const {
        auto const& base_state = motion_primitives_[rad_idx][length_idx].FinalState();

        return utils::TransformToNewBase(base_state, from);
    }

    State ComputeStartPose(const State& from, const Idx& angle_idx) const {
        State rotated_base = from;
        rotated_base.rotation() = utils::RotateAroundZ(from.rotation(), angle_sequence_[angle_idx]);

        return rotated_base;
    }

    const std::vector<State>& BaseMotion(const Idx& rad_idx, const Idx& length_idx) const {
        return motion_primitives_[rad_idx][length_idx].States();
    }

    const RealNum& RadiusOfCurvature(const Idx& rad_idx) const {
        return rad_sequence_[rad_idx];
    }

    const std::vector<RealNum>& RadCurvList() const {
        return rad_sequence_;
    }

    const RealNum& Length(const Idx& length_idx) const {
        return length_sequence_[length_idx];
    }

    const RealNum& AngleDiff(const Idx& angle_idx) const {
        return angle_sequence_[angle_idx];
    }

    const Idx& InitialNumberofOrientations() const {
        return init_num_orientations_;
    }

    const Idx& MaxLengthLevel() const {
        return max_length_i_;
    }

    const Idx& MaxAngleLevel() const {
        return max_angle_i_;
    }

    Idx MaxLengthIndex() const {
        return length_sequence_.size();
    }

    Idx MaxAngleIndex() const {
        return angle_sequence_.size();
    }

  private:
    const RealNum rad_curv_;
    const RealNum delta_ell_max_;
    const RealNum delta_theta_max_;
    const RealNum delta_ell_min_;
    const RealNum delta_theta_min_;

    Idx max_length_i_;
    Idx max_angle_i_;
    RealNum min_length_step_;
    RealNum min_angle_step_;
    Idx max_depth_;
    Idx init_num_orientations_;

    std::vector<RealNum> rad_sequence_;
    std::vector<RealNum> length_sequence_;
    std::vector<RealNum> angle_sequence_;

    std::vector<std::vector<MotionPrimitive<State>>> motion_primitives_;
};

} // end namespace unc::robotics::snp

#endif // SNP_NEEDLE_PROPAGATOR_H