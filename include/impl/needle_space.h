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
#ifndef SNP_NEEDLE_SPACE_H
#define SNP_NEEDLE_SPACE_H

#include <mpt/se3_space.hpp>

#include <nigh/metric/non_metric.hpp>
#include <nigh/metric/lp.hpp>
#include <nigh/se3_space.hpp>
#include <nigh/kdtree_batch.hpp>
#include <nigh/gnat.hpp>
#include <nigh/linear.hpp>
#include <memory>
#include <cmath>

#include "../needle_utils.h"
#include "needle_config_cost_state.h"

namespace unc::robotics::nigh::metric {

template<typename Scalar>
struct Space<mpt::SE3State<Scalar>, NonMetric<0>> {
    using Type = mpt::SE3State<Scalar>;
    using Distance = Scalar;
    using Vec2 = Eigen::Matrix<Scalar, 2, 1>;
    using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
    using Quat = Eigen::Quaternion<Scalar>;
    using Metric = NonMetric<0>;
    using Limits = std::numeric_limits<Distance>;

#ifndef HAVE_GLOBAL_VARIABLES
    const Scalar rad_curv = snp::kRadCurve;
#else
    const Scalar rad_curv = snp::global::needle_min_curve_rad;
#endif

    static constexpr Scalar max_range = 1000;
    static constexpr Scalar double_range = 2000;

    const Vec2 center_of_circle = Vec2(rad_curv, 0);

    static bool isValid(const Type& s) {
        return std::isfinite(std::get<1>(s)[0])
               && std::isfinite(std::get<1>(s)[1])
               && std::isfinite(std::get<1>(s)[2])
               && std::isfinite(std::get<0>(s).coeffs()[0])
               && std::isfinite(std::get<0>(s).coeffs()[1])
               && std::isfinite(std::get<0>(s).coeffs()[2])
               && std::isfinite(std::get<0>(s).coeffs()[3]);
    }

    constexpr unsigned dimensions() const {
        return 6;
    }

    Distance distance(const Type& from, const Type& to) const {
        const Vec3& sp = from.translation();
        const Quat sq = from.rotation().normalized();
        const Vec3& gp = to.translation();

        const Vec3 sg = gp - sp;
        const Vec3 tang = (sq*Vec3::UnitZ()).normalized();

        Distance y = sg.dot(tang);

        if (y < 0) {
            return double_range;
        }

        Distance d = sg.norm();
        Distance x = d * std::sin(std::acos(y/d));

        Distance dist_to_center = (Vec2(x, y) - center_of_circle).norm();

        if (dist_to_center < rad_curv) {
            return (rad_curv - dist_to_center + max_range);
        }

        Scalar alpha = std::acos(std::fmin(1, rad_curv/dist_to_center));
        Scalar beta = std::asin(y/dist_to_center);

        Scalar ang = x < rad_curv ? (beta - alpha) : (M_PI - beta - alpha);
        Distance straight_portion = dist_to_center * std::sin(alpha);
        Distance curve_portion = ang * rad_curv;

        return curve_portion + straight_portion;
    }
};

template<typename Scalar>
struct Space<mpt::ConfigCostState<Scalar>, NonMetric<1>> {
    using Type = mpt::ConfigCostState<Scalar>;
    using Distance = Scalar;
    using Vec2 = Eigen::Matrix<Scalar, 2, 1>;
    using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
    using Quat = Eigen::Quaternion<Scalar>;
    using Metric = NonMetric<3>;

#ifndef HAVE_GLOBAL_VARIABLES
    const Scalar cost_w = 1.0;
#else
    const Scalar cost_w = snp::global::aorrt_cost_w;
#endif

    static bool isValid(const Type& aug_s) {
        auto const& s = aug_s.first;
        auto const& cost = aug_s.second;
        return cost >= 0 && std::isfinite(cost)
               && std::isfinite(std::get<1>(s)[0])
               && std::isfinite(std::get<1>(s)[1])
               && std::isfinite(std::get<1>(s)[2])
               && std::isfinite(std::get<0>(s).coeffs()[0])
               && std::isfinite(std::get<0>(s).coeffs()[1])
               && std::isfinite(std::get<0>(s).coeffs()[2])
               && std::isfinite(std::get<0>(s).coeffs()[3]);
    }

    constexpr unsigned dimensions() const {
        return 7;
    }

    Distance distance(const Type& from, const Type& to) const {
        auto const& p0 = from.translation();
        auto const& p1 = to.translation();
        auto const& q0 = from.rotation();
        auto const& q1 = to.rotation();
        auto const& c0 = from.cost();
        auto const& c1 = to.cost();

        const Distance& trans_dist = NORM2_T_W * (p0 - p1).norm();
        const Distance& rot_dist = NORM2_R_W * std::sqrt(std::pow(q0.w() - q1.w(), 2)
                                 + std::pow(q0.x() - q1.x(), 2)
                                 + std::pow(q0.y() - q1.y(), 2)
                                 + std::pow(q0.z() - q1.z(), 2));
        const Distance& cost_dist = cost_w * std::abs(c0 - c1);

        return std::sqrt(std::pow(trans_dist, 2) + std::pow(rot_dist, 2) + std::pow(cost_dist, 2));
    }
};

} // namespace unc::robotics::nigh::metric

namespace unc::robotics::snp {

template <typename Scalar>
using NeedleSpace = nigh::metric::Space<mpt::SE3State<Scalar>, nigh::metric::NonMetric<0>>;

template <typename Scalar>
using ConfigCostSpace = nigh::metric::Space<mpt::ConfigCostState<Scalar>, nigh::metric::NonMetric<1>>;

template <typename Scalar>
using WeightedSE3Space = mpt::SE3Space<Scalar, SE3_T_W, SE3_R_W>;

template <typename Scalar, typename Space>
struct nn_select {};

template <typename Scalar>
struct nn_select<Scalar, mpt::SE3Space<Scalar>> {
    using type = nigh::KDTreeBatch<>;
};

template <typename Scalar>
struct nn_select<Scalar, NeedleSpace<Scalar>> {
    using type = nigh::Linear;
};

template <typename Scalar>
struct nn_select<Scalar, WeightedSE3Space<Scalar>> {
    using type = nigh::KDTreeBatch<>;
};

template <typename Scalar>
struct nn_select<Scalar, ConfigCostSpace<Scalar>> {
    using type = nigh::Linear;
};

} // namespace unc::robotics::snp

#endif // SNP_NEEDLE_SPACE_H