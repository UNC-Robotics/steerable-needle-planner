/// BSD 3-Clause License

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
#ifndef SNP_CONFIG_COST_STATE_H
#define SNP_CONFIG_COST_STATE_H

#include <Eigen/Dense>

namespace unc::robotics::mpt {

template <typename Scalar>
class ConfigCostState {
    using Base = std::pair<std::tuple<Eigen::Quaternion<Scalar>, Eigen::Matrix<Scalar, 3, 1>>, Scalar>;
    Base base_;

  public:
    using Distance = Scalar;

    ConfigCostState() = default;

    ConfigCostState(const Eigen::Quaternion<Scalar>& q, const Eigen::Matrix<Scalar, 3, 1>& p) {
        base_.first = {q, p};
        base_.second = 0;
    }

    ConfigCostState(const Eigen::Quaternion<Scalar>& q, const Eigen::Matrix<Scalar, 3, 1>& p, const Scalar& c) {
        base_.first = {q, p};
        base_.second = c;
    }

    Eigen::Quaternion<Scalar>& rotation() {
        return std::get<0>(base_.first);
    }

    const Eigen::Quaternion<Scalar>& rotation() const {
        return std::get<0>(base_.first);
    }

    Eigen::Matrix<Scalar, 3, 1>& translation() {
        return std::get<1>(base_.first);
    }

    const Eigen::Matrix<Scalar, 3, 1>& translation() const {
        return std::get<1>(base_.first);
    }

    Scalar& cost() {
        return base_.second;
    }

    const Scalar& cost() const {
        return base_.second;
    }

    template <typename Char, typename Traits>
    friend decltype(auto)
    operator << (std::basic_ostream<Char, Traits>& out, const ConfigCostState& q) {
        return out << "{t=[" << q.translation().transpose()
                   << "], r=[" << q.rotation().coeffs().transpose()
                   << "], c=[" << q.cost()
                   << "]}";
    }
};

} // namespace unc::robotics::mpt

#endif // SNP_CONFIG_COST_STATE_H