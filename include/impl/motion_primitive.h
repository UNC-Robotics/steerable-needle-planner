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
#ifndef SNP_MOTION_PRIMITIVE_H
#define SNP_MOTION_PRIMITIVE_H

#include <cmath>

#include "global_common.h"

namespace unc::robotics::snp {

const Vec3 kOrgP(0, 0, 0);
const Quat kOrgQ(1, 0, 0, 0);

template<typename State>
class MotionPrimitive {
  public:
    const RealNum length_;
    const RealNum rad_curv_;

    MotionPrimitive(const RealNum& l, const RealNum& rad, const RealNum& res)
        : length_(l), rad_curv_(rad) {
        State origin;
        origin.translation() = kOrgP;
        origin.rotation() = kOrgQ;
        states_ = this->SteerFrom(origin, length_, rad_curv_, res);
    }

    MotionPrimitive(const MotionPrimitive& other)
        : length_(other.length_), rad_curv_(other.rad_curv_) {
        states_ = other.states_;
    }

    const std::vector<State>& States() const {
        return states_;
    }

    const State& FinalState() const {
        return states_.back();
    }

  private:
    std::vector<State> states_;

    static std::vector<State> SteerFrom(const State& s, const RealNum& length,
                                        const RealNum& rad, const RealNum& step_size) {
        const Vec3& p = s.translation();
        const Quat q = (s.rotation()).normalized();

        State tmp;
        std::vector<State> states;

        if (rad == R_INF) {
            const Vec3 tang = (q*Vec3::UnitZ()).normalized();

            for (RealNum l = step_size; l < length; l += step_size) {
                tmp.translation() = p + l*tang;
                tmp.rotation() = q;
                states.emplace_back(tmp);
            }

            tmp.translation() = p + length*tang;
            tmp.rotation() = q;
            states.emplace_back(tmp);
        }
        else {
            const Vec3 center = p + rad*(q*Vec3::UnitX());
            const RealNum max_ang = length/rad;
            const RealNum angle_step = step_size/rad;

            const Vec3 normal_vec = (q*Vec3::UnitY()).normalized();

            for (RealNum ang = angle_step; ang < max_ang; ang += angle_step) {
                Quat proceed_quat(AngleAxis(ang, normal_vec));
                tmp.translation() = proceed_quat*(p - center) + center;
                tmp.rotation() = (proceed_quat*q).normalized();
                states.emplace_back(tmp);
            }

            Quat proceed_quat(AngleAxis(max_ang, normal_vec));
            tmp.translation() = proceed_quat*(p - center) + center;
            tmp.rotation() = (proceed_quat*q).normalized();
            states.emplace_back(tmp);
        }

        return states;
    }
};

} // unc::robotics::snp

#endif // SNP_MOTION_PRIMITIVE_H