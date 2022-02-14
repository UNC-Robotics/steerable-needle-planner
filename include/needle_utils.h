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
#ifndef SNP_UTILS_H
#define SNP_UTILS_H

#include <iostream>
#include <fstream>

#include <mpt/se3_space.hpp>
#include "impl/needle_config_cost_state.h"

#include "global_common.h"

namespace unc::robotics::snp {

bool InTrumpet(const Vec3& sp, const Vec3& st, const Vec3& gp, const RealNum& rad);
bool InTrumpet(const Vec3& sp, const Quat& sq, const Vec3& gp, const RealNum& rad);

RealNum RadiusOfCurvature(const Vec3& sp, const Quat& sq, const Vec3& gp);
RealNum RadiusOfCurvature(const Vec3& sp, const Vec3& st, const Vec3& gp);
RealNum RadiusOfCurvature(const Vec3& sp, const Vec3& st, const Vec3& gp, const RealNum& region_radius);
RealNum RadiusOfCurvature(const Vec3& sp, const Quat& sq, const Vec3& gp, const RealNum& region_radius);
RealNum DistanceToTrumpetBoundary(const Vec3& sp, const Vec3& st, const Vec3& gp,
        const RealNum& rad, const RealNum& orientation_tolerance=0);
RealNum MaxDistanceToTrumpetBoundary(const Vec3& sp, const Vec3& st, const Vec3& gp,
        const RealNum& rad, const RealNum& orientation_tolerance=0);
bool WorkspaceConnected(const Vec3& sp, const Quat& sq, const Vec3& gp, const Quat& gq,
                        const RealNum& rad, const RealNum& pos_tolerance=EPS);
bool WorkspaceConnected(const Vec3& sp, const Quat& sq, const Vec3& gp, const Quat& gq,
                        const RealNum& rad, const RealNum& pos_tolerance, const RealNum& ang_tolerance);

std::tuple<Vec3, Quat, RealNum> ForwardToCore(const Vec3& sp, const Quat& sq, const Vec3& gp, const RealNum& rad);

template<typename State>
State ForwardTo(const State& from, const State& to, const RealNum& rad);
template<typename State>
State ForwardTo(const State& from, const Vec3& gp, const RealNum& rad);
template<typename State>
State ForwardTo(const Vec3& sp, const Quat& sq, const Vec3& gp, const RealNum& rad);
template<typename State>
std::pair<std::vector<State>, RealNum> ForwardToWithPath(const State& from, const Vec3& gp, const RealNum& rad, const RealNum& step_size);
template<typename State>
std::tuple<std::vector<State>, RealNum, std::optional<State>> ShortestForwardToWithPath(const State& from, const Vec3& gp, const RealNum& rad,
                                                     const RealNum& step_size, const RealNum& pos_tolerance);
RealNum CurveLength(const Vec3& sp, const Quat& sq, const Vec3& gp, const Quat& gq);
template<typename State>
RealNum CurveLength(const State& from, const State& to);
template<typename State>
std::vector<State> Interpolate(const Vec3& sp, const Quat& sq, const Vec3& gp, const Quat& gq,
                               const RealNum& rad, const RealNum& step_size);
template<typename State>
std::vector<State> Interpolate(const State& from, const State& to, const RealNum& rad,
                               const RealNum& step_size);
template<typename State>
std::vector<State> AccurateInterpolate(const State& from, const State& to, const RealNum& rad,
                                       const RealNum& step_size, RealNum* remainder);
template<typename State>
std::vector<State> InterpolatePath(const std::vector<State>& path, const RealNum& rad,
                                   const RealNum& step_size);
template<typename State>
RealNum ShortestDistance(const State& from, const Vec3& to, const RealNum& rad_curv,
                         const RealNum& pos_tolerance=EPS);

RealNum DirectionDifference(const Quat& q0, const Quat& q1);
RealNum DirectionDifference(const Vec3& t0, const Vec3& t1);
template<typename State>
bool IsTheSameState(const State& a, const State& b);
template<typename State>
RealNum DirectionalDistance(const State& from, const State& to, const RealNum& rad_curv,
                            const RealNum& max_range);


template<typename State>
void PrintState(const State& s, std::ostream& out=std::cout);
template<typename State>
void PrintPosition(const State& s, std::ostream& out=std::cout);
template<typename State>
void PrintPath(const std::vector<State>& path, std::ostream& out=std::cout,
               const bool full_state=true);
template<typename State>
bool WritePathToFile(const std::vector<State>& path, const Str& file_name,
                     const bool full_state=true, const bool show_log=false);

double RelativeTime(const TimePoint& start);
double TimeDuration(const Clock::duration& elapsed);

template <typename Path>
inline void LinearInterpolate(const Vec3& sp, const Quat& sq, const Vec3& st, const RealNum& step_size,
                              const RealNum& d, Path& path, const RealNum& first_step=-1, const bool final_state=false)
{
    RealNum l = first_step > 0 ? first_step : step_size;
    for (; l < d; l += step_size) {
        path.emplace_back(sq, sp + st * l);
    }

    if (final_state && d - l > EPS) {
        path.emplace_back(sq, sp + st * d);
    }
}

template <typename Path>
inline void CurveInterpolate(const Vec3& sp, const Quat& sq, const Vec3& normal, const Vec3& center,
                             const RealNum& ang_step, const RealNum& max_ang, Path& path,
                             const RealNum& first_ang=-1, const bool final_state=false)
{
    RealNum ang = first_ang > 0 ? first_ang : ang_step;
    for (; ang < max_ang; ang += ang_step) {
        Quat q(AngleAxis(ang, normal));
        path.emplace_back((q * sq).normalized(), q * (sp - center) + center);
    }

    if (final_state && max_ang - ang > EPS) {
        Quat q(AngleAxis(max_ang, normal));
        path.emplace_back((q * sq).normalized(), q * (sp - center) + center);
    }
}

template <typename State>
struct Visitor {
    std::ofstream& out_;
    State from_;

    Visitor(std::ofstream& out) : out_(out) {}

    void vertex(const State& s) {
        const auto& p = s.translation();
        out_ << p[0] << " " << p[1] << " " << p[2] << std::endl;
    }

    void edge(const State& to) {
    }
};

} // namespace unc::robotics::snp

#include "impl/needle_utils.hpp"

#endif // SNP_UTILS_H