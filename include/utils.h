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

#include "global_common.h"

namespace unc::robotics::snp {

using State = unc::robotics::mpt::SE3State<RealNum>;

bool InTrumpet(const Vec3& sp, const Vec3& st, const Vec3& gp, const RealNum& rad);

bool InTrumpet(const Vec3& sp, const Quat& sq, const Vec3& gp, const RealNum& rad);

RealNum ComputeRadiusOfCurvature(const Vec3& sp, const Quat& sq, const Vec3& gp);

RealNum ComputeRadiusOfCurvature(const Vec3& sp, const Vec3& st, const Vec3& gp);

RealNum ComputeRadiusOfCurvature(const Vec3& sp, const Vec3& st, const Vec3& gp,
                                 const RealNum& region_radius);

RealNum ComputeRadiusOfCurvature(const Vec3& sp, const Quat& sq, const Vec3& gp,
                                 const RealNum& region_radius);

RealNum ComputeDistanceToTrumpetBoundary(const Vec3& sp, const Vec3& st, const Vec3& gp,
        const RealNum& rad, const RealNum& orientation_tolerance=0.0);

RealNum FarthestDistanceToTrumpetBoundary(const Vec3& sp, const Vec3& st, const Vec3& gp,
        const RealNum& rad, const RealNum& orientation_tolerance=0.0);

bool WorkspaceConnected(const Vec3& sp, const Quat& sq, const Vec3& gp, const Quat& gq,
                        const RealNum& rad, const RealNum& pos_tolerance=EPS);

bool WorkspaceConnected(const Vec3& sp, const Quat& sq, const Vec3& gp, const Quat& gq,
                        const RealNum& rad, const RealNum& pos_tolerance, const RealNum& ang_tolerance);

std::tuple<Vec3, Quat, RealNum> ForwardToCore(const Vec3& sp, const Quat& sq, const Vec3& gp,
        const RealNum& rad);

State ForwardTo(const State& from, const State& to, const RealNum& rad);

State ForwardTo(const State& from, const Vec3& gp, const RealNum& rad);

State ForwardTo(const Vec3& sp, const Quat& sq, const Vec3& gp, const RealNum& rad);

std::vector<State> Interpolate(const State& from, const State& to, const RealNum& rad,
                               const RealNum& step_size);

std::vector<State> Interpolate(const Vec3& sp, const Quat& sq, const Vec3& gp, const Quat& gq,
                               const RealNum& rad, const RealNum& step_size);

std::vector<State> AccurateInterpolate(const State& from, const State& to, const RealNum& rad,
                                       const RealNum& step_size, RealNum* remainder);

std::vector<State> InterpolatePath(const std::vector<State>& path, const RealNum& rad,
                                   const RealNum& step_size);

std::vector<State> LinearInterpolate(const Vec3& sp, const Vec3& gp, const RealNum& step_size);

RealNum CurveLength(const Vec3& sp, const Quat& sq, const Vec3& gp, const Quat& gq);

RealNum CurveLength(const State& from, const State& to);


RealNum DirectionDifference(const Quat& q0, const Quat& q1);

RealNum DirectionDifference(const Vec3& t0, const Vec3& t1);

bool IsTheSameState(const State& a, const State& b);

void PrintState(const State& s, std::ostream& out=std::cout);
void PrintPosition(const State& s, std::ostream& out=std::cout);
void PrintPath(const std::vector<State>& path, std::ostream& out=std::cout,
               const bool full_state=true);
bool WritePathToFile(const std::vector<State>& path, const Str& file_name,
                     const bool full_state=true, const bool show_log=false);

double RelativeTime(const TimePoint& start);
double TimeDuration(const Clock::duration& elapsed);

template <typename State>
struct Visitor {
    std::ofstream& out_;
    State from_;

    Visitor(std::ofstream& out) : out_(out) {}

    void vertex(const State& s) {
        const Vec3& p = s.translation();
        out_ << p[0] << " " << p[1] << " " << p[2] << std::endl;
    }

    void edge(const State& to) {
    }
};

}

#endif
