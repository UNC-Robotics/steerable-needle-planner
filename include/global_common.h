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
#ifndef SNP_GLOBAL_COMMON_H
#define SNP_GLOBAL_COMMON_H

#include <cmath>
#include <chrono>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>
#include <map>

#include <boost/multi_array.hpp>
#include <Eigen/Dense>

namespace unc::robotics::snp {

#define SE3_T_W 20
#define SE3_R_W 1

using Str = std::string;
using Idx = unsigned short;
using SizeType = std::size_t;
using RealNum = double;
using RealUniformDist = std::uniform_real_distribution<RealNum>;
using RealNormalDist = std::normal_distribution<RealNum>;
using IntUniformDist = std::uniform_int_distribution<int>;
const Idx I_INF = std::numeric_limits<Idx>::max();
const RealNum R_INF = std::numeric_limits<RealNum>::infinity();
const RealNum EPS = 1e-6;
const RealNum DEGREE_TO_RAD = M_PI/180.0;
const RealNum RAD_TO_DEGREE = 180.0/M_PI;

using IdxPoint = Eigen::Matrix<Idx, 3, 1, Eigen::ColMajor>;
using Vec2 = Eigen::Matrix<RealNum, 2, 1, Eigen::ColMajor>;
using Vec3 = Eigen::Matrix<RealNum, 3, 1, Eigen::ColMajor>;
using Vec4 = Eigen::Matrix<RealNum, 4, 1, Eigen::ColMajor>;
using Mat3 = Eigen::Matrix<RealNum, 3, 3, Eigen::ColMajor>;
using Mat4 = Eigen::Matrix<RealNum, 4, 4, Eigen::ColMajor>;
using Affine = Eigen::Transform<RealNum, 3, Eigen::Affine, Eigen::ColMajor>;
using Quat = Eigen::Quaternion<RealNum>;
using AngleAxis = Eigen::AngleAxis<RealNum>;

using BoolArray2 = boost::multi_array<bool, 2>;
using IdxArray2 = boost::multi_array<Idx, 2>;
using BoolArray3 = boost::multi_array<bool, 3>;
using IdxArray3 = boost::multi_array<Idx, 3>;
using RealArray3 = boost::multi_array<RealNum, 3>;

using Clock = std::chrono::steady_clock;
using TimePoint = std::chrono::time_point<Clock>;

const RealNum kRadCurve = 100.0;
const RealNum kAngleConstraintDegree = 90.0;

}

#endif
