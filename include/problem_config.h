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
#ifndef SNP_PROBLEM_CONFIG_H
#define SNP_PROBLEM_CONFIG_H

#include <cmath>

#include "global_common.h"
#include "image_environment.h"

namespace unc::robotics::snp {

struct ProblemConfig {
    // Constant parameters.
    const RealNum rad_curv;
    const RealNum needle_d;
    const RealNum ins_length;
    const RealNum ang_constraint_degree;
    const RealNum dist_threshold_0;
    const RealNum dist_threshold_1;

    // Environment.
    EnvPtr env;

    // Planner behavior control. All parameters use [mm], [rad].
    // If use goal state that considers orientation, this cannot be changed later.
    const bool constrain_goal_orientation;
    // Use single threads or multiple threads.
    bool multi_threading = true;
    // Position tolerance for the goal state.
    RealNum goal_pos_tolerance = EPS;
    // Orientation tolerance for the goal state.
    RealNum goal_ang_tolerance = 0.005;
    // When a state is close enough to the goal, always try direct connection.
    RealNum goal_connecting_rad = constrain_goal_orientation? 10.0 : 5.0;
    // Maximum distance between two states
    RealNum steer_step = constrain_goal_orientation ? 5.0 : 2.0;
    // Resolution to check if an edge is valid.
    RealNum validity_res = 0.5;
    RealNum cost_res = 0.1;
    // Safe margin for collision detection.
    RealNum safe_margin = 0.0;
    // Resolution used to reinterpolate the result plan.
    RealNum result_res = 2.0;
    // The probability of sampling goal state directly.
    RealNum goal_bias = 0.05;
    // For an arbitrary state, the probability of connecting it to the goal directly.
    RealNum direct_connect_ratio = constrain_goal_orientation? 1.0 : 0.5;
    // Method used to do goal connection.
    bool use_dubins_connection = false;
    // When doing spreading, if the start orientation is fixed.
    bool spreading_fix_start_orientation = false;
    // If we allow spreading in all directions, some of the configurations are used
    // for generating new start orientations.
    RealNum start_connect_ratio = 0.05;
    // If the sampler also sample orientation.
    bool sample_orientation = false;
    // For spreading planner.
    RealNum spreading_min_dist = 100.0;

    // For RCS planner.
    RealNum delta_ell_max = 16.0;
    RealNum delta_theta_max = 0.5 * M_PI;
    RealNum delta_ell_min = 0.125;
    RealNum delta_theta_min = 0.157;

    // For RCS* planner.
    unsigned look_ahead = 3;
    RealNum cost_approx_factor = 0.1;

    bool optimal = false;
    bool use_trilinear_interpolation = true;

    // Termination control.
    // Timeout in milliseconds.
    SizeType timeout = 1000;
    // Maximum number of nodes in the tree.
    SizeType num_nodes = 10000;
    // Number of plans needed for termination.
    SizeType num_plans_needed = 10;

    // Misc.
    Idx seed = 1;
    bool show_logs = true;
    Str output_file_root = "../data/output/test";
    Str obstacle_file = "../data/input/obstacles.txt";
    Str cost_file = "../data/input/costs.txt";
    Str healpix_file = "../data/input/HEALPix.txt";

#ifndef HAVE_GLOBAL_VARIABLES
    ProblemConfig(const bool orientation=false,
                  const RealNum needle_min_curve_rad=kRadCurve,
                  const RealNum needle_diameter=2.0,
                  const RealNum insert_length=80.0,
                  const RealNum angle_constraint=kAngleConstraintDegree)
        : constrain_goal_orientation(orientation)
        , rad_curv(needle_min_curve_rad)
        , needle_d(needle_diameter)
        , ins_length(insert_length)
        , ang_constraint_degree(angle_constraint)
        , dist_threshold_0(Threshold0(needle_min_curve_rad, angle_constraint))
        , dist_threshold_1(Threshold1(needle_min_curve_rad, angle_constraint)) {
    }
#else
    ProblemConfig(const bool orientation=false,
                  const RealNum needle_min_curve_rad=global::needle_min_curve_rad,
                  const RealNum needle_diameter=2.0,
                  const RealNum insert_length=80.0,
                  const RealNum angle_constraint=global::angle_constraint_degree)
        : constrain_goal_orientation(orientation)
        , rad_curv(needle_min_curve_rad)
        , needle_d(needle_diameter)
        , ins_length(insert_length)
        , ang_constraint_degree(angle_constraint)
        , dist_threshold_0(Threshold0(needle_min_curve_rad, angle_constraint))
        , dist_threshold_1(Threshold1(needle_min_curve_rad, angle_constraint)) {
    }
#endif

    RealNum Threshold0(const RealNum min_rad, const RealNum ang) const {
        return 2*(min_rad*std::sin(std::fmin(ang, 180.0) * DEGREE_TO_RAD/2));
    }

    RealNum Threshold1(const RealNum min_rad, const RealNum ang) const {
        return 2*min_rad;
    }

    void DefaultSetup() {
        env.reset(new ImageEnvironment());

        if (!env->ConstructEnvironmentFromFile(obstacle_file) || !env->ConstructCostFromFile(cost_file)) {
            throw std::runtime_error("Failed to initialize environment from file!");
        }

        if (constrain_goal_orientation) {
            env->SetCostType(ImageEnvironment::CostType::GOAL_ORIENTATION);
        }
        else {
            env->SetCostType(ImageEnvironment::CostType::PATH_LENGTH);
        }

        env->SetMinDist(0.5*needle_d + env->VoxelRadius() + safe_margin);
        env->EnableTrilinearInterpolation(use_trilinear_interpolation);
    }

    void SetEnvironment(EnvPtr environment) {
        env = environment;
        env->SetMinDist(0.5*needle_d + env->VoxelRadius() + safe_margin);
    }

    void UpdateSafeMargin(const RealNum& margin) {
        safe_margin = margin;

        if (env) {
            env->SetMinDist(0.5*needle_d + env->VoxelRadius() + safe_margin);
        }
    }
};

using ConfigPtr = std::shared_ptr<ProblemConfig>;

} // namespace unc::robotics::snp

#endif // SNP_PROBLEM_CONFIG_H