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
#ifndef SNP_NEEDLE_VALIDATOR_H
#define SNP_NEEDLE_VALIDATOR_H

#include <cmath>

#include "../problem_config.h"
#include "../utils.h"

namespace unc::robotics::snp {

namespace utils {

template <typename State>
bool ValidPoint2PointProblem(const State& start, const State& goal, const RealNum& pos_tolerance,
                             const RealNum& ang_tolerance,
                             EnvPtr env, const RealNum& rad_curv, const RealNum& ins_length,
                             const RealNum& ang_constraint_rad, const bool constrain_goal_orientation) {
    if (ang_constraint_rad > 0.5*M_PI + EPS) {
        std::cout << "Using an angular constraint of " << ang_constraint_rad* RAD_TO_DEGREE <<
                  " (> 90) degrees! Not supported yet!" << std::endl;
        return false;
    }

    const Vec3& start_p = start.translation();
    const Quat start_q = start.rotation().normalized();
    const Vec3& goal_p = goal.translation();
    const Quat goal_q = goal.rotation().normalized();

    if (!env->WithinImage(start_p)) {
        std::cout << "Start point is outside image range!" << std::endl;
        return false;
    }

    if (!env->WithinImage(goal_p)) {
        std::cout << "Goal point is outside image range!" << std::endl;
        return false;
    }

    if (!env->CollisionFree(start_p)) {
        std::cout << "Start point is in collision!" << std::endl;
        return false;
    }

    if (!env->CollisionFree(goal_p)) {
        std::cout << "Goal point is in collision!" << std::endl;
        return false;
    }

    if ((start_p - goal_p).norm() > ins_length) {
        std::cout << "Exceed maximum insertion distance!" << std::endl;
        return false;
    }

    if (ComputeDistanceToTrumpetBoundary(start_p, start_q*Vec3::UnitZ(), goal_p,
                                         rad_curv) > pos_tolerance) {
        RealNum max_rad = ComputeRadiusOfCurvature(start_p, start_q, goal_p);
        std::cout << "Goal point not in FRS, need rad to be " << max_rad << std::endl;
        return false;
    }

    if (constrain_goal_orientation) {
        if (ComputeDistanceToTrumpetBoundary(goal_p, -(goal_q*Vec3::UnitZ()), start_p, rad_curv,
                                             ang_tolerance) > pos_tolerance) {
            RealNum max_rad = ComputeRadiusOfCurvature(goal_p, -(goal_q*Vec3::UnitZ()), start_p);
            std::cout << "Start point not in BRS, need rad to be " << max_rad << std::endl;
            return false;
        }

        if (!WorkspaceConnected(start_p, start_q, goal_p, goal_q, rad_curv, pos_tolerance, ang_tolerance)) {
            std::cout << "Mission impossible (FRS and BRS not connected)!" << std::endl;
            return false;
        }
    }

    return true;
}

template <typename State>
bool ExceedAngleConstraint(const State& s, const State& start, const RealNum ang_constraint_rad) {
    RealNum diff = DirectionDifference(s.rotation(), start.rotation());

    return (diff > ang_constraint_rad);
}

template <typename State>
bool ValidFullState(const State& s, const State& start, const State& goal, EnvPtr env,
                    const RealNum& rad_curv, const bool constrain_goal_orientation) {
    const Vec3& p = s.translation();
    const Quat q = s.rotation().normalized();

    if (!env->CollisionFree(p)) {
        return false;
    }

    if (!InTrumpet(start.translation(), start.rotation(), p, rad_curv)) {
        return false;
    }

    if (!InTrumpet(p, -(q*Vec3::UnitZ()), start.translation(), rad_curv)) {
        return false;
    }

    if (!InTrumpet(p, q, goal.translation(), rad_curv)) {
        return false;
    }

    if (constrain_goal_orientation) {
        if (!InTrumpet(goal.translation(), -(goal.rotation()*Vec3::UnitZ()), p, rad_curv)) {
            return false;
        }

        if (!WorkspaceConnected(p, q, goal.translation(), goal.rotation(), rad_curv)) {
            return false;
        }
    }

    return true;
}

template <typename State>
bool ValidStateWithGoalReachability(const State& s, const State& goal, const RealNum& pos_tolerance,
                                    const RealNum& ang_tolerance,
                                    EnvPtr env, const RealNum& rad_curv, const bool constrain_goal_orientation) {
    const Vec3& p = s.translation();
    const Quat q = s.rotation().normalized();

    if (!env->CollisionFree(p)) {
        return false;
    }

    if (ComputeDistanceToTrumpetBoundary(p, q*Vec3::UnitZ(), goal.translation(),
                                         rad_curv) > pos_tolerance) {
        return false;
    }

    if (constrain_goal_orientation) {
        if (ComputeDistanceToTrumpetBoundary(goal.translation(), -(goal.rotation()*Vec3::UnitZ()), p,
                                             rad_curv, ang_tolerance) > pos_tolerance) {
            return false;
        }

        if (!WorkspaceConnected(p, q, goal.translation(), goal.rotation(), rad_curv, pos_tolerance,
                                ang_tolerance)) {
            return false;
        }
    }

    return true;
}

template <typename State>
bool ValidMotion(const State& from, const State& to, EnvPtr env, const RealNum& rad_curv,
                 const RealNum& resolution) {
    const Vec3& sp = from.translation();
    const Vec3& gp = to.translation();
    const Vec3 relative_pos = gp - sp;
    const RealNum d = relative_pos.norm();

    if (d < EPS) {
        return true;
    }

    const Quat sq_normalized = from.rotation().normalized();
    const Quat gq_normalized = to.rotation().normalized();
    const Vec3 st = (sq_normalized*Vec3::UnitZ()).normalized();
    const Vec3 gt = (gq_normalized*Vec3::UnitZ()).normalized();

    const Vec3 normal_vec = (st.cross(gt)).normalized();

    if (normal_vec.dot(relative_pos.normalized()) > EPS) {
        return false;
    }

    if (ComputeDistanceToTrumpetBoundary(sp, st, gp, rad_curv) > EPS) {
        return false;
    }

    const RealNum cos_theta = (relative_pos.normalized()).dot(st);
    Vec3 result_p;

    if (cos_theta > 1 - EPS) {
        for (RealNum l = resolution; l < d; l += resolution) {
            result_p = sp + st * l;

            if (!env->CollisionFree(result_p)) {
                return false;
            }
        }
        return true;
    }

    const RealNum r = 0.5*d/sin(acos(cos_theta));

    const Vec3 center = sp + r*(normal_vec.cross(st));
    const RealNum max_angle = acos(((gp - center).normalized()).dot((sp - center).normalized()));
    const RealNum angle_step = resolution / r;

    for (RealNum ang = angle_step; ang < max_angle; ang += angle_step) {
        Quat proceed_quat(AngleAxis(ang, normal_vec));
        result_p = proceed_quat*(sp - center) + center;

        if (!env->CollisionFree(result_p)) {
            return false;
        }
    }

    return true;
}

template <typename State>
bool ValidMotion(const State& new_base, const std::vector<State>& motion, EnvPtr env) {
    if (!env) {
        throw std::runtime_error("No image environment! Cannot check path validity!");
    }

    if (motion.size() == 0) {
        throw std::runtime_error("Path contains no states! Cannot check path validity!");
    }

    const Vec3& base_p = new_base.translation();
    const Quat base_q = new_base.rotation().normalized();

    std::queue<std::pair<SizeType, SizeType>> queue;
    queue.emplace(0, motion.size());

    while (!queue.empty()) {
        auto p = queue.front();
        queue.pop();

        SizeType middle = p.first + (p.second - p.first)/2;

        if (!env->CollisionFree(base_q * motion[middle].translation() + base_p)) {
            return false;
        }

        if (p.first < middle) {
            queue.emplace(p.first, middle);
        }

        if (middle+1 < p.second) {
            queue.emplace(middle+1, p.second);
        }
    }

    return true;
}

template <typename State>
std::pair<bool, State> DirectConnecting(const State& s, const State& start, EnvPtr env,
                                        const RealNum& rad_curv, const RealNum& resolution, const Quat& pi_x) {
    State result = start;

    const Vec3& p = s.translation();
    const Quat q = s.rotation().normalized()*pi_x;
    const Vec3 t = q*Vec3::UnitZ();

    if (!InTrumpet(p, t, start.translation(), rad_curv)) {
        return {false, result};
    }

    const State start_state = ForwardTo(p, q, start.translation(), rad_curv);
    const std::vector<State> states = Interpolate(p, q, start.translation(), start_state.rotation(),
                                      rad_curv, resolution);

    for (const auto& state : states) {
        if (!env->CollisionFree(state.translation())) {
            return {false, result};
        }
    }

    result.rotation() = ((start_state.rotation())*pi_x).normalized();
    return {true, result};
}

template <typename State>
std::pair<bool, State> DirectConnectingWithoutCollisionCheck(const State& s, const State& start,
        EnvPtr env,
        const RealNum& rad_curv, const RealNum& resolution, const Quat& pi_x) {
    State result = start;

    const Vec3& p = s.translation();
    const Quat q = s.rotation().normalized()*pi_x;
    const Vec3 t = q*Vec3::UnitZ();

    if (!InTrumpet(p, t, start.translation(), rad_curv)) {
        return {false, result};
    }

    const State start_state = ForwardTo(p, q, start.translation(), rad_curv);
    result.rotation() = ((start_state.rotation())*pi_x).normalized();
    return {true, result};
}

inline bool ValidLength(const RealNum& l, const RealNum& max_l) {
    return (l < max_l);
}

RealNum MaxCurveLength(const Vec3& s, const Vec3& g, const RealNum& rad_curv) {
    RealNum d = (g - s).norm();

    if (rad_curv == R_INF) {
        return d;
    }

    RealNum theta = 2 * (asin(0.5 * d / rad_curv));
    return theta * rad_curv;
}

bool GoalSpheresReachable(const State& s, const std::vector<Vec3>& goals, const RealNum& rad_curv,
                          const RealNum& ins_length, const RealNum& pos_tolerance) {
    const Vec3& sp = s.translation();
    const Vec3 st = (s.rotation().normalized())*Vec3::UnitZ();

    for (const auto& p : goals) {
        if ((p - sp).norm() > ins_length + pos_tolerance) {
            continue;
        }

        if (ComputeDistanceToTrumpetBoundary(sp, st, p, rad_curv) < pos_tolerance) {
            return true;
        }
    }

    return false;
}

RealNum QueryLS(const RealNum& min_length_step) {
    RealNum ls;

    if (std::abs(min_length_step - 0.078125) < EPS) {
        ls = 1.007681;
    }
    else if (std::abs(min_length_step - 0.125) < EPS) {
        ls = 1.012314;
    }
    else if (std::abs(min_length_step - 1.25) < EPS) {
        ls = 1.123808;
    }
    else if (std::abs(min_length_step - 2.5) < EPS) {
        ls = 1.247726;
    }
    else {
        throw std::runtime_error("Undefined minimum length step!");
    }

    return ls;
}

}

template<typename State>
class Point2PointCurveValidator {
  public:
    Point2PointCurveValidator(const ConfigPtr cfg, const State& start, const State& goal,
                              const unsigned radius_status=0)
        : constrain_goal_orientation_(cfg->constrain_goal_orientation)
        , start_(start)
        , goal_(goal)
        , pos_tolerance_(cfg->goal_pos_tolerance)
        , ang_tolerance_(cfg->goal_ang_tolerance)
        , rad_curv_(cfg->rad_curv)
        , ins_length_(cfg->ins_length)
        , ang_constraint_rad_(cfg->ang_constraint_degree*DEGREE_TO_RAD)
        , validity_res_(cfg->validity_res)
        , steer_step_(cfg->steer_step)
        , radius_status_(radius_status) {
        if (cfg->env == nullptr) {
            throw std::runtime_error("Construction of validator failed! Config class doesn't have a valid environment!");
        }

        env_ = cfg->env;
        start_.rotation().normalize();
        goal_.rotation().normalize();
    }

    bool ValidProblem() const {
        return utils::ValidPoint2PointProblem(start_, goal_, pos_tolerance_, ang_tolerance_, env_,
                                              rad_curv_,
                                              ins_length_, ang_constraint_rad_, constrain_goal_orientation_);
    }

    bool ValidState(const State& s) const {
        if (radius_status_ > 0 && utils::ExceedAngleConstraint(s, start_, ang_constraint_rad_)) {
            return false;
        }

        return utils::ValidStateWithGoalReachability(s, goal_, pos_tolerance_, ang_tolerance_, env_,
                rad_curv_, constrain_goal_orientation_);
    }

    bool InCollision(const State& s) const {
        return (!env_->CollisionFree(s.translation()));
    }

    bool ValidMotion(const State& from, const State& to) const {
        return utils::ValidMotion(from, to, env_, rad_curv_, validity_res_);
    }

    bool ValidLength(const RealNum& l) const {
        return utils::ValidLength(l, ins_length_);
    }

  private:
    const bool constrain_goal_orientation_;
    const RealNum pos_tolerance_;
    const RealNum ang_tolerance_;
    const RealNum rad_curv_;
    const RealNum ins_length_;
    const RealNum ang_constraint_rad_;
    const RealNum validity_res_;
    const RealNum steer_step_;
    const unsigned radius_status_;

    State start_;
    State goal_;
    EnvPtr env_;
};

template<typename State>
class SpreadingValidator {
  public:
    SpreadingValidator(const ConfigPtr cfg, const State& start)
        : fix_orientation_(cfg->spreading_fix_start_orientation)
        , start_(start)
        , rad_curv_(cfg->rad_curv)
        , ins_length_(cfg->ins_length)
        , ang_constraint_rad_(cfg->ang_constraint_degree*DEGREE_TO_RAD)
        , validity_res_(cfg->validity_res) {
        if (cfg->env == nullptr) {
            throw std::runtime_error("Construction of validator failed! Config class doesn't have a valid environment!");
        }

        env_ = cfg->env;
        start_.rotation().normalize();
        rotate_pi_x_ = Quat(AngleAxis(M_PI, Vec3::UnitX())).normalized();
    }

    bool ValidProblem() const {
        const Vec3& start_p = start_.translation();

        if (!env_->WithinImage(start_p)) {
            std::cout << "Start point is outside image range!" << std::endl;
            return false;
        }

        if (!env_->CollisionFree(start_p)) {
            std::cout << "Start point is in collision!" << std::endl;
            return false;
        }

        return true;
    }

    bool ValidState(const State& s) const {
        if (utils::ExceedAngleConstraint(s, start_, ang_constraint_rad_)) {
            return false;
        }

        if (!env_->CollisionFree(s.translation())) {
            return false;
        }

        return true;
    }

    bool InCollision(const State& s) const {
        return (!env_->CollisionFree(s.translation()));
    }

    bool ValidMotion(const State& from, const State& to) const {
        return utils::ValidMotion(from, to, env_, rad_curv_, validity_res_);
    }

    bool ValidLength(const RealNum& l) const {
        return utils::ValidLength(l, ins_length_);
    }

    std::pair<bool, State> DirectConnectingStart(const State& s) const {
        if (fix_orientation_) {
            return {false, start_};
        }

        if (!this->ValidState(s)) {
            return {false, start_};
        }

        return utils::DirectConnecting(s, start_, env_, rad_curv_, validity_res_, rotate_pi_x_);
    }

  private:
    const bool fix_orientation_;
    const RealNum rad_curv_;
    const RealNum ins_length_;
    const RealNum ang_constraint_rad_;
    const RealNum validity_res_;

    State start_;
    EnvPtr env_;

    Quat rotate_pi_x_;
};

template<typename State>
class SpreadingWithGoalRegionValidator {
  public:
    SpreadingWithGoalRegionValidator(const ConfigPtr cfg, const State& start)
        : fix_orientation_(cfg->spreading_fix_start_orientation)
        , start_(start)
        , pos_tolerance_(cfg->goal_pos_tolerance)
        , rad_curv_(cfg->rad_curv)
        , ins_length_(cfg->ins_length)
        , ang_constraint_rad_(cfg->ang_constraint_degree*DEGREE_TO_RAD)
        , validity_res_(cfg->validity_res) {
        if (cfg->env == nullptr) {
            throw std::runtime_error("Construction of validator failed! Config class doesn't have a valid environment!");
        }

        env_ = cfg->env;
        start_.rotation().normalize();
        rotate_pi_x_ = Quat(AngleAxis(M_PI, Vec3::UnitX())).normalized();
    }

    void ProvideGoalPoints(const std::vector<Vec3>& goals) {
        goals_ = goals;
    }

    bool ValidProblem() const {
        const Vec3& start_p = start_.translation();
        const Quat& start_q = start_.rotation();

        if (!env_->WithinImage(start_p)) {
            std::cout << "Start point is outside image range!" << std::endl;
            return false;
        }

        if (!env_->CollisionFree(start_p)) {
            std::cout << "Start point is in collision!" << std::endl;
            return false;
        }

        if (goals_.size() == 0) {
            std::cout << "No goal spheres provided!" << std::endl;
            return false;
        }

        if (!utils::GoalSpheresReachable(start_, goals_, rad_curv_, ins_length_, pos_tolerance_)) {
            std::cout << "None of the goal points are reachable from current start pose!" << std::endl;
            return false;
        }

        std::cout << "Valid problem with " << goals_.size() << " goal spheres." << std::endl;
        return true;
    }

    bool ValidState(const State& s) const {
        if (utils::ExceedAngleConstraint(s, start_, ang_constraint_rad_)) {
            return false;
        }

        if (!env_->CollisionFree(s.translation())) {
            return false;
        }

        if (!utils::GoalSpheresReachable(s, goals_, rad_curv_, ins_length_, pos_tolerance_)) {
            return false;
        }

        return true;
    }

    bool InCollision(const State& s) const {
        return (!env_->CollisionFree(s.translation()));
    }

    bool ValidMotion(const State& from, const State& to) const {
        return utils::ValidMotion(from, to, env_, rad_curv_, validity_res_);
    }

    bool ValidLength(const RealNum& l) const {
        return utils::ValidLength(l, ins_length_);
    }

    std::pair<bool, State> DirectConnectingStart(const State& s) const {
        if (fix_orientation_) {
            return {false, start_};
        }

        if (!this->ValidState(s)) {
            return {false, start_};
        }

        return utils::DirectConnecting(s, start_, env_, rad_curv_, validity_res_, rotate_pi_x_);
    }

  private:
    const bool fix_orientation_;
    const RealNum pos_tolerance_;
    const RealNum rad_curv_;
    const RealNum ins_length_;
    const RealNum ang_constraint_rad_;
    const RealNum validity_res_;

    State start_;
    EnvPtr env_;

    Quat rotate_pi_x_;

    std::vector<Vec3> goals_;
};

template<typename State>
class MotionPrimitiveValidator {
  public:
    MotionPrimitiveValidator(const ConfigPtr cfg, const State& start, const State& goal,
                             const unsigned radius_status=0)
        : constrain_goal_orientation_(cfg->constrain_goal_orientation)
        , start_(start)
        , goal_(goal)
        , pos_tolerance_(cfg->goal_pos_tolerance)
        , ang_tolerance_(cfg->goal_ang_tolerance)
        , rad_curv_(cfg->rad_curv)
        , ins_length_(cfg->ins_length)
        , ang_constraint_rad_(cfg->ang_constraint_degree*DEGREE_TO_RAD)
        , validity_res_(cfg->validity_res)
        , steer_step_(cfg->steer_step)
        , radius_status_(radius_status)
        , prune_result_branch_(cfg->prune_result_branch) {
        if (cfg->env == nullptr) {
            throw std::runtime_error("Construction of validator failed! Config class doesn't have a valid environment!");
        }

        env_ = cfg->env;
        start_.rotation().normalize();
        goal_.rotation().normalize();

        const Idx max_length_i = std::ceil(std::log2(cfg->delta_ell_max/cfg->delta_ell_min));
        const RealNum min_length_step = cfg->delta_ell_max/(std::pow(2, max_length_i));
        max_insertion_ = pos_tolerance_ + utils::MaxCurveLength(start_.translation(), goal_.translation(),
                         rad_curv_);
        const Idx max_depth = std::ceil(max_insertion_/min_length_step);

        if (max_depth == 0) {
            throw std::runtime_error("[ERROR] Max depth cannot be 0!");
        }

        RealNum ls = utils::QueryLS(min_length_step);
        config_tolerance_ = SE3_T_W * 0.5 * cfg->goal_pos_tolerance * (ls - 1)/(std::pow(ls,
                            max_depth) - 1);
        MPT_LOG(INFO) << "Config tolerance: " << config_tolerance_;
    }

    bool ValidProblem() const {
        return utils::ValidPoint2PointProblem(start_, goal_, pos_tolerance_, ang_tolerance_, env_,
                                              rad_curv_,
                                              ins_length_, ang_constraint_rad_, constrain_goal_orientation_);
    }

    bool ValidState(const State& s) const {
        if (radius_status_ > 0 && utils::ExceedAngleConstraint(s, start_, ang_constraint_rad_)) {
            return false;
        }

        return utils::ValidStateWithGoalReachability(s, goal_, pos_tolerance_, ang_tolerance_, env_,
                rad_curv_, constrain_goal_orientation_);
    }

    bool InCollision(const State& s) const {
        return (!env_->CollisionFree(s.translation()));
    }

    bool ValidMotion(const State& from, const std::vector<State>& motion) const {
        return utils::ValidMotion(from, motion, env_);
    }

    bool ValidLength(const RealNum& l) const {
        return utils::ValidLength(l, max_insertion_);
    }

    const RealNum& ConfigTolerance() const {
        return config_tolerance_;
    }

    const bool& PruneResultBranch() const {
        return prune_result_branch_;
    }

  private:
    const bool constrain_goal_orientation_;
    const RealNum pos_tolerance_;
    const RealNum ang_tolerance_;
    const RealNum rad_curv_;
    const RealNum ins_length_;
    const RealNum ang_constraint_rad_;
    const RealNum validity_res_;
    const RealNum steer_step_;
    const unsigned radius_status_;
    const bool prune_result_branch_;
    RealNum config_tolerance_;
    RealNum max_insertion_;

    State start_;
    State goal_;
    EnvPtr env_;
};

template<typename State>
class MotionPrimitiveWithGoalValidator {
  public:
    MotionPrimitiveWithGoalValidator(const ConfigPtr cfg, const State& start)
        : constrain_goal_orientation_(cfg->constrain_goal_orientation)
        , fix_orientation_(cfg->spreading_fix_start_orientation)
        , start_(start)
        , pos_tolerance_(cfg->goal_pos_tolerance)
        , ang_tolerance_(cfg->goal_ang_tolerance)
        , rad_curv_(cfg->rad_curv)
        , ins_length_(cfg->ins_length)
        , ang_constraint_rad_(cfg->ang_constraint_degree*DEGREE_TO_RAD)
        , validity_res_(cfg->validity_res)
        , steer_step_(cfg->steer_step) {
        if (cfg->env == nullptr) {
            throw std::runtime_error("Construction of validator failed! Config class doesn't have a valid environment!");
        }

        env_ = cfg->env;
        start_.rotation().normalize();

        const Idx max_length_i = std::ceil(std::log2(cfg->delta_ell_max/cfg->delta_ell_min));
        const RealNum min_length_step = cfg->delta_ell_max/(std::pow(2, max_length_i));
        max_insertion_ = ins_length_;
        const Idx max_depth = std::ceil(max_insertion_/min_length_step);

        if (max_depth == 0) {
            throw std::runtime_error("[ERROR] Max depth cannot be 0!");
        }

        RealNum ls = utils::QueryLS(min_length_step);
        config_tolerance_ = SE3_T_W * 0.5 * cfg->goal_pos_tolerance * (ls - 1)/(std::pow(ls,
                            max_depth) - 1);
        MPT_LOG(INFO) << "Config tolerance: " << config_tolerance_;

        rotate_pi_x_ = Quat(AngleAxis(M_PI, Vec3::UnitX())).normalized();
    }

    void ProvideGoalPoints(const std::vector<Vec3>& goals) {
        goals_ = goals;
    }

    bool ValidProblem() const {
        const Vec3& start_p = start_.translation();
        const Quat& start_q = start_.rotation();

        if (!env_->WithinImage(start_p)) {
            std::cout << "Start point is outside image range!" << std::endl;
            return false;
        }

        if (!env_->CollisionFree(start_p)) {
            std::cout << "Start point is in collision!" << std::endl;
            return false;
        }

        if (goals_.size() == 0) {
            std::cout << "No goal spheres provided!" << std::endl;
            return false;
        }

        if (!utils::GoalSpheresReachable(start_, goals_, rad_curv_, ins_length_, pos_tolerance_)) {
            std::cout << "None of the goal points are reachable from current start pose!" << std::endl;
            return false;
        }

        std::cout << "Valid problem with " << goals_.size() << " goal spheres." << std::endl;
        return true;
    }

    bool ValidState(const State& s) const {
        if (utils::ExceedAngleConstraint(s, start_, ang_constraint_rad_)) {
            return false;
        }

        if (!env_->CollisionFree(s.translation())) {
            return false;
        }

        if (!utils::GoalSpheresReachable(s, goals_, rad_curv_, ins_length_, pos_tolerance_)) {
            return false;
        }

        return true;
    }

    bool InCollision(const State& s) const {
        return (!env_->CollisionFree(s.translation()));
    }

    bool ValidMotion(const State& from, const std::vector<State>& motion) const {
        return utils::ValidMotion(from, motion, env_);
    }

    bool ValidLength(const RealNum& l) const {
        return utils::ValidLength(l, max_insertion_);
    }

    const RealNum& ConfigTolerance() const {
        return config_tolerance_;
    }

    std::pair<bool, State> DirectConnectingStart(const State& s) const {
        if (fix_orientation_) {
            return {false, start_};
        }

        return utils::DirectConnectingWithoutCollisionCheck(s, start_, env_, rad_curv_,
                validity_res_, rotate_pi_x_);
    }

  private:
    const bool constrain_goal_orientation_;
    const bool fix_orientation_;
    const RealNum pos_tolerance_;
    const RealNum ang_tolerance_;
    const RealNum rad_curv_;
    const RealNum ins_length_;
    const RealNum ang_constraint_rad_;
    const RealNum validity_res_;
    const RealNum steer_step_;
    RealNum config_tolerance_;
    RealNum max_insertion_;

    State start_;
    EnvPtr env_;
    std::vector<Vec3> goals_;
    Quat rotate_pi_x_;
};

}

#endif
