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
#include <queue>

#include "../problem_config.h"
#include "../needle_utils.h"

namespace unc::robotics::snp {

namespace utils {

template <typename State, bool Init=false>
bool CheckWorkspaceConnected(const State& s, const State& goal, const RealNum& rad_curv, const RealNum& pos_tolerance,
                             const std::vector<IntPoint>& neig, EnvPtr env, BoolArray3& visited, unsigned& max_size)
{
    const Vec3& sp = s.translation();
    const Quat sq = s.rotation().normalized();
    const Vec3 st = (sq * Vec3::UnitZ()).normalized();
    const Vec3& gp = goal.translation();
    const Vec3 sg = gp - sp;
    const Vec3 unit_sg = sg.normalized();
    const RealNum d = sg.norm();
    const RealNum& voxel_rad = env->VoxelRadius();

    RealNum max_h;
    if (2 * rad_curv - pos_tolerance < d) {
        max_h = d + pos_tolerance;
    }
    else {
        RealNum r = rad_curv - pos_tolerance;
        RealNum cos_theta = (d*d + std::pow(rad_curv, 2) - r*r)/(2*d*rad_curv);
        max_h = 2 * rad_curv*cos_theta;
    }

    RealNum center_x;
    RealNum center_y = 0.5 * max_h;
    if (rad_curv > center_y) {
        center_x = std::sqrt(rad_curv * rad_curv - center_y * center_y);
    }
    else {
        center_x = 0;
    }
    const Vec2 rugby_center(-center_x, center_y);
    const RealNum rugby_rad = std::max(rad_curv, center_y);

    if constexpr (Init) {
        env->SetWorkspace(false);
    }
    std::fill(visited.data(), visited.data() + visited.num_elements(), false);

    bool connected = false;
    std::queue<IntPoint> queue;
    auto const start_ijk = env->RasToIjk(sp).cast<int>();
    queue.push(start_ijk);
    if constexpr (Init) {
        env->SetWorkspace(start_ijk[0], start_ijk[1], start_ijk[2]);
    }
    visited[start_ijk[0]][start_ijk[1]][start_ijk[2]] = true;

    IntPoint cur_ijk, inc;
    IdxPoint inc_ijk;
    SizeType counter = 0;
    while (!queue.empty()) {
        cur_ijk = queue.front();
        queue.pop();
        ++counter;

        if constexpr (!Init) {
            if (counter > max_size) {
                return true;
            }
        }

        for (auto const& n : neig) {
            inc = cur_ijk + n;
            inc_ijk = inc.cast<Idx>();
            if (!env->WithinImage(inc_ijk)) {
                continue;
            }

            if (visited[inc[0]][inc[1]][inc[2]]) {
                continue;
            }
            visited[inc[0]][inc[1]][inc[2]] = true;

            bool valid = true;
            if constexpr (Init) {
                valid = !env->IsObstacle(inc_ijk);
            }
            else {
                valid = env->Workspace(inc[0], inc[1], inc[2]);
            }

            if (valid) {
                bool valid = false;
                const Vec3 inc_ras = env->IjkToRas(inc_ijk);
                if ((inc_ras - sp).norm() < voxel_rad) {
                    valid = true;
                }
                else {
                    RealNum dist_to_trumpet = DistanceToTrumpetBoundary(sp, st, inc_ras, rad_curv);
                    if (dist_to_trumpet < voxel_rad) {
                        const Vec3 relative_p = inc_ras - sp;
                        RealNum inc_y = relative_p.dot(unit_sg);
                        if (inc_y > -voxel_rad && inc_y < max_h + voxel_rad) {
                            RealNum inc_x = (relative_p - inc_y * unit_sg).norm();
                            if ((Vec2(inc_x, inc_y) - rugby_center).norm() < rugby_rad + voxel_rad) {
                                valid = true;
                            }
                        }
                    }
                }

                if (valid) {
                    if ((inc_ras - gp).norm() < voxel_rad + pos_tolerance) {
                        connected = true;
                        if constexpr (!Init) {
                            return connected;
                        }
                    }
                    queue.push(inc);
                    if constexpr (Init) {
                        env->SetWorkspace(inc[0], inc[1], inc[2]);
                    }
                }
            }
        }
    }

    if constexpr (Init) {
        max_size = counter;
    }

    return connected;
}

template <typename State>
bool ValidPoint2PointProblem(const State& start, const State& goal, const RealNum& pos_tolerance,
                             const RealNum& ang_tolerance, const std::vector<IntPoint>& neig,
                             EnvPtr env, const RealNum& rad_curv, const RealNum& ins_length,
                             const RealNum& ang_constraint_rad, const bool constrain_goal_orientation,
                             BoolArray3& visited, unsigned& max_size) {
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

    if (DistanceToTrumpetBoundary(start_p, start_q*Vec3::UnitZ(), goal_p, rad_curv) > pos_tolerance) {
        const RealNum max_rad = RadiusOfCurvature(start_p, start_q, goal_p, pos_tolerance);
        std::cout << "Goal point not in FRS, need rad to be " << max_rad << std::endl;
        return false;
    }

    if (!CheckWorkspaceConnected<State, true>(start, goal, rad_curv, pos_tolerance, neig, env, visited, max_size)) {
        std::cout << "Reachable workspace is not connected to the goal!" << std::endl;
        return false;
    }

    if (constrain_goal_orientation) {
        if (DistanceToTrumpetBoundary(goal_p, -(goal_q*Vec3::UnitZ()), start_p, rad_curv, ang_tolerance) > pos_tolerance) {
            const RealNum max_rad = RadiusOfCurvature(goal_p, -(goal_q*Vec3::UnitZ()), start_p, pos_tolerance);
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
    const RealNum diff = DirectionDifference(s.rotation(), start.rotation());

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

    if (DistanceToTrumpetBoundary(p, q*Vec3::UnitZ(), goal.translation(),
                                         rad_curv) > pos_tolerance) {
        return false;
    }

    if (constrain_goal_orientation) {
        if (DistanceToTrumpetBoundary(goal.translation(), -(goal.rotation()*Vec3::UnitZ()), p,
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

    const Vec3 normal_vec = (st.cross(gt)).normalized();

    if (normal_vec.dot(relative_pos.normalized()) > EPS) {
        return false;
    }

    if (DistanceToTrumpetBoundary(sp, st, gp, rad_curv) > EPS) {
        return false;
    }

    const RealNum r = 0.5 * d / std::sin(std::acos(cos_theta));

    const Vec3 center = sp + r*(normal_vec.cross(st));
    const RealNum max_angle = std::acos(((gp - center).normalized()).dot((sp - center).normalized()));
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
bool ValidMotion(const State& new_base, const std::vector<State>& motion, EnvPtr env, const unsigned& offset=0) {
    if (!env) {
        throw std::runtime_error("No image environment! Cannot check path validity!");
    }

    if (motion.size() == 0) {
        throw std::runtime_error("Path contains no states! Cannot check path validity!");
    }

    const Vec3& base_p = new_base.translation();
    const Quat& base_q = new_base.rotation().normalized();

    std::queue<std::pair<SizeType, SizeType>> queue;
    queue.emplace(offset, motion.size());

    Vec3 result_p;
    while (!queue.empty()) {
        auto p = queue.front();
        queue.pop();

        SizeType middle = p.first + (p.second - p.first)/2;

        result_p = base_q * motion[middle].translation() + base_p;
        if (!env->CollisionFree(result_p)) {
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
bool ValidMotion(const std::vector<State>& motion, EnvPtr env, const unsigned& offset=0) {
    if (!env) {
        throw std::runtime_error("No image environment! Cannot check path validity!");
    }

    if (motion.size() == 0) {
        throw std::runtime_error("Path contains no states! Cannot check path validity!");
    }

    std::queue<std::pair<SizeType, SizeType>> queue;
    queue.emplace(offset, motion.size());

    Vec3 result_p;
    while (!queue.empty()) {
        auto p = queue.front();
        queue.pop();

        SizeType middle = p.first + (p.second - p.first)/2;

        if (!env->CollisionFree(motion[middle].translation())) {
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
std::optional<State> DirectConnecting(const State& s, const State& start, EnvPtr env,
                                      const RealNum& rad_curv, const RealNum& resolution, const Quat& pi_x) {
    State result = start;

    const Vec3& p = s.translation();
    const Quat q = s.rotation().normalized()*pi_x;
    const Vec3 t = q*Vec3::UnitZ();

    if (!InTrumpet(p, t, start.translation(), rad_curv)) {
        return {};
    }

    const State start_state = ForwardTo<State>(p, q, start.translation(), rad_curv);
    const std::vector<State> states = Interpolate<State>(p, q, start.translation(), start_state.rotation(),
                                      rad_curv, resolution);

    for (auto const& state : states) {
        if (!env->CollisionFree(state.translation())) {
            return {};
        }
    }

    result.rotation() = ((start_state.rotation())*pi_x).normalized();
    return result;
}

template <typename State>
std::optional<State> DirectConnectingWithoutCollisionCheck(const State& s, const State& start, EnvPtr env,
                                                           const RealNum& rad_curv, const RealNum& resolution, const Quat& pi_x) {
    State result = start;

    const Vec3& p = s.translation();
    const Quat q = s.rotation().normalized()*pi_x;
    const Vec3 t = q*Vec3::UnitZ();

    if (!InTrumpet(p, t, start.translation(), rad_curv)) {
        return {};
    }

    const State start_state = ForwardTo<State>(p, q, start.translation(), rad_curv);
    result.rotation() = ((start_state.rotation())*pi_x).normalized();
    return result;
}

inline bool ValidLength(const RealNum& l, const RealNum& max_l) {
    return (l < max_l);
}

RealNum MaxCurveLength(const Vec3& s, const Vec3& g, const RealNum& rad_curv) {
    const RealNum d = (g - s).norm();

    if (rad_curv == R_INF) {
        return d;
    }

    if (d > 2 * rad_curv) {
        return std::sqrt(2) * d - 2 * rad_curv + 0.5 * M_PI * rad_curv;
    }

    const RealNum& theta = 2 * (std::asin(0.5 * d / rad_curv));

    return theta * rad_curv;
}

template<typename State>
bool GoalSpheresReachable(const State& s, const std::vector<Vec3>& goals, const RealNum& rad_curv,
                          const RealNum& ins_length, const RealNum& pos_tolerance) {
    const Vec3& sp = s.translation();
    const Vec3& st = (s.rotation().normalized())*Vec3::UnitZ();

    for (auto const& p : goals) {
        if (ShortestDistance(s, p, rad_curv, pos_tolerance) > ins_length) {
            continue;
        }

        if (DistanceToTrumpetBoundary(sp, st, p, rad_curv) < pos_tolerance) {
            return true;
        }
    }

    return false;
}

RealNum QueryLS(const RealNum& min_length_step) {
    // These values are emprically determined.
    if (std::abs(min_length_step - 0.078125) < EPS) {
        return 1.007681;
    }
    else if (std::abs(min_length_step - 0.125) < EPS) {
        return 1.012314;
    }
    else if (std::abs(min_length_step - 0.5) < EPS) {
        return 1.049267;
    }
    else if (std::abs(min_length_step - 1.0) < EPS) {
        return 1.098859;
    }
    else if (std::abs(min_length_step - 1.25) < EPS) {
        return 1.123808;
    }
    else if (std::abs(min_length_step - 2.0) < EPS) {
        return 1.198380;
    }
    else if (std::abs(min_length_step - 2.5) < EPS) {
        return 1.248207;
    }

    throw std::runtime_error("Undefined minimum length step!");
    return -1;
}

} // namespace utils

template<typename State>
class ValidatorBase {
  public:
    ValidatorBase(const ConfigPtr cfg) : ins_length_(cfg->ins_length)
    {
        if (cfg->env == nullptr) {
            throw std::runtime_error("Construction of validator failed! Config class doesn't have a valid environment!");
        }
        env_ = cfg->env;
    }
    ~ValidatorBase() = default;

    bool InCollision(const State& s) const {
        return (!env_->CollisionFree(s.translation()));
    }
    bool InCollision(const Vec3& p) const {
        return (!env_->CollisionFree(p));
    }
    bool ValidLength(const RealNum& l) const {
        return utils::ValidLength(l, ins_length_);
    }

    const RealNum ins_length_;
    EnvPtr env_;
};

template<typename State>
class Point2PointCurveValidator : public ValidatorBase<State> {
    using base = ValidatorBase<State>;
  public:
    Point2PointCurveValidator(const ConfigPtr cfg, const State& start, const State& goal,
                              const unsigned radius_status=0)
        : ValidatorBase<State>(cfg)
        , constrain_goal_orientation_(cfg->constrain_goal_orientation)
        , start_(start)
        , goal_(goal)
        , pos_tolerance_(cfg->goal_pos_tolerance)
        , ang_tolerance_(cfg->goal_ang_tolerance)
        , rad_curv_(cfg->rad_curv)
        , ins_length_(cfg->ins_length)
        , ang_constraint_rad_(cfg->ang_constraint_degree*DEGREE_TO_RAD)
        , validity_res_(cfg->validity_res)
        , radius_status_(radius_status)
    {
        start_.rotation().normalize();
        goal_.rotation().normalize();

        neig_.emplace_back(-1, 0, 0);
        neig_.emplace_back(1, 0, 0);
        neig_.emplace_back(0, -1, 0);
        neig_.emplace_back(0, 1, 0);
        neig_.emplace_back(0, 0, -1);
        neig_.emplace_back(0, 0, 1);
        auto const& img_size = base::env_->ImageSize();
        visited_.resize(boost::extents[img_size[0]][img_size[1]][img_size[2]]);
    }

    bool ValidProblem() {
        return utils::ValidPoint2PointProblem(start_, goal_, pos_tolerance_, ang_tolerance_, neig_, base::env_,
                                            rad_curv_, ins_length_, ang_constraint_rad_, constrain_goal_orientation_,
                                            visited_, max_size_);
    }

    bool Valid(const State& s) const {
        if (radius_status_ > 0 && utils::ExceedAngleConstraint(s, start_, ang_constraint_rad_)) {
            return false;
        }

        return utils::ValidStateWithGoalReachability(s, goal_, pos_tolerance_, ang_tolerance_, base::env_,
                rad_curv_, constrain_goal_orientation_);
    }

    bool ValidMotion(const State& from, const State& to) const {
        return utils::ValidMotion(from, to, base::env_, rad_curv_, validity_res_);
    }

  private:
    const bool constrain_goal_orientation_;
    const RealNum pos_tolerance_;
    const RealNum ang_tolerance_;
    const RealNum rad_curv_;
    const RealNum ins_length_;
    const RealNum ang_constraint_rad_;
    const RealNum validity_res_;
    const unsigned radius_status_;

    State start_;
    State goal_;
    std::vector<IntPoint> neig_;
    BoolArray3 visited_;
    unsigned max_size_;
};

template<typename State>
class SpreadingValidator : public ValidatorBase<State> {
    using base = ValidatorBase<State>;
  public:
    SpreadingValidator(const ConfigPtr cfg, const State& start)
        : ValidatorBase<State>(cfg)
        , fix_orientation_(cfg->spreading_fix_start_orientation)
        , start_(start)
        , pos_tolerance_(cfg->goal_pos_tolerance)
        , rad_curv_(cfg->rad_curv)
        , ins_length_(cfg->ins_length)
        , ang_constraint_rad_(cfg->ang_constraint_degree*DEGREE_TO_RAD)
        , validity_res_(cfg->validity_res)
    {
        start_.rotation().normalize();
        rotate_pi_x_ = Quat(AngleAxis(M_PI, Vec3::UnitX())).normalized();
    }

    void ProvideGoalPoints(const std::vector<Vec3>& goals) {
        goals_ = goals;
    }

    bool ValidProblem() const {
        const Vec3& start_p = start_.translation();

        if (!base::env_->WithinImage(start_p)) {
            std::cout << "Start point is outside image range!" << std::endl;
            return false;
        }

        if (base::InCollision(start_p)) {
            std::cout << "Start point is in collision!" << std::endl;
            return false;
        }

        if (!goals_.empty() && !utils::GoalSpheresReachable(start_, goals_, rad_curv_, ins_length_, pos_tolerance_)) {
            std::cout << "None of the goal points are reachable from current start pose!" << std::endl;
            return false;
        }

        std::cout << "Valid problem with " << goals_.size() << " goal spheres." << std::endl;
        return true;
    }

    bool Valid(const State& s, const RealNum& length=0) const {
        if (utils::ExceedAngleConstraint(s, start_, ang_constraint_rad_)) {
            return false;
        }

        if (base::InCollision(s)) {
            return false;
        }

        if (!goals_.empty() && !utils::GoalSpheresReachable(s, goals_, rad_curv_, ins_length_ - length, pos_tolerance_)) {
            return false;
        }

        return true;
    }

    bool ValidMotion(const State& from, const State& to) const {
        return utils::ValidMotion(from, to, base::env_, rad_curv_, validity_res_);
    }

    std::optional<State> DirectConnectingStart(const State& s) const {
        if (fix_orientation_) {
            return {};
        }

        return utils::DirectConnectingWithoutCollisionCheck(s, start_, base::env_, rad_curv_,
                validity_res_, rotate_pi_x_);
    }

  private:
    const bool fix_orientation_;
    const RealNum pos_tolerance_;
    const RealNum rad_curv_;
    const RealNum ins_length_;
    const RealNum ang_constraint_rad_;
    const RealNum validity_res_;

    State start_;
    Quat rotate_pi_x_;
    std::vector<Vec3> goals_;
};

template<typename State>
class MotionPrimitiveValidator : public ValidatorBase<State> {
    using base = ValidatorBase<State>;
  public:
    MotionPrimitiveValidator(const ConfigPtr cfg, const State& start, const State& goal,
                             const unsigned radius_status=0)
        : ValidatorBase<State>(cfg)
        , constrain_goal_orientation_(cfg->constrain_goal_orientation)
        , start_(start)
        , goal_(goal)
        , pos_tolerance_(cfg->goal_pos_tolerance)
        , ang_tolerance_(cfg->goal_ang_tolerance)
        , rad_curv_(cfg->rad_curv)
        , ins_length_(cfg->ins_length)
        , ang_constraint_rad_(cfg->ang_constraint_degree*DEGREE_TO_RAD)
        , validity_res_(cfg->validity_res)
        , radius_status_(radius_status)
    {
        start_.rotation().normalize();
        goal_.rotation().normalize();

        const Idx max_length_i = std::ceil(std::log2(cfg->delta_ell_max/cfg->delta_ell_min));
        const RealNum min_length_step = cfg->delta_ell_max/(std::pow(2, max_length_i));
        max_insertion_ = utils::MaxCurveLength(start_.translation(), goal_.translation(), rad_curv_) + pos_tolerance_;
        const Idx max_depth = std::ceil(max_insertion_/min_length_step);

        if (max_depth == 0) {
            throw std::runtime_error("[ERROR] Max depth cannot be 0!");
        }

        RealNum ls = utils::QueryLS(min_length_step);
        RealNum delta = 0.5 * pos_tolerance_;

        if (cfg->optimal) {
            delta = std::min(delta, cfg->cost_approx_factor / base::env_->CostK());
        }

        config_tolerance_ = std::min(2 * rad_curv_ * std::sin(0.5 * min_length_step / rad_curv_),
                            SE3_T_W * 0.5 * delta * (ls - 1) / (std::pow(ls, max_depth) - 1)) - EPS;

        MPT_LOG(INFO) << "Config tolerance: " << config_tolerance_;

        goal_min_clearance_ = base::env_->DistanceToObstacleCenter(goal_.translation());

        neig_.emplace_back(-1, 0, 0);
        neig_.emplace_back(1, 0, 0);
        neig_.emplace_back(0, -1, 0);
        neig_.emplace_back(0, 1, 0);
        neig_.emplace_back(0, 0, -1);
        neig_.emplace_back(0, 0, 1);

        auto const& img_size = base::env_->ImageSize();
        visited_.resize(boost::extents[img_size[0]][img_size[1]][img_size[2]]);
    }

    bool ValidProblem() {
        const bool& valid = utils::ValidPoint2PointProblem(start_, goal_, pos_tolerance_, ang_tolerance_, neig_, base::env_,
                                            rad_curv_, ins_length_, ang_constraint_rad_, constrain_goal_orientation_,
                                            visited_, max_size_);
        max_size_ /= 10;
        return valid;
    }

    bool Valid(const State& s) const {
        if (radius_status_ > 0 && utils::ExceedAngleConstraint(s, start_, ang_constraint_rad_)) {
            return false;
        }

        return utils::ValidStateWithGoalReachability(s, goal_, pos_tolerance_, ang_tolerance_, base::env_,
                rad_curv_, constrain_goal_orientation_);
    }

    bool ValidReachableSpace(const State& s) {
        return utils::CheckWorkspaceConnected(s, goal_, rad_curv_, pos_tolerance_, neig_, base::env_, visited_, max_size_);
    }

    bool ValidMotion(const State& from, const std::vector<State>& motion, const unsigned& offset) const {
        return utils::ValidMotion(from, motion, base::env_, offset);
    }

    bool Valid(const std::vector<State>& motion) const {
        return utils::ValidMotion(motion, base::env_);
    }

    const RealNum& ConfigTolerance() const {
        return config_tolerance_;
    }

    RealNum CostToGo(const State& s) const {
        RealNum shortest_dist = ShortestDistance(s, goal_.translation(), rad_curv_, pos_tolerance_);

        if (shortest_dist == R_INF) {
            shortest_dist = 1000.0;
        }

        if (base::env_->ActiveCostType() == ImageEnvironment::CostType::PATH_LENGTH) {
            return shortest_dist;
        }

        if (base::env_->ActiveCostType() == ImageEnvironment::CostType::DIST_TO_OBS) {
            RealNum cur_clearance = base::env_->DistanceToObstacleCenter(s.translation());
            RealNum lower_bound = std::log(std::pow(cur_clearance
                                                    + goal_min_clearance_
                                                    + shortest_dist, 2)
                                           /(4*cur_clearance*goal_min_clearance_));
            return lower_bound;
        }

        if (base::env_->ActiveCostType() == ImageEnvironment::CostType::COST_MAP) {
            return base::env_->MinCost() * shortest_dist;
        }

        return 0;
    }

    IdxPoint ImageCoordinates(const State& s) const {
        return base::env_->RasToIjk(s.translation());
    }

    const State& GoalState() const {
        return goal_;
    }

    const RealNum& GoalTolerance() const {
        return pos_tolerance_;
    }

  private:
    const bool constrain_goal_orientation_;
    const RealNum pos_tolerance_;
    const RealNum ang_tolerance_;
    const RealNum rad_curv_;
    const RealNum ins_length_;
    const RealNum ang_constraint_rad_;
    const RealNum validity_res_;
    const unsigned radius_status_;
    RealNum config_tolerance_;
    RealNum max_insertion_;
    RealNum goal_min_clearance_;

    State start_;
    State goal_;
    std::vector<IntPoint> neig_;
    BoolArray3 visited_;
    unsigned max_size_;
};

template<typename State>
class MotionPrimitiveSpreadingValidator : public ValidatorBase<State> {
    using base = ValidatorBase<State>;
  public:
    MotionPrimitiveSpreadingValidator(const ConfigPtr cfg, const State& start)
        : ValidatorBase<State>(cfg)
        , constrain_goal_orientation_(cfg->constrain_goal_orientation)
        , fix_orientation_(cfg->spreading_fix_start_orientation)
        , start_(start)
        , pos_tolerance_(cfg->goal_pos_tolerance)
        , ang_tolerance_(cfg->goal_ang_tolerance)
        , rad_curv_(cfg->rad_curv)
        , ins_length_(cfg->ins_length)
        , ang_constraint_rad_(cfg->ang_constraint_degree*DEGREE_TO_RAD)
        , validity_res_(cfg->validity_res)
        , healpix_file_(cfg->healpix_file)
    {
        start_.rotation().normalize();

        const Idx max_length_i = std::ceil(std::log2(cfg->delta_ell_max/cfg->delta_ell_min));
        const RealNum min_length_step = cfg->delta_ell_max/(std::pow(2, max_length_i));
        max_insertion_ = ins_length_;
        const Idx max_depth = std::ceil(max_insertion_/min_length_step);

        if (max_depth == 0) {
            throw std::runtime_error("[ERROR] Max depth cannot be 0!");
        }

        RealNum ls = utils::QueryLS(min_length_step);
        RealNum delta = 0.5 * pos_tolerance_;

        if (cfg->optimal) {
            delta = std::min(delta, cfg->cost_approx_factor / base::env_->CostK());
        }

        config_tolerance_ = std::min(2 * rad_curv_ * std::sin(0.5 * min_length_step / rad_curv_),
                            SE3_T_W * 0.5 * delta * (ls - 1) / (std::pow(ls, max_depth) - 1)) - EPS;

        MPT_LOG(INFO) << "Config tolerance: " << config_tolerance_;

        rotate_pi_x_ = Quat(AngleAxis(M_PI, Vec3::UnitX())).normalized();
    }

    void ProvideGoalPoints(const std::vector<Vec3>& goals) {
        goals_ = goals;
    }

    bool ValidProblem() const {
        const Vec3& start_p = start_.translation();

        if (!base::env_->WithinImage(start_p)) {
            std::cout << "Start point is outside image range!" << std::endl;
            return false;
        }

        if (base::InCollision(start_p)) {
            std::cout << "Start point is in collision!" << std::endl;
            return false;
        }

        if (!goals_.empty() && !utils::GoalSpheresReachable(start_, goals_, rad_curv_, ins_length_, pos_tolerance_)) {
            std::cout << "None of the goal points are reachable from current start pose!" << std::endl;
            return false;
        }

        std::cout << "Valid problem with " << goals_.size() << " goal spheres." << std::endl;
        return true;
    }

    bool Valid(const State& s, const RealNum& length=0) const {
        if (utils::ExceedAngleConstraint(s, start_, ang_constraint_rad_)) {
            return false;
        }

        if (base::InCollision(s)) {
            return false;
        }

        if (!goals_.empty() && !utils::GoalSpheresReachable(s, goals_, rad_curv_, ins_length_ - length, pos_tolerance_)) {
            return false;
        }

        return true;
    }

    bool ValidMotion(const State& from, const std::vector<State>& motion, const unsigned& offset) const {
        return utils::ValidMotion(from, motion, base::env_, offset);
    }

    const RealNum& ConfigTolerance() const {
        return config_tolerance_;
    }

    std::optional<State> DirectConnectingStart(const State& s) const {
        if (fix_orientation_) {
            return {};
        }

        return utils::DirectConnectingWithoutCollisionCheck(s, start_, base::env_, rad_curv_,
                validity_res_, rotate_pi_x_);
    }

    void InitHEALPix() {
        std::ifstream fin;
        fin.open(healpix_file_);
        if (!fin.is_open()) {
            throw std::runtime_error("Failed to open " + healpix_file_);
        }

        Str line;
        Vec3 point;
        while (std::getline(fin, line)) {
            std::istringstream s(line);
            for (unsigned i = 0; i < 3; ++i) {
                s >> point[i];
            }

            healpix_vecs_.push_back(point);
        }

        MPT_LOG(INFO) << "Loaded " << healpix_vecs_.size() << " HEALPix points.";
    }

    std::optional<State> IterateNextStart() {
        if (healpix_idx_ >= healpix_vecs_.size()) {
            return {};
        }

        State res = start_;
        res.rotation() = Quat::FromTwoVectors(Vec3::UnitZ(), healpix_vecs_[healpix_idx_].normalized());
        healpix_idx_++;

        return res;
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
    const Str healpix_file_;
    RealNum config_tolerance_;
    RealNum max_insertion_;

    State start_;
    std::vector<Vec3> goals_;
    Quat rotate_pi_x_;

    std::vector<Vec3> healpix_vecs_;
    SizeType healpix_idx_{0};
};

} // namespace unc::robotics::snp

#endif // SNP_NEEDLE_VALIDATOR_H