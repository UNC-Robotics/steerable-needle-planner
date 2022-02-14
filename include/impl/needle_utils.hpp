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

namespace unc::robotics::snp {

bool InTrumpet(const Vec3& sp, const Vec3& st, const Vec3& gp, const RealNum& rad) {
    const Vec3 sg = gp - sp;
    const RealNum d = sg.norm();
    const RealNum y = sg.dot(st.normalized());

    if (d < EPS) {
        return true;
    }

    if (y < 0) {
        return false;
    }

    const RealNum x = d * std::sin(std::acos(std::fmin(1, y / d)));
    const RealNum dist_to_center = Vec2(x - rad, y).norm();

    return (dist_to_center > rad - EPS);
}

bool InTrumpet(const Vec3& sp, const Quat& sq, const Vec3& gp, const RealNum& rad) {
    return InTrumpet(sp, sq.normalized() * Vec3::UnitZ(), gp, rad);
}

RealNum RadiusOfCurvature(const Vec3& sp, const Vec3& st, const Vec3& gp) {
    const Vec3 sg = gp - sp;
    const RealNum d = sg.norm();
    const RealNum cos_theta = (sg.normalized()).dot(st.normalized());

    if (cos_theta > 1 - EPS) {
        return R_INF;
    }

    return 0.5 * d / std::sin(std::acos(cos_theta));
}

RealNum RadiusOfCurvature(const Vec3& sp, const Quat& sq, const Vec3& gp) {
    return RadiusOfCurvature(sp, sq.normalized() * Vec3::UnitZ(), gp);
}

RealNum RadiusOfCurvature(const Vec3& sp, const Vec3& st, const Vec3& gp,
                          const RealNum& goal_tolerance)
{
    const Vec3 sg = gp - sp;
    const RealNum d = sg.norm();

    if (d < goal_tolerance) {
        throw std::runtime_error("[RadiusOfCurvature] Goal point is too close!");
    }

    const Vec3 tang = st.normalized();
    const RealNum y = sg.dot(tang);
    const RealNum x = d * std::sin(std::acos(std::fmin(1, y / d)));
    const RealNum l = x - goal_tolerance;
    const RealNum tmp = (y * y - l * l) / (2 * l);

    return x + tmp;
}

RealNum RadiusOfCurvature(const Vec3& sp, const Quat& sq, const Vec3& gp,
                          const RealNum& goal_tolerance)
{
    return RadiusOfCurvature(sp, sq.normalized() * Vec3::UnitZ(), gp, goal_tolerance);
}

RealNum DistanceToTrumpetBoundary(const Vec3& sp, const Vec3& st, const Vec3& gp,
                                  const RealNum& rad, const RealNum& ang_tolerance)
{
    const Vec3 sg = gp - sp;
    const RealNum d = sg.norm();

    if (d < EPS) {
        return 0.0;
    }

    const Vec3 tang = st.normalized();
    const RealNum y = sg.dot(tang);

    if (y > 0) {
        const RealNum x = d * std::sin(std::acos(std::fmin(1, y / d)));
        Vec2 center(rad * std::cos(ang_tolerance), -rad * std::sin(ang_tolerance));
        const RealNum dist_to_center = (Vec2(x, y) - center).norm();

        return rad - dist_to_center;
    }

    return R_INF;
}

RealNum MaxDistanceToTrumpetBoundary(const Vec3& sp, const Vec3& st, const Vec3& gp,
                                     const RealNum& rad, const RealNum& ang_tolerance)
{
    const Vec3 sg = gp - sp;
    const RealNum d = sg.norm();

    if (d < EPS) {
        return 0.0;
    }

    const Vec3 tang = st.normalized();
    const RealNum y = sg.dot(tang);

    if (y > 0) {
        const RealNum x = d * std::sin(std::acos(std::fmin(1, y / d)));
        Vec2 center(rad * std::cos(ang_tolerance), -rad * std::sin(ang_tolerance));
        const RealNum dist_to_center = (Vec2(-x, y) - center).norm();

        return rad - dist_to_center;
    }

    return R_INF;
}

bool WorkspaceConnected(const Vec3& sp, const Quat& sq, const Vec3& gp, const Quat& gq,
                        const RealNum& rad, const RealNum& pos_tolerance)
{
    const RealNum proceed_ang = M_PI/180;
    const RealNum dist_required = 2*rad + pos_tolerance;
    const Idx max_num = 2*M_PI/proceed_ang;
    const Quat sq_normalized = sq.normalized();
    const Quat gq_normalized = gq.normalized();
    const Quat s_proceed(AngleAxis(proceed_ang, (sq_normalized*Vec3::UnitZ()).normalized()));
    const Quat g_proceed(AngleAxis(proceed_ang, (gq_normalized*Vec3::UnitZ()).normalized()));

    Vec3 s_iter = rad*((sq_normalized*Vec3::UnitX()).normalized());
    Vec3 g_iter = rad*((gq_normalized*Vec3::UnitX()).normalized());

    bool valid = false;

    for (Idx i = 0; i < max_num; ++i) {
        valid = false;

        for (Idx j = 0; j < max_num; ++j) {
            RealNum d = ((s_iter + sp) - (g_iter + gp)).norm();

            if (d > dist_required) {
                valid = true;
                break;
            }

            g_iter = g_proceed*g_iter;
        }

        if (!valid) {
            return false;
        }

        s_iter = s_proceed*s_iter;
    }

    return true;
}

bool WorkspaceConnected(const Vec3& sp, const Quat& sq, const Vec3& gp, const Quat& gq,
                        const RealNum& rad, const RealNum& pos_tolerance,
                        const RealNum& ang_tolerance)
{
    const RealNum proceed_ang = M_PI/180;
    const RealNum dist_required = rad + pos_tolerance;
    const Idx max_num = 2*M_PI/proceed_ang;
    const Quat sq_normalized = sq.normalized();
    const Quat gq_normalized = gq.normalized();
    const Quat s_proceed(AngleAxis(proceed_ang, (sq_normalized*Vec3::UnitZ()).normalized()));
    const Vec3 gt = (gq_normalized*Vec3::UnitZ()).normalized();

    Vec3 s_iter = rad*((sq_normalized*Vec3::UnitX()).normalized());

    for (Idx i = 0; i < max_num; ++i) {
        const RealNum dist = MaxDistanceToTrumpetBoundary(gp, -gt, s_iter + sp, rad, ang_tolerance);

        if (-dist < dist_required) {
            return false;
        }

        s_iter = s_proceed*s_iter;
    }

    return true;
}

std::tuple<Vec3, Quat, RealNum> ForwardToCore(const Vec3& sp, const Quat& sq, const Vec3& gp, const RealNum& rad) {
    const Quat sq_normalized = sq.normalized();
    const Vec3 tang = (sq_normalized*Vec3::UnitZ()).normalized();
    const Vec3 sg = gp - sp;
    const RealNum d = sg.norm();
    const RealNum cos_theta = (sg.normalized()).dot(tang);

    if (d < EPS || cos_theta > 1 - EPS) {
        const Vec3 result_p = sp + tang * d;
        return {result_p, sq_normalized, d};
    }

    const RealNum r = std::fmax(rad, 0.5 * d / std::sin(std::acos(cos_theta)));
    const Vec3 normal_vec = (tang.cross(sg)).normalized();
    const Vec3 center = sp + r * (normal_vec.cross(tang));
    const RealNum proceed_ang = std::acos(((gp - center).normalized()).dot((sp - center).normalized()));
    const Quat proceed_quat(AngleAxis(proceed_ang, normal_vec));
    const Vec3 result_p = proceed_quat*(sp - center) + center;
    const Quat result_q = (proceed_quat*sq_normalized).normalized();

    if (std::isnan(result_q.w())) {
        throw std::runtime_error("[ForwardToCore] Get nan result quaternion!");
    }

    return {result_p, result_q, proceed_ang*r};
}

template<typename State>
State ForwardTo(const State& from, const State& to, const RealNum& rad) {
    auto [p, q, dist] = ForwardToCore(from.translation(), from.rotation(), to.translation(), rad);
    return State(q, p);
}

template<typename State>
State ForwardTo(const State& from, const Vec3& gp, const RealNum& rad) {
    auto [p, q, dist] = ForwardToCore(from.translation(), from.rotation(), gp, rad);
    return State(q, p);
}

template<typename State>
State ForwardTo(const Vec3& sp, const Quat& sq, const Vec3& gp, const RealNum& rad) {
    auto [p, q, dist] = ForwardToCore(sp, sq, gp, rad);
    return State(q, p);
}

template<typename State>
std::pair<std::vector<State>, RealNum> ForwardToWithPath(const State& from, const Vec3& gp, const RealNum& rad, const RealNum& step_size) {
    std::vector<State> path;

    const Vec3& sp = from.translation();
    const Vec3 sg = gp - sp;
    const RealNum d = sg.norm();

    if (d < EPS) {
        path.push_back(from);
        return {path, d};
    }

    const Quat& sq = from.rotation().normalized();
    const Vec3& st = (sq*Vec3::UnitZ()).normalized();
    const RealNum& cos_theta = (sg.normalized()).dot(st);

    if (cos_theta > 1 - EPS) {
        LinearInterpolate(sp, sq, sg.normalized(), step_size, d, path, -1, true);
        return {path, d};
    }

    const RealNum r = std::fmax(rad, 0.5 * d / std::sin(std::acos(cos_theta)));
    const Vec3 normal_vec = (st.cross(sg)).normalized();
    const Vec3 center = sp + r * (normal_vec.cross(st));
    const RealNum max_ang = std::acos(((gp - center).normalized()).dot((sp - center).normalized()));
    const RealNum ang_step = step_size / r;

    for (RealNum ang = ang_step; ang < max_ang; ang += ang_step) {
        Quat proceed_quat(AngleAxis(ang, normal_vec));
        path.emplace_back(proceed_quat*sq, proceed_quat*(sp - center) + center);
    }

    Quat proceed_quat(AngleAxis(max_ang, normal_vec));
    path.emplace_back(proceed_quat*sq, proceed_quat*(sp - center) + center);

    return {path, max_ang * r};
}

template<typename State>
std::tuple<std::vector<State>, RealNum, std::optional<State>> ShortestForwardToWithPath(const State& from, const Vec3& gp, const RealNum& rad,
                                                     const RealNum& step_size, const RealNum& pos_tolerance)
{
    std::vector<State> path;
    const Vec3& sp = from.translation();
    const Quat sq_normalized = from.rotation().normalized();
    const Vec3 sg = gp - sp;
    const Vec3 tang = (sq_normalized*Vec3::UnitZ()).normalized();
    const RealNum y = sg.dot(tang);

    if (y < 0) {
        path.push_back(from);
        return {path, R_INF, {}};
    }

    const RealNum d = sg.norm();

    if (d < EPS) {
        path.push_back(from);
        return {path, d, {}};
    }

    const RealNum x = d * std::sin(std::acos(y / d));

    const RealNum dist_to_center = Vec2(x - rad, y).norm();
    const Vec3 normal = (tang.cross(sg)).normalized();
    const Vec3 center = sp + rad*(normal.cross(tang));

    if (dist_to_center < rad) {
        const RealNum max_ang = std::acos(((gp - center).normalized()).dot((sp - center).normalized()));
        const RealNum ang_step = step_size / rad;

        for (RealNum ang = ang_step; ang < max_ang; ang += ang_step) {
            Quat proceed_quat(AngleAxis(ang, normal));
            path.emplace_back(proceed_quat*sq_normalized, proceed_quat*(sp - center) + center);
        }

        Quat proceed_quat(AngleAxis(max_ang, normal));
        path.emplace_back(proceed_quat*sq_normalized, proceed_quat*(sp - center) + center);

        return {path, max_ang * rad, {}};
    }

    const RealNum alpha = std::acos(std::fmin(1, rad / dist_to_center));
    const RealNum beta = std::asin(y / dist_to_center);
    const RealNum max_ang = x < rad ? (beta - alpha) : (M_PI - beta - alpha);
    const RealNum ang_step = step_size / rad;
    const RealNum curve_portion = max_ang * rad;

    for (RealNum ang = ang_step; ang < max_ang; ang += ang_step) {
        Quat proceed_quat(AngleAxis(ang, normal));
        path.emplace_back(proceed_quat*sq_normalized, proceed_quat*(sp - center) + center);
    }

    const Quat proceed_quat(AngleAxis(max_ang, normal));
    const Vec3 result_p = proceed_quat*(sp - center) + center;
    const Quat result_q = proceed_quat*sq_normalized;
    path.emplace_back(result_q, result_p);

    const RealNum straight_portion = std::fmax(0, dist_to_center * std::sin(alpha) - pos_tolerance + EPS);
    if (straight_portion > EPS) {
        const Vec3 tang_dir = (result_q*Vec3::UnitZ()).normalized();

        for (RealNum l = step_size; l < straight_portion; l += step_size) {
            path.emplace_back(result_q, result_p + tang_dir * l);
        }

        path.emplace_back(result_q, result_p + tang_dir * straight_portion);
    }

    return {path, curve_portion + straight_portion, State(result_q, result_p)};
}

RealNum CurveLength(const Vec3& sp, const Quat& sq, const Vec3& gp, const Quat& gq) {
    const Vec3 st = (sq.normalized()*Vec3::UnitZ()).normalized();
    const Vec3 gt = (gq.normalized()*Vec3::UnitZ()).normalized();
    const Vec3 sg = gp - sp;
    const RealNum d = sg.norm();
    const RealNum cos_alpha = (st.normalized()).dot(gt);

    if (d < EPS || cos_alpha > 1 - EPS) {
        return d;
    }

    const Vec3 normal_vec = (st.cross(gt)).normalized();

    if (normal_vec.dot(sg.normalized()) > EPS) {
        std::cerr << "[CurveLength] Not on a plane, returning approximate length." << std::endl;
        return d;
    }

    const RealNum r = 0.5 * d / std::sqrt((1 - cos_alpha) / 2);

    if (std::isnan(r * std::acos(cos_alpha))) {
        throw std::runtime_error("[CurveLength] Get nan curve length!");
    }

    return r * std::acos(cos_alpha);
}

template<typename State>
RealNum CurveLength(const State& from, const State& to) {
    return CurveLength(from.translation(), from.rotation(), to.translation(), to.rotation());
}

template<typename State>
std::vector<State> Interpolate(const Vec3& sp, const Quat& sq, const Vec3& gp, const Quat& gq,
                               const RealNum& rad, const RealNum& step_size)
{
    const Vec3 sg = gp - sp;
    const RealNum d = sg.norm();

    std::vector<State> path;

    if (d < step_size) {
        return path;
    }

    const Quat sq_normalized = sq.normalized();
    const Quat gq_normalized = gq.normalized();
    const Vec3 st = (sq_normalized*Vec3::UnitZ()).normalized();
    const Vec3 gt = (gq_normalized*Vec3::UnitZ()).normalized();
    const RealNum cos_alpha = (st).dot(gt);
    if (cos_alpha > 1 - EPS) {
        LinearInterpolate(sp, sq_normalized, sg.normalized(), step_size, d, path);
        return path;
    }

    const Vec3 normal_vec = (st.cross(gt)).normalized();
    if (normal_vec.dot(sg.normalized()) > EPS) {
        std::cout << "[Interpolate] Not on a plane, linear interpolation." << std::endl;
        const Quat uniformed_quat = Quat::FromTwoVectors(Vec3::UnitZ(), sg.normalized()).normalized();
        LinearInterpolate(sp, uniformed_quat, sg.normalized(), step_size, d, path);
        return path;
    }

    if (DistanceToTrumpetBoundary(sp, st, gp, rad) > EPS) {
        std::cout << "[Interpolate] Target outside the trumpet, empty interpolation." << std::endl;
        return path;
    }

    const RealNum r = 0.5 * d / std::sqrt((1 - cos_alpha) / 2);
    const Vec3 center = sp + r * (normal_vec.cross(st));
    const RealNum max_ang = std::acos(std::fmin(1, ((gp - center).normalized()).dot((sp - center).normalized())));
    const RealNum ang_step = step_size / r;

    CurveInterpolate(sp, sq_normalized, normal_vec, center, ang_step, max_ang, path);
    return path;
}

template<typename State>
std::vector<State> Interpolate(const State& from, const State& to, const RealNum& rad,
                               const RealNum& step_size)
{
    return Interpolate<State>(from.translation(), from.rotation(), to.translation(), to.rotation(), rad,
                       step_size);
}

template<typename State>
std::vector<State> AccurateInterpolate(const State& from, const State& to, const RealNum& rad,
                                       const RealNum& step_size, RealNum* remainder)
{
    const Vec3& sp = from.translation();
    const Vec3& gp = to.translation();
    const Quat sq_normalized = from.rotation().normalized();
    const Quat gq_normalized = to.rotation().normalized();
    const RealNum first_step = step_size - *remainder;
    const RealNum max_length = CurveLength(sp, sq_normalized, gp, gq_normalized);

    if (std::isnan(max_length)) {
        throw std::runtime_error("[AccurateInterpolate] Get nan curve length!");
    }

    std::vector<State> path;

    if (max_length < first_step) {
        *remainder += max_length;
        return path;
    }

    *remainder = fmod((max_length - first_step), step_size);

    const Vec3 st = (sq_normalized*Vec3::UnitZ()).normalized();
    const Vec3 gt = (gq_normalized*Vec3::UnitZ()).normalized();
    const Vec3 sg = gp - sp;
    const RealNum d = sg.norm();
    const RealNum cos_alpha = (st.normalized()).dot(gt);
    if (cos_alpha > 1 - EPS) {
        LinearInterpolate(sp, sq_normalized, sg.normalized(), step_size, d + EPS, path, first_step);
        return path;
    }

    const Vec3 normal_vec = (st.cross(gt)).normalized();
    if (normal_vec.dot(sg.normalized()) > EPS) {
        std::cout << "[AccurateInterpolate] Not on a plan, linear interpolation." << std::endl;
        const Quat uniformed_quat = Quat::FromTwoVectors(Vec3::UnitZ(), sg.normalized()).normalized();
        LinearInterpolate(sp, uniformed_quat, sg.normalized(), step_size, d + EPS, path, first_step);
        return path;
    }

    if (DistanceToTrumpetBoundary(sp, st, gp, rad) > EPS) {
        std::cout << "[AccurateInterpolate] Target outside the trumpet, empty interpolation." << std::endl;
        return path;
    }

    const RealNum r = 0.5 * d / std::sqrt((1 - cos_alpha) / 2);
    const Vec3 center = sp + r * (normal_vec.cross(st));
    const RealNum first_ang = first_step / r;
    const RealNum max_ang = std::acos(std::fmin(1, ((gp - center).normalized()).dot((sp - center).normalized())));
    const RealNum ang_step = step_size/r;

    CurveInterpolate(sp, sq_normalized, normal_vec, center, ang_step, max_ang + EPS, path, first_ang);
    return path;
}

template<typename State>
std::vector<State> InterpolatePath(const std::vector<State>& path, const RealNum& rad, const RealNum& step_size) {
    if (path.size() < 2) {
        return path;
    }

    std::vector<State> fine_path;
    RealNum remainder = step_size;
    fine_path.push_back(path[0]);

    for (SizeType i = 1; i < path.size(); ++i) {
        const std::vector<State> states = AccurateInterpolate(path[i-1], path[i], rad, step_size, &remainder);

        for (auto const& state : states) {
            if ((state.translation() - fine_path.back().translation()).norm() > 0.5 * step_size) {
                fine_path.push_back(state);
            }
        }
    }

    if (remainder > EPS) {
        fine_path.push_back(path.back());
    }

    return fine_path;
}

template<typename State>
RealNum ShortestDistance(const State& from, const Vec3& gp, const RealNum& rad_curv, const RealNum& pos_tolerance) {
    const Vec3& sp = from.translation();
    const Quat& sq = from.rotation();
    const Vec3 sg = gp - sp;
    const Vec3 tang = (sq*Vec3::UnitZ()).normalized();
    const RealNum y = sg.dot(tang);

    if (y < 0) {
        return R_INF;
    }

    const RealNum d = sg.norm();

    if (d < EPS) {
        return d;
    }

    const RealNum x = d * std::sin(std::acos(y / d));
    const RealNum dist_to_center = Vec2(x - rad_curv, y).norm();

    if (dist_to_center < rad_curv) {
        if (DistanceToTrumpetBoundary(sp, tang, gp, rad_curv) > pos_tolerance) {
            return R_INF;
        }

        const Vec3 normal = (tang.cross(sg)).normalized();
        const Vec3 center = sp + rad_curv*(normal.cross(tang));
        const RealNum max_ang = std::acos(((gp - center).normalized()).dot((sp - center).normalized()));

        return max_ang * rad_curv;
    }

    const RealNum alpha = std::acos(std::fmin(1, rad_curv / dist_to_center));
    const RealNum beta = std::asin(y / dist_to_center);

    const RealNum ang = x < rad_curv ? (beta - alpha) : (M_PI - beta - alpha);
    const RealNum straight_portion = std::fmax(0, dist_to_center * std::sin(alpha) - pos_tolerance + EPS);
    const RealNum curve_portion = ang * rad_curv;

    return curve_portion + straight_portion;
}

RealNum DirectionDifference(const Vec3& t0, const Vec3& t1) {
    const RealNum cos_alpha = std::fmin(1, t0.normalized().dot(t1.normalized()));
    return std::acos(cos_alpha);
}

RealNum DirectionDifference(const Quat& q0, const Quat& q1) {
    return DirectionDifference(q0.normalized() * Vec3::UnitZ(), q1.normalized() * Vec3::UnitZ());
}

template<typename State>
bool IsTheSameState(const State& a, const State& b) {
    const Vec3& ap = a.translation();
    const Quat aq = a.rotation().normalized();
    const Vec3& bp = b.translation();
    const Quat bq = b.rotation().normalized();

    if ((ap - bp).norm() < EPS && aq.angularDistance(bq) < EPS) {
        return true;
    }

    return false;
}

template<typename State>
RealNum DirectionalDistance(const State& from, const State& to, const RealNum& rad,
                            const RealNum& max_range)
{
    const Vec3& sp = from.translation();
    const Quat sq = from.rotation().normalized();
    const Vec3 st = (sq*Vec3::UnitZ()).normalized();
    const Vec3& gp = to.translation();
    const Quat gq = to.rotation().normalized();
    const Vec3 gt = (gq*Vec3::UnitZ()).normalized();

    const RealNum t_d = (sp - gp).norm();
    const RealNum r_d = std::acos(std::fmin(1, st.dot(gt)));
    const RealNum d = t_d + r_d;

    if (InTrumpet(sp, st, gp, rad)
        && InTrumpet(gp, -gt, sp, rad)
        && WorkspaceConnected(sp, sq, gp, gq, rad))
    {
        return d;
    }

    return max_range + d;
}

template<typename State>
void PrintState(const State& s, std::ostream& out) {
    const Vec3& p = s.translation();
    const Quat& q = s.rotation();
    out << p[0] << " " << p[1] << " " << p[2] << " "
        << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
        << std::endl;
}

template<typename State>
void PrintPosition(const State& s, std::ostream& out) {
    const Vec3& p = s.translation();
    out << p[0] << " " << p[1] << " " << p[2] << std::endl;
}

template<typename State>
void PrintPath(const std::vector<State>& path, std::ostream& out, const bool full_state) {
    for (const State& s : path) {
        if (full_state) {
            PrintState(s, out);
        }
        else {
            PrintPosition(s, out);
        }
    }
}

template<typename State>
bool WritePathToFile(const std::vector<State>& path, const Str& file_name,
                     const bool full_state, const bool show_log)
{
    std::ofstream fout;
    fout.open(file_name);

    if (!fout.is_open()) {
        throw std::runtime_error("Failed to open " + file_name);
    }

    PrintPath(path, fout, full_state);
    fout.close();

    if (show_log) {
        std::cout << "Result with " << path.size()
                  << " nodes written to " << file_name << std::endl;
    }

    return true;
}

double RelativeTime(const TimePoint& start) {
    return std::chrono::duration_cast<std::chrono::duration<double>>(Clock::now() - start).count();
}

double TimeDuration(const Clock::duration& elapsed) {
    return std::chrono::duration_cast<std::chrono::duration<double>>(elapsed).count();
}

} // namespace unc::robotics::snp