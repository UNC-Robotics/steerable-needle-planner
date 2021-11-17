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

#include "utils.h"

namespace unc::robotics::snp {

using State = unc::robotics::mpt::SE3State<RealNum>;

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

    const RealNum x = d*sin(acos(std::fmin(1, y/d)));

    const RealNum dist_to_center = Vec2(x-rad, y).norm();

    if (dist_to_center > rad) {
        return true;
    }

    return false;
}

bool InTrumpet(const Vec3& sp, const Quat& sq, const Vec3& gp, const RealNum& rad) {
    return InTrumpet(sp, sq.normalized()*Vec3::UnitZ(), gp, rad);
}

RealNum ComputeRadiusOfCurvature(const Vec3& sp, const Vec3& st, const Vec3& gp) {
    const Vec3 sg = gp - sp;
    const RealNum d = sg.norm();
    const RealNum cos_theta = (sg.normalized()).dot(st.normalized());

    if (cos_theta > 1 - EPS) {
        return R_INF;
    }

    return 0.5*d/sin(acos(cos_theta));
}

RealNum ComputeRadiusOfCurvature(const Vec3& sp, const Quat& sq, const Vec3& gp) {
    return ComputeRadiusOfCurvature(sp, sq.normalized()*Vec3::UnitZ(), gp);
}

RealNum ComputeDistanceToTrumpetBoundary(const Vec3& sp, const Vec3& st, const Vec3& gp,
        const RealNum& rad, const RealNum& ang_tolerance) {
    const Vec3 sg = gp - sp;
    const RealNum d = sg.norm();
    const Vec3 tang = st.normalized();

    const RealNum y = sg.dot(tang);

    if (y > 0) {
        const RealNum x = d*sin(acos(std::fmin(1, y/d)));
        Vec2 center(rad*cos(ang_tolerance), -rad*sin(ang_tolerance));
        const RealNum dist_to_center = (Vec2(x, y) - center).norm();

        return rad - dist_to_center;
    }

    return R_INF;
}

RealNum FarthestDistanceToTrumpetBoundary(const Vec3& sp, const Vec3& st, const Vec3& gp,
        const RealNum& rad, const RealNum& ang_tolerance) {
    const Vec3 sg = gp - sp;
    const RealNum d = sg.norm();
    const Vec3 tang = st.normalized();

    const RealNum y = sg.dot(tang);

    if (y > 0) {
        const RealNum x = d*sin(acos(std::fmin(1, y/d)));
        Vec2 center(rad*cos(ang_tolerance), -rad*sin(ang_tolerance));
        const RealNum dist_to_center = (Vec2(-x, y) - center).norm();

        return rad - dist_to_center;
    }

    return R_INF;
}

bool WorkspaceConnected(const Vec3& sp, const Quat& sq, const Vec3& gp, const Quat& gq,
                        const RealNum& rad, const RealNum& pos_tolerance) {
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
                        const RealNum& rad, const RealNum& pos_tolerance, const RealNum& ang_tolerance) {
    const RealNum proceed_ang = M_PI/180;
    const RealNum dist_required = rad + pos_tolerance;
    const Idx max_num = 2*M_PI/proceed_ang;
    const Quat sq_normalized = sq.normalized();
    const Quat gq_normalized = gq.normalized();
    const Quat s_proceed(AngleAxis(proceed_ang, (sq_normalized*Vec3::UnitZ()).normalized()));
    const Vec3 gt = (gq_normalized*Vec3::UnitZ()).normalized();

    Vec3 s_iter = rad*((sq_normalized*Vec3::UnitX()).normalized());

    for (Idx i = 0; i < max_num; ++i) {
        RealNum dist = FarthestDistanceToTrumpetBoundary(gp, -gt, s_iter + sp, rad, ang_tolerance);

        if (-dist < dist_required) {
            return false;
        }

        s_iter = s_proceed*s_iter;
    }

    return true;
}

std::tuple<Vec3, Quat, RealNum> ForwardToCore(const Vec3& sp, const Quat& sq, const Vec3& gp,
        const RealNum& rad) {
    const Quat sq_normalized = sq.normalized();
    const Vec3 tang = (sq_normalized*Vec3::UnitZ()).normalized();

    const Vec3 sg = gp - sp;
    const RealNum d = sg.norm();

    const RealNum cos_theta = (sg.normalized()).dot(tang);

    if (cos_theta > 1 - EPS) {
        const Vec3 result_p = sp + tang * d;
        return {result_p, sq_normalized, d};
    }

    const RealNum r = std::fmax(rad, 0.5*d/sin(acos(cos_theta)));
    const Vec3 normal_vec = (tang.cross(sg)).normalized();

    const Vec3 center = sp + r*(normal_vec.cross(tang));
    const RealNum proceed_ang = acos(((gp - center).normalized()).dot((sp - center).normalized()));
    const Quat proceed_quat(AngleAxis(proceed_ang, normal_vec));

    const Vec3 result_p = proceed_quat*(sp - center) + center;
    const Quat result_q = proceed_quat*sq_normalized;

    return {result_p, result_q.normalized(), proceed_ang*r};
}

State ForwardTo(const State& from, const State& to, const RealNum& rad) {
    auto [p, q, dist] = ForwardToCore(from.translation(), from.rotation(), to.translation(), rad);

    State s;
    s.translation() = p;
    s.rotation() = q;

    return s;
}

State ForwardTo(const State& from, const Vec3& gp, const RealNum& rad) {
    auto [p, q, dist] = ForwardToCore(from.translation(), from.rotation(), gp, rad);

    State s;
    s.translation() = p;
    s.rotation() = q;

    return s;
}

State ForwardTo(const Vec3& sp, const Quat& sq, const Vec3& gp, const RealNum& rad) {
    auto [p, q, dist] = ForwardToCore(sp, sq, gp, rad);

    State s;
    s.translation() = p;
    s.rotation() = q;

    return s;
}

std::vector<State> Interpolate(const Vec3& sp, const Quat& sq, const Vec3& gp, const Quat& gq,
                               const RealNum& rad, const RealNum& step_size) {
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
    const Vec3 normal_vec = (st.cross(gt)).normalized();

    if (normal_vec.dot(sg.normalized()) > EPS) {
        std::cerr << "[Interpolate] Not on a plane, using linear interpolation." << std::endl;
        return LinearInterpolate(sp, gp, step_size);
    }

    if (ComputeDistanceToTrumpetBoundary(sp, st, gp, rad) > EPS) {
        std::cerr << "[Interpolate] Target point is outside the trumpet, returning empty interpolation." <<
                  std::endl;
        return path;
    }

    const RealNum cos_theta = (sg.normalized()).dot(st);

    State s;
    if (cos_theta > 1 - EPS) {
        for (RealNum l = step_size; l < d; l += step_size) {
            s.translation() = sp + st * l;
            s.rotation() = sq_normalized;

            path.push_back(s);
        }

        return path;
    }

    const RealNum r = 0.5*d/sin(acos(cos_theta));

    const Vec3 center = sp + r*(normal_vec.cross(st));
    const RealNum max_ang = acos(std::fmin(1,
                                           ((gp - center).normalized()).dot((sp - center).normalized())));
    const RealNum angle_step = step_size/r;

    for (RealNum ang = angle_step; ang < max_ang; ang += angle_step) {
        Quat proceed_quat(AngleAxis(ang, normal_vec));
        s.translation() = proceed_quat*(sp - center) + center;
        s.rotation() = (proceed_quat*sq_normalized).normalized();

        path.push_back(s);
    }

    return path;
}

std::vector<State> Interpolate(const State& from, const State& to, const RealNum& rad,
                               const RealNum& step_size) {
    return Interpolate(from.translation(), from.rotation(), to.translation(), to.rotation(), rad,
                       step_size);
}

std::vector<State> AccurateInterpolate(const State& from, const State& to, const RealNum& rad,
                                       const RealNum& step_size, RealNum* remainder) {
    const Vec3& sp = from.translation();
    const Vec3& gp = to.translation();
    const Quat& sq_normalized = from.rotation().normalized();
    const Quat& gq_normalized = to.rotation().normalized();
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
    const Vec3 normal_vec = (st.cross(gt)).normalized();
    const RealNum d = sg.norm();

    if (normal_vec.dot(sg.normalized()) > EPS) {
        const Quat uniformed_quat = Quat::FromTwoVectors(Vec3::UnitZ(), sg.normalized()).normalized();

        for (RealNum l = first_step; l < d; l += step_size) {
            RealNum portion = l / d;
            path.emplace_back(uniformed_quat, ((1-portion)*sp + portion*gp));
        }

        std::cerr << "[Interpolate] Not on a plane, returning linear interpolation." << std::endl;
        return path;
    }

    if (ComputeDistanceToTrumpetBoundary(sp, st, gp, rad) > EPS) {
        std::cerr << "[Interpolate] Target point is outside the trumpet, returning empty interpolation." <<
                  std::endl;
        return path;
    }

    const RealNum cos_theta = std::fmin(1, (sg.normalized()).dot(st));

    if (cos_theta > 1.0 - EPS) {
        for (RealNum l = first_step; l < d; l += step_size) {
            path.emplace_back(sq_normalized, sp + st*l);
        }
    }
    else {
        const RealNum r = 0.5*d/sin(acos(cos_theta));
        const Vec3 center = sp + r*(normal_vec.cross(st));

        const RealNum first_ang = first_step/r;
        const RealNum max_ang = acos(std::fmin(1,
                                               ((gp - center).normalized()).dot((sp - center).normalized())));
        const RealNum angle_step = step_size/r;

        for (RealNum ang = first_ang; ang < max_ang; ang += angle_step) {
            Quat proceed_quat(AngleAxis(ang, normal_vec));
            path.emplace_back(proceed_quat*sq_normalized, proceed_quat*(sp - center) + center);
        }
    }

    return path;
}

std::vector<State> InterpolatePath(const std::vector<State>& path, const RealNum& rad,
                                   const RealNum& step_size) {
    if (path.size() < 2) {
        return path;
    }

    std::vector<State> fine_path;
    RealNum remainder = step_size;

    for (SizeType i = 1; i < path.size(); ++i) {
        const std::vector<State> states = AccurateInterpolate(path[i-1], path[i], rad, step_size, &remainder);

        for (const auto& state : states) {
            fine_path.push_back(state);
        }
    }

    if (remainder > EPS) {
        fine_path.push_back(path.back());
    }

    return fine_path;
}

std::vector<State> LinearInterpolate(const Vec3& sp, const Vec3& gp, const RealNum& step_size) {
    std::vector<State> path;
    const Vec3 sg = gp - sp;
    const RealNum dist = sg.norm();
    const Quat uniformed_quat = Quat::FromTwoVectors(Vec3::UnitZ(), sg.normalized()).normalized();

    for (RealNum l = step_size; l < dist; l += step_size) {
        RealNum portion = l / dist;
        path.emplace_back(uniformed_quat, ((1-portion)*sp + portion*gp));
    }

    return path;
}

RealNum CurveLength(const Vec3& sp, const Quat& sq, const Vec3& gp, const Quat& gq) {
    const Vec3 st = (sq.normalized()*Vec3::UnitZ()).normalized();
    const Vec3 gt = (gq.normalized()*Vec3::UnitZ()).normalized();
    const Vec3 sg = gp - sp;
    const RealNum d = sg.norm();

    if (d < EPS) {
        return d;
    }

    if (std::abs(sg.dot(st) - d) < EPS) {
        return d;
    }

    const Vec3 normal_vec = (st.cross(gt)).normalized();

    if (normal_vec.dot(sg.normalized()) > EPS) {
        std::cerr << "[CurveLength] Not on a plane, returning approximate length." << std::endl;
        return d;
    }

    const RealNum cos_alpha = std::fmax(-1, std::fmin(1, st.dot(gt)));
    if (cos_alpha > 1 - EPS) {
        return d;
    }
    const RealNum r = 0.5*d/(sqrt((1 - cos_alpha)/2));

    if (std::isnan(r*acos(cos_alpha))) {
        throw std::runtime_error("[CurveLength] Get nan curve length!");
    }

    return r*acos(cos_alpha);
}

RealNum CurveLength(const State& from, const State& to) {
    return CurveLength(from.translation(), from.rotation(), to.translation(), to.rotation());
}

RealNum DirectionDifference(const Quat& q0, const Quat& q1) {
    return DirectionDifference(q0.normalized()*Vec3::UnitZ(), q1.normalized()*Vec3::UnitZ());
}

RealNum DirectionDifference(const Vec3& t0, const Vec3& t1) {
    const RealNum cos_alpha = std::fmin(1, t0.normalized().dot(t1.normalized()));
    return acos(cos_alpha);
}

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

void PrintState(const State& s, std::ostream& out) {
    const Vec3& p = s.translation();
    const Quat& q = s.rotation();
    out << p[0] << " " << p[1] << " " << p[2] << " "
        << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
        << std::endl;
}

void PrintPosition(const State& s, std::ostream& out) {
    const Vec3& p = s.translation();
    out << p[0] << " " << p[1] << " " << p[2] << std::endl;
}

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

bool WritePathToFile(const std::vector<State>& path, const Str& file_name, const bool full_state,
                     const bool show_log) {
    std::ofstream fout;
    fout.open(file_name);

    if (!fout.is_open()) {
        throw std::runtime_error("Failed to open " + file_name);
    }

    PrintPath(path, fout, full_state);
    fout.close();

    if (show_log) {
        std::cout << "Result with " << path.size() <<  " nodes written to " << file_name << std::endl;
    }

    return true;
}

double RelativeTime(const TimePoint& start) {
    return std::chrono::duration_cast<std::chrono::duration<double>>(Clock::now() - start).count();
}

double TimeDuration(const Clock::duration& elapsed) {
    return std::chrono::duration_cast<std::chrono::duration<double>>(elapsed).count();
}

}
