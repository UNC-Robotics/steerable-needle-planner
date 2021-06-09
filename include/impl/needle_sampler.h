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
#ifndef SNP_NEEDLE_SAMPLER_H
#define SNP_NEEDLE_SAMPLER_H

#include <cmath>

#include "../problem_config.h"
#include "../utils.h"

namespace unc::robotics::snp {

struct null_sampler {};
struct sample_sphere {};
struct sample_trumpet {};
struct sample_rugby {};
struct sample_intersection {};

namespace utils {

template<typename RNG, typename Uniform, typename Normal>
Vec3 SampleInUnitSphere(RNG& rng, Uniform& uniform, Normal& normal) {
    Vec3 result;

    result[0] = normal(rng);
    result[1] = normal(rng);
    result[2] = normal(rng);
    RealNum r = uniform(rng);
    result *= r/result.norm();

    return result;
}

template<typename RNG, typename Uniform, typename Normal>
Quat SampleOrientation(const Vec3& sp, const Vec3& p, const RealNum& rad_curv, RNG& rng,
                       Uniform& uniform, Normal& normal) {
    Vec3 sg = sp - p;
    RealNum d = sg.norm();
    RealNum alpha = asin(d/(2.0*rad_curv)) * uniform(rng);
    RealNum beta = 2*M_PI*uniform(rng);

    Vec3 dir;
    dir[2] = cos(alpha);
    dir[0] = sin(alpha)*sin(beta);
    dir[1] = sin(alpha)*cos(beta);

    Quat local = Quat::FromTwoVectors(Vec3::UnitZ(), sg.normalized());
    Vec3 tang = -(local*dir.normalized());

    return Quat::FromTwoVectors(Vec3::UnitZ(), tang).normalized();
}

template<typename RNG, typename Uniform, typename Normal>
Quat SampleOrientation(const Vec3& sp, const Vec3& gp, const Vec3& p, const Idx& max_iter,
                       const RealNum& rad_curv, RNG& rng, Uniform& uniform, Normal& normal) {
    bool valid = false;
    Vec3 sg = sp - p;
    RealNum d = sg.norm();
    RealNum alpha = asin(d/(2.0*rad_curv));
    RealNum beta = 2*M_PI;

    RealNum a, b;
    Vec3 dir;
    Quat local;
    Vec3 tang;
    Idx iter = 0;

    while (!valid) {
        a = alpha*uniform(rng);
        b = beta*uniform(rng);

        dir[2] = cos(a);
        dir[0] = sin(a)*sin(b);
        dir[1] = sin(a)*cos(b);

        local = Quat::FromTwoVectors(Vec3::UnitZ(), sg.normalized());
        tang = -(local*dir.normalized());

        if (InTrumpet(p, tang, gp, rad_curv) || iter > max_iter) {
            valid = true;
        }

        iter++;
    }

    return Quat::FromTwoVectors(Vec3::UnitZ(), tang).normalized();
}

template<typename RNG, typename Uniform>
std::pair<RealNum, RealNum> SampleInUnitCircle(RNG& rng, Uniform& uniform) {
    RealNum t = 2*M_PI * uniform(rng);
    RealNum u = uniform(rng) + uniform(rng);
    RealNum r = (u > 1) ? (2.0 - u) : u;

    return {t, r};
}

template<typename RNG, typename Uniform, typename Normal>
Vec3 SampleInTrumpet(const RealNum& rad_curv, const RealNum& ins_length, const RealNum& max_r,
                     RNG& rng, Uniform& uniform, Normal& normal) {
    Vec3 sample;
    bool valid = false;
    Vec2 center(rad_curv, 0);

    while (!valid) {
        sample[2] = uniform(rng)*ins_length;

        auto [t, r] = SampleInUnitCircle(rng, uniform);
        r *= max_r;

        Vec2 relative_2d(r, sample[2]);

        if ((relative_2d - center).norm() < rad_curv) {
            continue;
        }

        if (relative_2d.norm() > ins_length) {
            continue;
        }

        sample[0] = r*cos(t);
        sample[1] = r*sin(t);

        valid = true;
    }

    return sample;
}

Vec3 VirtualGoal(const Vec3& start, const Vec3& goal, const RealNum& rad_curv,
                 const RealNum& tolerance) {
    RealNum d = (goal - start).norm();
    RealNum r = rad_curv - tolerance;
    RealNum cos_theta = (d*d + rad_curv*rad_curv - r*r)/(2*d*rad_curv);
    RealNum virtual_d = 2 * rad_curv * cos_theta;

    return start + (goal - start).normalized() * virtual_d;
}

}

template <typename State, typename SubSpace>
class NeedleSampler {};

template<typename State>
class NeedleSampler<State, null_sampler> {
  public:
    NeedleSampler(const ConfigPtr cfg, const State& start)
        : start_(start) {
    }

    template <typename RNG>
    State operator() (RNG& rng) {
        return start_;
    }

  private:
    const State start_;
};

template<typename State>
class NeedleSampler<State, sample_sphere> {
  public:
    NeedleSampler(const ConfigPtr cfg, const State& start)
        : sample_orientation_(cfg->sample_orientation)
        , start_p_(start.translation())
        , start_q_(start.rotation().normalized())
        , rad_curv_(cfg->rad_curv)
        , ins_length_(cfg->ins_length + 10) {

    }

    template <typename RNG>
    State operator() (RNG& rng) {
        State sample;
        sample.translation() = start_p_ + ins_length_*utils::SampleInUnitSphere(rng, uniform_, normal_);

        if (sample_orientation_) {
            sample.rotation() = utils::SampleOrientation(start_p_, sample.translation(), rad_curv_, rng,
                                uniform_, normal_);
        }
        else {
            sample.rotation() = start_q_;
        }

        return sample;
    }

  private:
    const bool sample_orientation_;
    const Vec3 start_p_;
    const Quat start_q_;
    const RealNum rad_curv_;
    const RealNum ins_length_;

    RealUniformDist uniform_;
    RealNormalDist normal_;
};

template <typename State>
class NeedleSampler<State, sample_trumpet> {
  public:
    NeedleSampler(const ConfigPtr cfg, const State& start)
        : sample_orientation_(cfg->sample_orientation)
        , start_p_(start.translation())
        , start_q_(start.rotation().normalized())
        , rad_curv_(cfg->rad_curv)
        , ins_length_(cfg->ins_length + 10) {
        if (ins_length_ > rad_curv_) {
            if (ins_length_ > 2*rad_curv_) {
                max_r_ = ins_length_;
            }
            else {
                max_r_ = sqrt(ins_length_*ins_length_ - rad_curv_*rad_curv_);
            }
        }
        else {
            max_r_ = ins_length_*ins_length_/(2*rad_curv_);
        }
    }

    NeedleSampler(const ConfigPtr cfg, const State& start, const State& goal)
        : sample_orientation_(cfg->sample_orientation)
        , start_p_(start.translation())
        , start_q_(start.rotation().normalized())
        , rad_curv_(cfg->rad_curv)
        , ins_length_(cfg->ins_length + 10) {
        if (ins_length_ > rad_curv_) {
            if (ins_length_ > 2*rad_curv_) {
                max_r_ = ins_length_;
            }
            else {
                max_r_ = sqrt(ins_length_*ins_length_ - rad_curv_*rad_curv_);
            }
        }
        else {
            max_r_ = ins_length_*ins_length_/(2*rad_curv_);
        }
    }

    template <typename RNG>
    State operator() (RNG& rng) {
        State sample;
        sample.translation() = start_p_ + start_q_*utils::SampleInTrumpet(rad_curv_, ins_length_, max_r_,
                               rng, uniform_, normal_);

        if (sample_orientation_) {
            sample.rotation() = utils::SampleOrientation(start_p_, sample.translation(), rad_curv_, rng,
                                uniform_, normal_);
        }
        else {
            sample.rotation() = start_q_;
        }

        return sample;
    }

  private:
    const bool sample_orientation_;
    const Vec3 start_p_;
    const Quat start_q_;
    const RealNum rad_curv_;
    const RealNum ins_length_;

    RealNum max_r_;

    RealUniformDist uniform_;
    RealNormalDist normal_;
};

template <typename State>
class NeedleSampler<State, sample_rugby> {
  public:
    NeedleSampler(const ConfigPtr cfg, const State& start, const State& goal)
        : sample_orientation_(cfg->sample_orientation)
        , start_p_(start.translation())
        , start_q_(start.rotation().normalized())
        , goal_p_(utils::VirtualGoal(start.translation(), goal.translation(), cfg->rad_curv,
                                     cfg->goal_pos_tolerance))
        , true_goal_p_(goal.translation())
        , goal_q_(goal.rotation().normalized())
        , pos_tolerance_(cfg->goal_pos_tolerance)
        , rad_curv_(cfg->rad_curv)
        , ins_length_(cfg->ins_length) {
        start_t_ = start_q_*Vec3::UnitZ();
        goal_t_ = goal_q_*Vec3::UnitZ();
        sg_ = goal_p_ - start_p_;
        rad_curv2_ = rad_curv_ * rad_curv_;

        max_h_ = sg_.norm();
        true_h_ = (true_goal_p_ - start_p_).norm() + pos_tolerance_;
        center_[1] = 0.5*max_h_;

        if (max_h_ > 2*rad_curv_) {
            sample_sphere_ = true;
            sphere_rad_ = center_[1];
            sphere_center_ = 0.5*(start_p_ + goal_p_);
        }
        else {
            sample_sphere_ = false;
            center_[0] = - sqrt(rad_curv2_ - (center_[1])*(center_[1]));
        }

        rugby_q_ = Quat::FromTwoVectors(Vec3::UnitZ(), sg_).normalized();
    }

    template <typename RNG>
    State operator() (RNG& rng) {
        State sample;

        if (!sample_sphere_) {
            sample.translation() = this->sampleInRugby(rng);
        }
        else {
            sample.translation() = this->sampleInSphere(rng);
        }

        if (sample_orientation_) {
            sample.rotation() = utils::SampleOrientation(start_p_, sample.translation(), rad_curv_, rng,
                                uniform_, normal_);
        }
        else {
            sample.rotation() = start_q_;
        }

        return sample;
    }

  private:
    const bool sample_orientation_;
    const Vec3 start_p_;
    const Quat start_q_;
    const Vec3 goal_p_;
    const Vec3 true_goal_p_;
    const Quat goal_q_;
    const RealNum pos_tolerance_;
    const RealNum rad_curv_;
    const RealNum ins_length_;

    bool sample_sphere_;
    Vec2 center_;
    Vec3 start_t_, goal_t_, sg_, sphere_center_;
    RealNum rad_curv2_, max_h_, true_h_, sphere_rad_;
    Quat rugby_q_;

    RealUniformDist uniform_;
    RealNormalDist normal_;

    template <typename RNG>
    Vec3 sampleInRugby(RNG& rng) {
        Vec3 sample;
        bool valid = false;

        while(!valid) {
            sample[2] = true_h_*uniform_(rng);

            RealNum max_r;
            RealNum z = abs(sample[2] - center_[1]);
            max_r = sqrt(abs(rad_curv2_ - z*z)) + center_[0];

            auto [t, r] = utils::SampleInUnitCircle(rng, uniform_);
            r *= max_r;

            sample[0] = r * cos(t);
            sample[1] = r * sin(t);

            sample = start_p_ + rugby_q_*sample;

            valid = true;
        }

        return sample;
    }

    template <typename RNG>
    Vec3 sampleInSphere(RNG& rng) {
        Vec3 sample;
        bool valid = false;

        while(!valid) {
            sample = sphere_center_ + sphere_rad_*utils::SampleInUnitSphere(rng, uniform_, normal_);
            valid = true;
        }

        return sample;
    }
};

template <typename State>
class NeedleSampler<State, sample_intersection> {
  public:
    NeedleSampler(const ConfigPtr cfg, const State& start, const State& goal)
        : sample_orientation_(cfg->sample_orientation)
        , constrain_goal_orientation_(cfg->constrain_goal_orientation)
        , start_p_(start.translation())
        , start_q_(start.rotation().normalized())
        , goal_p_(utils::VirtualGoal(start.translation(), goal.translation(), cfg->rad_curv,
                                     cfg->goal_pos_tolerance))
        , true_goal_p_(goal.translation())
        , goal_q_(goal.rotation().normalized())
        , pos_tolerance_(cfg->goal_pos_tolerance)
        , ang_tolerance_(cfg->goal_ang_tolerance)
        , rad_curv_(cfg->rad_curv)
        , ins_length_(cfg->ins_length) {
        start_t_ = start_q_*Vec3::UnitZ();
        goal_t_ = goal_q_*Vec3::UnitZ();
        sg_ = goal_p_ - start_p_;
        rad_curv2_ = rad_curv_ * rad_curv_;

        max_h_ = sg_.norm();
        true_h_ = (true_goal_p_ - start_p_).norm() + pos_tolerance_;
        center_[1] = 0.5*max_h_;

        if (max_h_ > 2*rad_curv_) {
            sample_sphere_ = true;
            sphere_rad_ = center_[1];
            sphere_center_ = 0.5*(start_p_ + goal_p_);
        }
        else {
            sample_sphere_ = false;
            center_[0] = - sqrt(rad_curv2_ - (center_[1])*(center_[1]));
        }

        rugby_q_ = Quat::FromTwoVectors(Vec3::UnitZ(), sg_).normalized();
    }

    template <typename RNG>
    State operator() (RNG& rng) {
        State sample;

        if (!sample_sphere_) {
            sample.translation() = this->sampleInRugbyIntersectsTrumpet(rng);
        }
        else {
            sample.translation() = this->sampleInSphereIntersectsTrumpet(rng);
        }


        if (sample_orientation_) {
            sample.rotation() = utils::SampleOrientation(start_p_, sample.translation(), rad_curv_, rng,
                                uniform_, normal_);
        }
        else {
            sample.rotation() = start_q_;
        }

        return sample;
    }

  private:
    const bool sample_orientation_;
    const bool constrain_goal_orientation_;
    const Vec3 start_p_;
    const Quat start_q_;
    const Vec3 goal_p_;
    const Vec3 true_goal_p_;
    const Quat goal_q_;
    const RealNum pos_tolerance_;
    const RealNum ang_tolerance_;
    const RealNum rad_curv_;
    const RealNum ins_length_;

    bool sample_sphere_;
    Vec2 center_;
    Vec3 start_t_, goal_t_, sg_, sphere_center_;
    RealNum rad_curv2_, max_h_, true_h_, sphere_rad_;
    Quat rugby_q_;

    RealUniformDist uniform_;
    RealNormalDist normal_;

    template <typename RNG>
    Vec3 sampleInRugbyIntersectsTrumpet(RNG& rng) {
        Vec3 sample;
        bool valid = false;

        while(!valid) {
            sample[2] = true_h_*uniform_(rng);

            RealNum max_r;
            RealNum z = abs(sample[2] - center_[1]);
            max_r = sqrt(abs(rad_curv2_ - z*z)) + center_[0];

            auto [t, r] = utils::SampleInUnitCircle(rng, uniform_);
            r *= max_r;

            sample[0] = r * cos(t);
            sample[1] = r * sin(t);

            sample = start_p_ + rugby_q_*sample;

            if (!InTrumpet(start_p_, start_t_, sample, rad_curv_)) {
                continue;
            }

            if (constrain_goal_orientation_ &&
                    ComputeDistanceToTrumpetBoundary(goal_p_, -goal_t_, sample, rad_curv_,
                            ang_tolerance_) > pos_tolerance_) {
                continue;
            }

            valid = true;
        }

        return sample;
    }

    template <typename RNG>
    Vec3 sampleInSphereIntersectsTrumpet(RNG& rng) {
        Vec3 sample;
        bool valid = false;

        while(!valid) {
            sample = sphere_center_ + sphere_rad_*utils::SampleInUnitSphere(rng, uniform_, normal_);

            if (!InTrumpet(start_p_, start_t_, sample, rad_curv_)) {
                continue;
            }

            if (constrain_goal_orientation_ &&
                    ComputeDistanceToTrumpetBoundary(goal_p_, -goal_t_, sample, rad_curv_,
                            ang_tolerance_) > pos_tolerance_) {
                continue;
            }

            valid = true;
        }

        return sample;
    }
};

}

#endif
