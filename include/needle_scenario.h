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
#ifndef SNP_NEEDLE_SCENARIO_H
#define SNP_NEEDLE_SCENARIO_H

#include "impl/needle_planning_scenario.h"

namespace unc::robotics::snp {

template <typename Scalar>
struct Point2PointScenario {
    using Space = NeedleSpace<Scalar>;
    using State = typename Space::Type;
    using Sampler = NeedleSampler<State, sample_intersection>;
    using Propagator = CurvePropagator<State>;
    using Validator = Point2PointCurveValidator<State>;
    using Type = NeedlePlanningScenario<Space, Sampler, Propagator, Validator, 0>;
};

template <typename Scalar>
struct SpreadingScenario {
    using Space = NeedleSpace<Scalar>;
    using State = typename Space::Type;
    using Sampler = NeedleSampler<State, sample_sphere>;
    using Propagator = CurvePropagator<State>;
    using Validator = SpreadingValidator<State>;
    using Type = NeedlePlanningScenario<Space, Sampler, Propagator, Validator, 1>;
};

template <typename Scalar>
struct FixOrientationSpreadingScenario {
    using Space = NeedleSpace<Scalar>;
    using State = typename Space::Type;
    using Sampler = NeedleSampler<State, sample_trumpet>;
    using Propagator = CurvePropagator<State>;
    using Validator = SpreadingValidator<State>;
    using Type = NeedlePlanningScenario<Space, Sampler, Propagator, Validator, 1>;
};

template <typename Scalar>
struct PRCSPoint2PointScenario {
    using Space = WeightedSE3Space<Scalar>;
    using State = typename Space::Type;
    using Sampler = NeedleSampler<State, null_sampler>;
    using Propagator = MotionPrimitivePropagator<State>;
    using Validator = MotionPrimitiveValidator<State>;
    using Type = NeedlePlanningScenario<Space, Sampler, Propagator, Validator, 0>;
};

template <typename Scalar>
struct PRCSSpreadingScenario {
    using Space = WeightedSE3Space<Scalar>;
    using State = typename Space::Type;
    using Sampler = NeedleSampler<State, sample_sphere>;
    using Propagator = MotionPrimitivePropagator<State>;
    using Validator = MotionPrimitiveSpreadingValidator<State>;
    using Type = NeedlePlanningScenario<Space, Sampler, Propagator, Validator, 1>;
};

template <typename Scalar>
struct PAORRTPoint2PointScenario {
    using Space = ConfigCostSpace<Scalar>;
    using State = typename Space::Type;
    using Sampler = NeedleSampler<State, sample_random>;
    using Propagator = RandomForwardPropagator<State>;
    using Validator = Point2PointCurveValidator<State>;
    using Type = NeedlePlanningScenario<Space, Sampler, Propagator, Validator, 0>;
};

template <typename Scalar>
struct PAORRTSpreadingScenario {
    using Space = ConfigCostSpace<Scalar>;
    using State = typename Space::Type;
    using Sampler = NeedleSampler<State, sample_sphere>;
    using Propagator = RandomForwardPropagator<State>;
    using Validator = SpreadingValidator<State>;
    using Type = NeedlePlanningScenario<Space, Sampler, Propagator, Validator, 1>;
};

} // namespace unc::robotics::snp

#endif // SNP_NEEDLE_SCENARIO_H