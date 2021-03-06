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
#ifndef SNP_NEEDLE_PRCS_H
#define SNP_NEEDLE_PRCS_H

#include <mpt/planner.hpp>
#include <mpt/planner_tags.hpp>
#include <mpt/impl/packs.hpp>
#include <mpt/impl/pack_nearest.hpp>
#include <mpt/impl/nearest_strategy.hpp>

#include "impl/needle_planner_mode.h"
#include "impl/prcs/needle_prcs_impl.h"
#include "impl/prcs/needle_prcs_star_impl.h"
#include "impl/prcs/needle_spreading_prcs_impl.h"

namespace unc::robotics::mpt {

namespace impl {

template <int maxThreads, class Mode, bool reportStats, typename NNStrategy>
struct NeedlePRCSStrategy {};

template <typename ... Options>
struct NeedlePRCSOptions {
    static constexpr int maxThreads = pack_int_tag_v<max_threads, 0, Options...>;
    static constexpr bool reportStats = pack_bool_tag_v<report_stats, false, Options...>;

    static constexpr bool useSpreading = pack_contains_v<spreading, Options...>;
    static constexpr bool useOptimal = pack_contains_v<optimal, Options...>;

    static_assert(!(useSpreading && useOptimal), "Not implemented!");

    using Mode = std::conditional_t<!useSpreading, std::conditional_t<!useOptimal, point2point_planner, point2point_optimal_planner>,
                                                   std::conditional_t<!useOptimal, spreading_planner, spreading_optimal_planner>>;

    using NNStrategy = pack_nearest_t<Options...>;

    using type = NeedlePRCSStrategy<maxThreads, Mode, reportStats, NNStrategy>;
};

template <typename Scenario, int maxThreads, bool reportStats, typename NNStrategy>
struct PlannerResolver<Scenario, impl::NeedlePRCSStrategy<maxThreads, point2point_planner, reportStats, NNStrategy>> {
    using type = impl::prcs::NeedlePRCS<
                 Scenario, maxThreads, reportStats,
                 nearest_strategy_t<Scenario, maxThreads, NNStrategy>>;
};

template <typename Scenario, int maxThreads, bool reportStats, typename NNStrategy>
struct PlannerResolver<Scenario, impl::NeedlePRCSStrategy<maxThreads, spreading_planner, reportStats, NNStrategy>> {
    using type = impl::prcs::NeedleSpreadingPRCS<
                 Scenario, maxThreads, reportStats,
                 nearest_strategy_t<Scenario, maxThreads, NNStrategy>>;
};

template <typename Scenario, int maxThreads, bool reportStats, typename NNStrategy>
struct PlannerResolver<Scenario, impl::NeedlePRCSStrategy<maxThreads, point2point_optimal_planner, reportStats, NNStrategy>> {
    using type = impl::prcs::NeedlePRCSStar<
                 Scenario, maxThreads, reportStats,
                 nearest_strategy_t<Scenario, maxThreads, NNStrategy>>;
};

} // namespace impl

template <typename ... Options>
using NeedlePRCS = typename impl::NeedlePRCSOptions<Options...>::type;

} // namespace unc::robotics::mpt

#endif // SNP_NEEDLE_PRCS_H