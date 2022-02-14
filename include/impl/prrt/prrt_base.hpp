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

//! @author Jeff Ichnowski

#pragma once
#ifndef SNP_PRRT_BASE_H
#define SNP_PRRT_BASE_H

#include <mpt/impl/atom.hpp>
#include <mpt/impl/goal_has_sampler.hpp>
#include <mpt/impl/link_trajectory.hpp>
#include <mpt/impl/object_pool.hpp>
#include <mpt/impl/planner_base.hpp>
#include <mpt/impl/scenario_goal.hpp>
#include <mpt/impl/scenario_goal_sampler.hpp>
#include <mpt/impl/scenario_link.hpp>
#include <mpt/impl/scenario_rng.hpp>
#include <mpt/impl/scenario_sampler.hpp>
#include <mpt/impl/scenario_space.hpp>
#include <mpt/impl/timer_stat.hpp>
#include <mpt/impl/worker_pool.hpp>
#include <mpt/log.hpp>
#include <mpt/random_device_seed.hpp>
#include <forward_list>
#include <mutex>
#include <utility>

#include "edge.hpp"
#include "node.hpp"
#include "../../needle_utils.h"

namespace unc::robotics::mpt::impl::prrt {
template <bool enable>
struct WorkerStats;

template <>
struct WorkerStats<false> {
    void countIteration() const {}
    void countBiasedSample() const {}
    auto& validMotion() { return TimerStat<void>::instance(); }
    auto& nearest() { return TimerStat<void>::instance(); }
};

template <>
struct WorkerStats<true> {
    mutable std::size_t iterations_{0};
    mutable std::size_t biasedSamples_{0};
    mutable TimerStat<> validMotion_;
    mutable TimerStat<> nearest_;

    void countIteration() const { ++iterations_; }
    void countBiasedSample() const { ++biasedSamples_; }

    TimerStat<>& validMotion() const { return validMotion_; }
    TimerStat<>& nearest() const { return nearest_; }

    WorkerStats& operator += (const WorkerStats& other) {
        iterations_ += other.iterations_;
        biasedSamples_ += other.biasedSamples_;
        validMotion_ += other.validMotion_;
        nearest_ += other.nearest_;
        return *this;
    }

    void print() const {
        MPT_LOG(INFO) << "iterations: " << iterations_;
        MPT_LOG(INFO) << "biased samples: " << biasedSamples_;
        MPT_LOG(INFO) << "valid motion: " << validMotion_;
        MPT_LOG(INFO) << "nearest: " << nearest_;
    }
};

} // unc::robotics::mpt::impl::prrt

#endif // SNP_PRRT_BASE_H