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
#ifndef MPT_IMPL_PRRT_NODE_HPP
#define MPT_IMPL_PRRT_NODE_HPP

#include "edge.hpp"
#include <utility>

namespace unc::robotics::mpt::impl::prrt {
template <typename State, typename Traj>
class Node {
    using Scalar = typename State::Distance;

    State state_;
    Edge<State, Traj> parent_;
    Scalar traj_length_{0};
    Scalar cost_{0};

  public:
    template <typename ... Args>
    Node(Traj&& traj, Node *parent, Args&& ... args)
        : state_(std::forward<Args>(args)...)
        , parent_(std::move(traj), parent)
    {
    }

    Scalar& length() {
        return traj_length_;
    }

    const Scalar& length() const {
        return traj_length_;
    }

    Scalar& cost() {
        return cost_;
    }

    const Scalar& cost() const {
        return cost_;
    }

    const State& state() const {
        return state_;
    }

    const Edge<State, Traj>& edge() const {
        return parent_;
    }

    const Node* parent() const {
        return parent_;
    }
};

struct NodeKey {
    template <typename State, typename Traj>
    const State& operator() (const Node<State, Traj>* node) const {
        return node->state();
    }
};

} // unc::robotics::mpt::impl::prrt

#endif // MPT_IMPL_PRRT_NODE_HPP