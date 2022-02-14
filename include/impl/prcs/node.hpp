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
#ifndef MPT_IMPL_PRCS_NODE_HPP
#define MPT_IMPL_PRCS_NODE_HPP

#include "edge.hpp"
#include <utility>
#include <array>
#include <set>
#include <mutex>
#include <iostream>
#include <unordered_set>

namespace unc::robotics::mpt::impl::prcs {
using ResLevels = std::array<unsigned, 2>;
using NodeIndices = std::array<unsigned, 3>;

template <typename Vector>
Str IdxToStr(const Vector& idx) {
    return std::to_string(idx[0]) + ","
           + std::to_string(idx[1]) + ","
           + std::to_string(idx[2]);
}

using IndexSet3 = std::unordered_set<Str>;

template <typename State, typename Traj>
class Node {
    using Scalar = typename State::Distance;

    State state_;
    Edge<State, Traj> parent_;
    Scalar traj_length_{0};
    Scalar cost_to_come_{0};
    Scalar cost_to_go_{0};
    bool valid_{false};

    unsigned rank_{0};
    ResLevels levels_{ {0,0} };
    NodeIndices indices_{ {0,0,0} };

    IndexSet3 explored_;
    std::mutex mutex_;

  public:
    template <typename ... Args>
    Node(Traj&& traj, Node* parent, Args&& ... args)
        : state_(std::forward<Args>(args)...)
        , parent_(std::move(traj), parent) {
    }

    template <typename ... Args>
    void reset(Traj&& traj, Node* parent, Args&& ... args) {
        state_ = State(std::forward<Args>(args)...);
        parent_ = Edge<State, Traj>(std::move(traj), parent);
        traj_length_ = 0;
        cost_to_come_ = 0;
        cost_to_go_ = 0;
        valid_ = false;
        rank_ = 0;
        levels_ = {0,0};
        indices_ = {0,0,0};
        explored_.clear();
    }

    void setResolution(const ResLevels& levels, const NodeIndices& indices) {
        rank_ = (((Node*)parent_) == nullptr ? 0 : ((Node*)parent_)->rank()) + levels[0] + levels[1] + 1;
        levels_ = levels;
        indices_ = indices;
    }

    Scalar& length() {
        return traj_length_;
    }

    const Scalar& length() const {
        return traj_length_;
    }

    Scalar& cost() {
        return cost_to_come_;
    }

    const Scalar& cost() const {
        return cost_to_come_;
    }

    Scalar& costToGo() {
        return cost_to_go_;
    }

    const Scalar& costToGo() const {
        return cost_to_go_;
    }

    Scalar f() const {
        return cost_to_come_ + cost_to_go_;
    }

    bool& valid() {
        return valid_;
    }

    const bool& valid() const {
        return valid_;
    }

    State& state() {
        return state_;
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

    Node* parent() {
        return parent_;
    }

    const unsigned& lengthLevel() const {
        return levels_[0];
    }

    const unsigned& angleLevel() const {
        return levels_[1];
    }

    const unsigned& rank() const {
        return rank_;
    }

    const unsigned& radIndex() const {
        return indices_[0];
    }

    const unsigned& lengthIndex() const {
        return indices_[1];
    }

    const unsigned& angleIndex() const {
        return indices_[2];
    }

    const ResLevels& levels() const {
        return levels_;
    }

    const NodeIndices& indices() const {
        return indices_;
    }

    bool explored(const NodeIndices& indices) {
        std::lock_guard<std::mutex> lock(mutex_);

        if (explored_.find(IdxToStr(indices)) != explored_.end()) {
            return true;
        }

        explored_.insert(IdxToStr(indices));
        return false;
    }

    void print(std::ostream& out=std::cout) const {
        out << "length level: " << levels_[0] << ", "
            << "angle level: " << levels_[1] << ", "
            << "radius index: " << indices_[0] << ", "
            << "length index: " << indices_[1] << ", "
            << "angle index: " << indices_[2] << ", "
            << "node rank: " << rank_ << ", "
            << "if parent exists: " << (this->parent() != nullptr) << ".\n ";

        if (this->parent()) {
            out << "Start from: ";
            this->printState(this->parent()->state(), out);
        }

        out << "End at: ";
        this->printState(this->state(), out);
    }

    void printState(const State& state, std::ostream& out=std::cout) const {
        auto const& p = state.translation();
        auto const& q = state.rotation();

        out << "[" << p[0] << "," << p[1] << "," << p[2] << ","
            << q.w() << "," << q.x() << "," << q.y() << "," << q.z() << "]"
            << std::endl;
    }
};

struct NodeKey {
    template <typename State, typename Traj>
    const State& operator() (const Node<State, Traj>* node) const {
        return node->state();
    }
};

template <typename State>
struct StateNode {
    State state;

    StateNode(const State& s) {
        state = s;
        Eigen::Vector3d tang = (s.rotation().normalized()*Eigen::Vector3d::UnitZ()).normalized();
        state.rotation() = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), tang).normalized();
    }
};

struct StateNodeKey {
    template <typename State>
    const State& operator() (const StateNode<State>& n) const {
        return n.state;
    }
};

} // unc::robotics::mpt::impl::prcs

#endif // MPT_IMPL_PRCS_NODE_HPP