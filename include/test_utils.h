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
#ifndef SNP_TEST_UTILS_H
#define SNP_TEST_UTILS_H

#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <mpt/log.hpp>

#include "global_common.h"
#include "problem_config.h"
#include "needle_utils.h"

namespace unc::robotics::snp::utils {

Str DateAndTime() {
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y%m%d-%H-%M-%S");

    return oss.str();
}

std::tuple<RealNum, RealNum, RealNum, RealNum>
ReadNeedleParameters(Str const& filename, const bool print_info=false) {
    std::ifstream fin;
    fin.open(filename);

    if (!fin.is_open()) {
        throw std::runtime_error("Failed to open " + filename);
    }

    RealNum rad_curv, diameter, length, ang_constraint;

    Str line;

    if (std::getline(fin, line)) {
        std::istringstream s(line);
        s >> rad_curv
          >> diameter
          >> length
          >> ang_constraint;
    }

    fin.close();

    if (print_info) {
        std::cout << "Needle parameters:\n"
                  << "\tradius of curvature " << rad_curv << "\n"
                  << "\tdiameter " << diameter << "\n"
                  << "\tinsertion length " << length  << "\n"
                  << "\tangle constraint (degree) " << ang_constraint
                  << std::endl;
    }

    return {rad_curv, diameter, length, ang_constraint};
}

std::tuple<Vec3, Quat, Vec3, Quat>
ReadStartAndGoal(Str const& filename) {
    std::ifstream fin;
    fin.open(filename);

    if (!fin.is_open()) {
        throw std::runtime_error("Failed to open " + filename);
    }

    Vec3 start_p, goal_p;
    Quat start_q, goal_q;

    Str line;

    if (std::getline(fin, line)) {
        std::istringstream s(line);

        s >> start_p[0] >> start_p[1] >> start_p[2]
          >> start_q.w() >> start_q.x() >> start_q.y() >> start_q.z();
    }

    if (std::getline(fin, line)) {
        std::istringstream s(line);

        s >> goal_p[0] >> goal_p[1] >> goal_p[2]
          >> goal_q.w() >> goal_q.x() >> goal_q.y() >> goal_q.z();
    }

    fin.close();

    return {start_p, start_q.normalized(), goal_p, goal_q.normalized()};
}

std::pair<Vec3, Quat>
ReadStart(Str const& filename) {
    std::ifstream fin;
    fin.open(filename);

    if (!fin.is_open()) {
        throw std::runtime_error("Failed to open " + filename);
    }

    Vec3 start_p;
    Quat start_q;

    Str line;

    if (std::getline(fin, line)) {
        std::istringstream s(line);

        s >> start_p[0] >> start_p[1] >> start_p[2]
          >> start_q.w() >> start_q.x() >> start_q.y() >> start_q.z();
    }

    fin.close();

    return {start_p, start_q.normalized()};
}

std::tuple<Vec3, Quat>
ReadGoal(Str const& filename) {
    std::ifstream fin;
    fin.open(filename);

    if (!fin.is_open()) {
        throw std::runtime_error("Failed to open " + filename);
    }

    Vec3 start_p, goal_p;
    Quat start_q, goal_q;

    Str line;

    if (std::getline(fin, line)) {
        std::istringstream s(line);

        s >> start_p[0] >> start_p[1] >> start_p[2]
          >> start_q.w() >> start_q.x() >> start_q.y() >> start_q.z();
    }

    if (std::getline(fin, line)) {
        std::istringstream s(line);

        s >> goal_p[0] >> goal_p[1] >> goal_p[2]
          >> goal_q.w() >> goal_q.x() >> goal_q.y() >> goal_q.z();
    }

    fin.close();

    return {goal_p, goal_q.normalized()};
}

template<unsigned Mode=0, typename Planner>
void Run(Planner& planner, ConfigPtr cfg, const bool save_only_best_plan=true, const bool save_cost=false)
{
    using State = typename Planner::State;
    static_assert(Mode < 8, "Mode not defined!");

    std::vector<State> path;
    RealNum cost;
    std::vector<std::vector<State>> paths;
    std::vector<RealNum> costs;

    const TimePoint start_time = Clock::now();

    if constexpr (Mode == 0) {
        // Always runs for the specified time, collect all possible results.
        planner.solveFor(std::chrono::milliseconds(cfg->timeout));
    }
    else if constexpr (Mode == 1) {
        // Terminates when reaches specified tree size.
        planner.solve([&, count = (std::size_t)cfg->num_nodes] () {
            return planner.size() >= count;
        });
    }
    else if constexpr (Mode == 2) {
        // Runs until the first solution is found.
        planner.solve([&] () {
            return planner.solved();
        });
    }
    else if constexpr (Mode == 3) {
        // Runs until the first solution is found or reaches specified tree size.
        planner.solve([&, count = (std::size_t)cfg->num_nodes] () {
            return (planner.solved() || planner.size() >= count);
        });
    }
    else if constexpr (Mode == 4) {
        // Runs until the first solution is found or reaches allowed time.
        planner.solveFor([&] { return planner.solved(); }, std::chrono::milliseconds(cfg->timeout));
    }
    else if constexpr (Mode == 5) {
        // Runs until the first N solutions are found or reaches allowed time.
        planner.solveFor([&] { return planner.numPlansFound() >= cfg->num_plans_needed; },
                         std::chrono::milliseconds(cfg->timeout));
    }
    else if constexpr (Mode == 6) {
        planner.solve([&] () {
            // Runs until the planner is exhausted.
            return planner.exhausted();
        });
    }
    else if constexpr (Mode == 7) {
        // Runs until the planner is exhausted or reaches allowed time.
        planner.solveFor([&] { return planner.exhausted(); }, std::chrono::milliseconds(cfg->timeout));
    }

    Clock::duration elapsed = Clock::now() - start_time;

    MPT_LOG(INFO) << "planner generated " << planner.size() << " states";
    MPT_LOG(INFO) << "solve time " << TimeDuration(elapsed) << " seconds";

    planner.printStats();

    if (planner.solved() || planner.approxSolved()) {
        if (planner.solved()) {
            MPT_LOG(INFO) << "exact solution";
        }
        else {
            MPT_LOG(INFO) << "approximate solution";
        }

        path = planner.solution();
        cost = planner.cost();

        if (!save_only_best_plan) {
            paths = planner.allSolutions();
            costs = planner.allCosts();
        }
    }
    else {
        MPT_LOG(INFO) << "no solution found";
    }

    // Save results.
    std::ofstream file(cfg->output_file_root + "_ptcloud.txt");
    planner.visitGraph(Visitor<State>(file));
    std::cout << planner.size() << " vertices written to "
              << cfg->output_file_root + "_ptcloud.txt"
              << std::endl;

    if (save_only_best_plan) {
        if (!path.empty()) {
            WritePathToFile(path, cfg->output_file_root + "_org.txt");
            auto const& interpolated = InterpolatePath(path, cfg->rad_curv, cfg->result_res);
            WritePathToFile(interpolated, cfg->output_file_root + "_interp.txt");

            if (save_cost) {
                std::ofstream fout;
                fout.open(cfg->output_file_root + "_cost.txt");

                if (!fout.is_open()) {
                    throw std::runtime_error("Failed to open " + cfg->output_file_root + "_cost.txt");
                }

                for (auto const& p : interpolated) {
                    auto cost = cfg->env->PointCost(p.translation());
                    fout << cost << std::endl;
                }
                fout.close();
            }

            std::cout << "Result path written to "
                      << cfg->output_file_root + "_org.txt and "
                      << cfg->output_file_root + "_interp.txt"
                      << std::endl;
        }
    }
    else {
        std::cout << "Retrieved " << paths.size() << " result paths." << std::endl;

        SizeType i = 0;
        for (auto const& path : paths) {
            if (!path.empty()) {
                WritePathToFile(path, cfg->output_file_root + "_org_" + std::to_string(i) + ".txt");
                auto const& interpolated = InterpolatePath(path, cfg->rad_curv, cfg->result_res);
                WritePathToFile(interpolated, cfg->output_file_root + "_interp_" + std::to_string(i) + ".txt");

                if (save_cost) {
                    std::ofstream fout;
                    fout.open(cfg->output_file_root + "_cost_" + std::to_string(i) + ".txt");

                    if (!fout.is_open()) {
                        throw std::runtime_error("Failed to open " + cfg->output_file_root + "_cost_" + std::to_string(i) + ".txt");
                    }

                    for (auto const& p : interpolated) {
                        auto cost = cfg->env->PointCost(p.translation());
                        fout << cost << std::endl;
                    }
                    fout.close();
                }
            }

            ++i;
        }

        std::cout << i << " paths written to "
                  << cfg->output_file_root + "_org_(i).txt and "
                  << cfg->output_file_root + "_interp_(i).txt"
                  << std::endl;
    }
}

} // namespace unc::robotics::snp

#endif // SNP_TEST_UTILS_H