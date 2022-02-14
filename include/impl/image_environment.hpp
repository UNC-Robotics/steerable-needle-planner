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

#include <iostream>
#include <fstream>

#include "needle_utils.h"

namespace unc::robotics::snp {

ImageEnvironment::ImageEnvironment(const Affine& ijk_to_ras) {
    this->SetIjkToRasAffine(ijk_to_ras);
}

ImageEnvironment::~ImageEnvironment() {
    cost_array_.resize(boost::extents[0][0][0]);

    for (auto const& [name, nn] : all_nns_) {
        if (nn.second) {
            delete nn.second;
        }
    }
}

void ImageEnvironment::SetIjkToRasAffine(const Affine& ijk_to_ras) {
    ijk_to_ras_ = ijk_to_ras;
    ras_to_ijk_ = ijk_to_ras.inverse();
    voxel_rad_ = 0.5*Vec3(ijk_to_ras_(0, 0), ijk_to_ras_(1, 1), ijk_to_ras_(2, 2)).norm();
}

Affine ImageEnvironment::IjkToRasAffine() const {
    return ijk_to_ras_;
}

void ImageEnvironment::SetImageSize(const IdxPoint& size) {
    this->SetImageSize(size[0], size[1], size[2]);
}

void ImageEnvironment::SetImageSize(const Idx size_x, const Idx size_y, const Idx size_z) {
    image_size_[0] = size_x;
    image_size_[1] = size_y;
    image_size_[2] = size_z;

    cost_array_.resize(boost::extents[size_x][size_y][size_z]);
    workspace_.resize(boost::extents[size_x][size_y][size_z]);
}

IdxPoint ImageEnvironment::ImageSize() const {
    return image_size_;
}

bool ImageEnvironment::CreateNewNN(const Str image_name) {
    if (all_nns_.find(image_name) != all_nns_.end()) {
        std::cerr << "Image with name " << image_name << " already exists!" << std::endl;
        return false;
    }

    ImgNNWithMask tmp;
    tmp.first = true;
    tmp.second = new ImgNN();
    all_nns_.insert(std::pair<Str, ImgNNWithMask>(image_name, tmp));

    obstacle_idx_[image_name] = 0;
    return true;
}

bool ImageEnvironment::RemoveNN(const Str image_name) {
    auto search = all_nns_.find(image_name);

    if (search == all_nns_.end()) {
        std::cerr << "Image with name " << image_name << " does not exist!" << std::endl;
        return false;
    }

    delete search->second.second;
    all_nns_.erase(image_name);
    obstacle_idx_.erase(image_name);
    return true;
}

bool ImageEnvironment::EnableNN(const Str image_name) {
    auto search = all_nns_.find(image_name);

    if (search == all_nns_.end()) {
        std::cerr << "Image with name " << image_name << " does not exist!" << std::endl;
        return false;
    }

    search->second.first = true;
    return true;
}

bool ImageEnvironment::DisableNN(const Str image_name) {
    auto search = all_nns_.find(image_name);

    if (search == all_nns_.end()) {
        std::cerr << "Image with name " << image_name << " does not exist!" << std::endl;
        return false;
    }

    search->second.first = false;
    return true;
}

std::vector<Str> ImageEnvironment::ListAllNN() const {
    std::vector<Str> names;

    for (auto const& [name, nn] : all_nns_) {
        names.push_back(name);
    }

    return names;
}

std::vector<Str> ImageEnvironment::ListActiveNN() const {
    std::vector<Str> names;

    for (auto const& [name, nn] : all_nns_) {
        if (nn.first) {
            names.push_back(name);
        }
    }

    return names;
}

void ImageEnvironment::GenerateEmptyImage(const Idx size_x, const Idx size_y, const Idx size_z) {
    this->SetImageSize(size_x, size_y, size_z);
}

bool ImageEnvironment::ConstructEnvironmentFromFile(const Str file_name) {
    std::ifstream fin;
    fin.open(file_name);

    if (!fin.is_open()) {
        throw std::runtime_error("Failed to open " + file_name);
    }

    Str line;
    Affine ijk_to_ras;

    for (auto i = 0; i < 4; ++i) {
        if (std::getline(fin, line)) {
            std::istringstream s(line);

            for (auto j = 0; j < 4; ++j) {
                s >> ijk_to_ras(i, j);
            }
        }
        else {
            std::cout << "Wrong format for ras_to_ijk affine!" << std::endl;
            return false;
        }
    }

    this->SetIjkToRasAffine(ijk_to_ras);

    IdxPoint size;

    if (std::getline(fin, line)) {
        std::istringstream s(line);

        for (auto i = 0; i < 3; ++i) {
            s >> size[i];
        }
    }
    else {
        std::cout << "Wrong format for image size!" << std::endl;
        return false;
    }

    this->SetImageSize(size);

    this->CreateNewNN("defualt");
    IdxPoint obs;

    while (std::getline(fin, line)) {
        std::istringstream s(line);

        for (auto i = 0; i < 3; ++i) {
            s >> obs[i];
        }

        this->AddObstacle(obs, "defualt");
        std::cout << "\rLoaded obstacle points: " << obstacle_idx_["defualt"] << std::flush;
    }

    std::cout << std::endl;
    return true;
}

bool ImageEnvironment::ConstructCostFromFile(const Str file_name) {
    std::ifstream fin;
    fin.open(file_name);

    if (!fin.is_open()) {
        throw std::runtime_error("Failed to open " + file_name);
    }

    Str line;
    IdxPoint obs;
    RealNum cost;
    SizeType iter = 0;

    while (std::getline(fin, line)) {
        std::istringstream s(line);

        for (auto i = 0; i < 3; ++i) {
            s >> obs[i];
        }

        s >> cost;

        this->AddCost(obs, cost);
        std::cout << "\rLoaded cost: " << ++iter << std::flush;
    }

    std::cout << std::endl;
    return true;
}

Vec3 ImageEnvironment::IjkToRas(const IdxPoint& p) const {
    return ijk_to_ras_*Vec3(p[0], p[1], p[2]);
}

Vec3 ImageEnvironment::IjkToRas(const Idx& i, const Idx& j, const Idx& k) const {
    return ijk_to_ras_*Vec3(i, j, k);
}

IdxPoint ImageEnvironment::RasToIjk(const Vec3& p) const {
    Vec3 rough = ras_to_ijk_*p;

    if (rough[0] < 0 || rough[1] < 0 || rough[2] < 0) {
        std::cerr << "Negative index! Point: "
                  << p.transpose()
                  << " to "
                  << rough.transpose()
                  << std::endl;
        getchar();
    }

    return IdxPoint(round(rough[0]), round(rough[1]), round(rough[2]));
}

IdxPoint ImageEnvironment::RasToIjk(const RealNum& r, const RealNum& a, const RealNum& s) const {
    return this->RasToIjk(Vec3(r, a, s));
}

void ImageEnvironment::AddObstacle(const Vec3& p, const Str& image_name) {
    auto search = all_nns_.find(image_name);

    if (search == all_nns_.end()) {
        throw std::runtime_error("Image with name " + image_name + " does not exist! Cannot add obstacle!");
    }

    ImgNN* nn = search->second.second;
    auto nearest = nn->nearest(p);

    if (!nearest || (nearest->first.point - p).norm() > EPS) {
        nn->insert(ObstaclePoint(obstacle_idx_[image_name]++, p));
    }
}

void ImageEnvironment::AddObstacle(const IdxPoint& p, const Str& image_name) {
    if (!this->WithinImage(p)) {
        throw std::runtime_error("Trying to add an obstacle outside the image region.");
    }

    auto ras_p = this->IjkToRas(p);
    this->AddObstacle(ras_p, image_name);
}

void ImageEnvironment::AddCost(const IdxPoint& p, const RealNum& cost) {
    if (!this->WithinImage(p)) {
        throw std::runtime_error("Trying to add a cost outside the image region.");
    }

    cost_array_[p[0]][p[1]][p[2]] = cost;
}

bool ImageEnvironment::IsObstacle(const IdxPoint& p) const {
    if (this->WithinImage(p)) {
        return this->IsObstacleCenter(this->IjkToRas(p));
    }

    std::cerr << "Out of image region!" << std::endl;
    return true;
}

bool ImageEnvironment::IsObstacle(const Vec3& p) const {
    const Vec3 center = this->IjkToRas(this->RasToIjk(p));

    return this->IsObstacleCenter(center);
}

bool ImageEnvironment::IsObstacleCenter(const Vec3& p) const {
    return (this->DistanceToObstacleCenter(p) < EPS);
}

IdxPoint ImageEnvironment::NearestObstacle(const IdxPoint& p) const {
    return this->NearestObstacle(this->IjkToRas(p));
}

IdxPoint ImageEnvironment::NearestObstacle(const Vec3& p) const {
    auto [point, min_dist] = this->NearestObstacleCenter(p);

    if(min_dist == R_INF) {
        return IdxPoint(0, 0, 0);
    }

    return this->RasToIjk(point);
}

std::pair<Vec3, RealNum> ImageEnvironment::NearestObstacleCenter(const Vec3& p) const {
    RealNum min_dist = R_INF;
    Vec3 point(0, 0, 0);

    for (auto const& [name, nn] : all_nns_) {
        if (nn.first) {
            auto p_nearest = nn.second->nearest(p);

            if (p_nearest && p_nearest->second < min_dist) {
                min_dist = p_nearest->second;
                point = p_nearest->first.point;
            }
        }
    }

    return {point, min_dist};
}

std::pair<Vec3, RealNum> ImageEnvironment::NearestObstacleCenter(const Vec3& p,
        const Str& image_name) const {
    auto search = all_nns_.find(image_name);

    if (search == all_nns_.end()) {
        throw std::runtime_error("Image with name " + image_name +
                                 "does not exist! Cannot query obstacle center!");
    }

    auto nn = search->second.second;
    auto p_nearest = nn->nearest(p);

    if (!p_nearest) {
        return {Vec3(0, 0, 0), R_INF};
    }

    return {p_nearest->first.point, p_nearest->second};
}

RealNum ImageEnvironment::DistanceToObstacleCenter(const Vec3& p) const {
    auto [point, min_dist] = this->NearestObstacleCenter(p);

    if(min_dist == R_INF) {
        return nn_search_rad_;
    }

    return fmin(min_dist, nn_search_rad_);
}

RealNum ImageEnvironment::DistanceToObstacleCenter(const Vec3& p, const Str& image_name) const {
    auto [point, min_dist] = this->NearestObstacleCenter(p, image_name);

    if(min_dist == R_INF) {
        return nn_search_rad_;
    }

    return fmin(min_dist, nn_search_rad_);
}

bool ImageEnvironment::WithinImage(const Vec3& p) const {
    Vec3 rough = ras_to_ijk_*p;

    if (rough[0] < 0 || rough[1] < 0 || rough[2] < 0) {
        return false;
    }

    if (round(rough[0]) < image_size_[0]
            && round(rough[1]) < image_size_[1]
            && round(rough[2]) < image_size_[2]) {
        return true;
    }

    return false;
}

bool ImageEnvironment::WithinImage(const IdxPoint& p) const {
    if (p[0] < image_size_[0] && p[1] < image_size_[1] && p[2] < image_size_[2]) {
        return true;
    }

    return false;
}

RealNum ImageEnvironment::VoxelRadius() const {
    return voxel_rad_;
}

void ImageEnvironment::ClearWhiteList() {
    auto n = white_list_.size();
    white_list_.clear();

    std::cout << "Removed " << n << " white list components." << std::endl;
}

void ImageEnvironment::SetWhiteList(bool flag) {
    use_white_list_ = flag;
}

bool ImageEnvironment::InWhiteListArea(const Vec3& p) const {
    for (const auto ball : white_list_) {
        if ((p - ball.first).norm() < ball.second) {
            return true;
        }
    }

    return false;
}

void ImageEnvironment::AddToWhiteList(const Vec3& p, const RealNum r) {
    white_list_.emplace_back(p, r);
}

RealNum ImageEnvironment::PointCost(const Vec3& p) const {
    return PointCost(p, cost_type_);
}

RealNum ImageEnvironment::PointCost(const Vec3& p, const CostType cost_type) const {
    if (!this->WithinImage(p)) {
        return out_of_image_cost_;
    }

    if (cost_type == NO_COST || cost_type == GOAL_ORIENTATION) {
        return 0;
    }

    if (cost_type == PATH_LENGTH) {
        return 1;
    }

    if (cost_type == COST_MAP) {
        if (use_trilinear_interpolation_) {
            return std::max(min_cost_, TrilinearInterpolatedCostFromMap(p));
        }

        IdxPoint idx = this->RasToIjk(p);
        return std::max(min_cost_, cost_array_[idx[0]][idx[1]][idx[2]]);
    }

    if (cost_type == DIST_TO_OBS) {
        auto dist = this->DistanceToObstacleCenter(p);
        return 1.0 / std::max(delta_, dist - min_dist_to_obs_);
    }

    return 0;
}

RealNum ImageEnvironment::TrilinearInterpolatedCostFromMap(const Vec3& p) const {
    const IdxPoint nearest = this->RasToIjk(p);
    const Vec3 nearest_ras = this->IjkToRas(nearest);

    int dx, dy, dz;
    dx = (p[0] - nearest_ras[0]) * ijk_to_ras_(0, 0) > 0 ? 1 : -1;
    dy = (p[1] - nearest_ras[1]) * ijk_to_ras_(1, 1) > 0 ? 1 : -1;
    dz = (p[2] - nearest_ras[2]) * ijk_to_ras_(2, 2) > 0 ? 1 : -1;

    IntPoint tmp;
    std::vector<RealNum> neig;
    for (int x = 0; x < 2; ++x) {
        tmp[0] = nearest[0] + x*dx;
        for (int y = 0; y < 2; ++y) {
            tmp[1] = nearest[1] + y*dy;
            for (int z = 0; z < 2; ++z) {
                tmp[2] = nearest[2] + z*dz;

                if (tmp[0] < 0 || tmp[1] < 0 || tmp[2] < 0
                    || tmp[0] >= image_size_[0] || tmp[1] >= image_size_[1] || tmp[2] >= image_size_[2]) 
                {
                    neig.push_back(out_of_image_cost_);
                }
                else {
                    neig.push_back(cost_array_[tmp[0]][tmp[1]][tmp[2]]);
                }
            }
        }
    }

    RealNum x_d = (p[0] - nearest_ras[0])/(dx * ijk_to_ras_(0, 0));
    RealNum y_d = (p[1] - nearest_ras[1])/(dy * ijk_to_ras_(1, 1));
    RealNum z_d = (p[2] - nearest_ras[2])/(dz * ijk_to_ras_(2, 2));

    RealNum c_00 = neig[0] * (1 - dx) + neig[4] * x_d;
    RealNum c_01 = neig[1] * (1 - dx) + neig[5] * x_d;
    RealNum c_10 = neig[2] * (1 - dx) + neig[6] * x_d;
    RealNum c_11 = neig[3] * (1 - dx) + neig[7] * x_d;

    RealNum c_0 = c_00 * (1 - y_d) + c_10 * y_d;
    RealNum c_1 = c_01 * (1 - y_d) + c_11 * y_d;

    return c_0 * (1 - z_d) + c_1 * z_d;
}

RealNum ImageEnvironment::CostInCostMap(const Vec3& p) const {
    if (!this->WithinImage(p)) {
        throw std::runtime_error("Trying to query the cost of a point outside the image.");
    }

    IdxPoint idx = this->RasToIjk(p);
    return cost_array_[idx[0]][idx[1]][idx[2]];
}

void ImageEnvironment::SetCostType(const CostType type) {
    cost_type_ = type;

    switch (type) {
        case NO_COST: {
            min_cost_ = 0.0;
            cost_k_ = 1.0;
            break;
        }

        case PATH_LENGTH: {
            min_cost_ = 1.0;
            cost_k_ = 1.0;
            break;
        }

        case COST_MAP: {
            min_cost_ = 0.01;
            auto min_size = std::min(std::min(ijk_to_ras_(0, 0),
                                              ijk_to_ras_(1, 1)),
                                              ijk_to_ras_(2, 2));
            cost_k_ = (1.0 / min_size + 1.0) / min_cost_;
            break;
        }

        case DIST_TO_OBS: {
            min_cost_ = 0.01;
            cost_k_ = (1.0 / (delta_ * delta_) + 1.0 / delta_) / min_cost_;
            break;
        }

        case GOAL_ORIENTATION: {
            min_cost_ = 0.0;
            cost_k_ = -1;
            break;
        }
    }
}

Str ImageEnvironment::CostTypeString() {
    switch (cost_type_) {
        case NO_COST: {
            return "NO_COST";
        }

        case PATH_LENGTH: {
            return "PATH_LENGTH";
        }

        case COST_MAP: {
            return "IMAGE_COST_MAP";
        }

        case DIST_TO_OBS: {
            return "DISTANCE_TO_OBSTACLES";
        }

        case GOAL_ORIENTATION: {
            return "GOAL_ORIENTATION_DIFFERENCE";
        }
    }

    return "NOT_DEFINED!!!";
}

ImageEnvironment::CostType ImageEnvironment::ActiveCostType() const {
    return cost_type_;
}

RealNum ImageEnvironment::CurveCost(const Vec3& sp, const Quat& sq, const Vec3& gp, const Quat& gq,
                                    const RealNum rad, const RealNum resolution) const {
    return CurveCost(sp, sq, gp, gq, rad, resolution, cost_type_);
}

RealNum ImageEnvironment::CurveCost(const Vec3& sp, const Quat& sq, const Vec3& gp, const Quat& gq,
                                    const RealNum rad, const RealNum resolution, const CostType cost_type) const {
    RealNum curve_cost = 0;

    switch (cost_type) {
        case NO_COST: {
            curve_cost = 0;
            break;
        }

        case PATH_LENGTH: {
            curve_cost = CurveLength(sp, sq, gp, gq);
            break;
        }

        case GOAL_ORIENTATION: {
            curve_cost = 0;
            break;
        }

        default: {
            auto avg_cost = PointCost(sp) + PointCost(gp);
            using Se3State = unc::robotics::mpt::SE3State<RealNum>;
            const auto pairs = Interpolate<Se3State>(sp, sq, gp, gq, rad, resolution);

            for (const auto& pair : pairs) {
                avg_cost += PointCost(pair.translation());
            }

            avg_cost /= pairs.size() + 2;

            curve_cost = avg_cost*CurveLength(sp, sq, gp, gq);
            break;
        }
    }

    if (std::isnan(curve_cost)) {
        throw std::runtime_error("[ImageEnvironment] Get nan curve cost!");
    }

    return curve_cost;
}

RealNum ImageEnvironment::FinalStateCost(const Vec3& p, const Quat& q, const Vec3& gp,
        const Quat& gq) const {
    if (cost_type_ == GOAL_ORIENTATION) {
        return DirectionDifference(q, gq);
    }

    return 0;
}

void ImageEnvironment::SetMinDist(const RealNum dist) {
    min_dist_to_obs_ = dist;
}

RealNum ImageEnvironment::MinDist() const {
    return min_dist_to_obs_;
}

bool ImageEnvironment::CollisionFree(const Vec3& p) const {
    if (!this->WithinImage(p)) {
        return false;
    }

    if (!use_obstacles_) {
        return true;
    }

    if (use_hit_detection_) {
        return !this->IsObstacle(p);
    }

    for (auto const& [name, nn] : all_nns_) {
        if (nn.first) {
            auto p_nearest = nn.second->nearest(p);

            if (p_nearest && p_nearest->second < min_dist_to_obs_) {
                if (!this->InWhiteListArea(p_nearest->first.point)) {
                    return false;
                }

                std::vector<PointPair> nbh;
                auto search = obstacle_idx_.find(name);

                if (search == obstacle_idx_.end()) {
                    throw std::runtime_error("Cannot find " + name + " in obstacle idx map!");
                }

                nn.second->nearest(nbh, p, search->second, min_dist_to_obs_);

                for (auto n : nbh) {
                    if (!this->InWhiteListArea(n.first.point)) {
                        return false;
                    }
                }
            }
        }
    }

    return true;
}

bool ImageEnvironment::CollisionFree(const Vec3& p, const Str& image_name) const {
    if (!this->WithinImage(p)) {
        return false;
    }

    if (!use_obstacles_) {
        return true;
    }

    if (use_hit_detection_) {
        throw std::runtime_error("No implementation for hit detection for a specific image!");
    }

    auto search = all_nns_.find(image_name);

    if (search == all_nns_.end()) {
        throw std::runtime_error("Image with name " + image_name +
                                 "does not exist! Cannot query collision!");
    }

    auto nn = search->second;
    auto p_nearest = nn.second->nearest(p);

    if (p_nearest && p_nearest->second < min_dist_to_obs_) {
        if (!this->InWhiteListArea(p_nearest->first.point)) {
            return false;
        }

        std::vector<PointPair> nbh;
        auto search_idx = obstacle_idx_.find(image_name);

        if (search_idx == obstacle_idx_.end()) {
            throw std::runtime_error("Cannot find " + image_name + " in obstacle idx map!");
        }

        nn.second->nearest(nbh, p, search_idx->second, min_dist_to_obs_);

        for (auto n : nbh) {
            if (!this->InWhiteListArea(n.first.point)) {
                return false;
            }
        }
    }

    return true;
}

bool ImageEnvironment::ObstaclesEnabled() const {
    return use_obstacles_;
}

void ImageEnvironment::EnableObstacles() {
    use_obstacles_ = true;
}

void ImageEnvironment::DisableObstacles() {
    use_obstacles_ = false;
}

bool ImageEnvironment::HitDetectionsEnabled() const {
    return use_hit_detection_;
}

void ImageEnvironment::EnableHitDetection() {
    use_hit_detection_ = true;
}

void ImageEnvironment::DisableHitDetection() {
    use_hit_detection_ = false;
}

void ImageEnvironment::SaveRasPtc(const Str& file_name) const {
    std::ofstream fout;
    fout.open(file_name);

    if (!fout.is_open()) {
        throw std::runtime_error("Failed to open " + file_name);
    }

    std::cout << "Saving ras point cloud..." << std::endl;

    SizeType count = 0;

    for (auto const& [name, nn] : all_nns_) {
        auto ptc = nn.second->list();

        for (const auto& p : ptc) {
            const Vec3& point = p.point;

            fout << point[0] << " " << point[1] << " " << point[2] << std::endl;
        }

        count += ptc.size();
    }

    std::cout << "Saved " << count << " points to " << file_name << std::endl;
}

void ImageEnvironment::EnableTrilinearInterpolation(const bool enable) {
    use_trilinear_interpolation_ = enable;
}

RealNum ImageEnvironment::MinCost() const {
    return min_cost_;
}

RealNum ImageEnvironment::CostK() const {
    return cost_k_;
}

void ImageEnvironment::SetWorkspace(const bool value) {
    std::fill(workspace_.data(), workspace_.data() + workspace_.num_elements(), value);
}

void ImageEnvironment::SetWorkspace(const Idx& x, const Idx& y, const Idx& z, const bool value) {
    workspace_[x][y][z] = value;
}

bool ImageEnvironment::Workspace(const Idx& x, const Idx& y, const Idx& z) const {
    return workspace_[x][y][z];
}

} // namespace unc::robotics::snp