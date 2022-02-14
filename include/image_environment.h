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
#ifndef SNP_IMAGE_ENVIRONMENG_H
#define SNP_IMAGE_ENVIRONMENG_H

#include <nigh/lp_space.hpp>
#include <nigh/kdtree_batch.hpp>

#include "global_common.h"

namespace unc::robotics::snp {

struct ObstaclePoint {
    SizeType idx;
    Vec3 point;
    bool valid;
    ObstaclePoint(const SizeType i, const Vec3& p) : idx(i), point(p) {
        valid = true;
    }
};

struct PointKey {
    const Vec3& operator() (const ObstaclePoint& node) const {
        return node.point;
    }
};

using PointPair = std::pair<ObstaclePoint, RealNum>;
using ImgNN =
    nigh::Nigh<ObstaclePoint, nigh::L2Space<RealNum, 3>, PointKey, nigh::Concurrent, nigh::KDTreeBatch<>>;
using ImgNNWithMask = std::pair<bool, ImgNN*>;

class ImageEnvironment {
  public:
    ImageEnvironment(const Affine& ijk_to_ras=Affine::Identity());
    ~ImageEnvironment();

    void SetIjkToRasAffine(const Affine& ijk_to_ras);
    Affine IjkToRasAffine() const;

    void SetImageSize(const IdxPoint& size);
    void SetImageSize(const Idx size_x, const Idx size_y, const Idx size_z);
    IdxPoint ImageSize() const;

    bool CreateNewNN(const Str image_name="default");
    bool RemoveNN(const Str image_name);
    bool EnableNN(const Str image_name);
    bool DisableNN(const Str image_name);
    std::vector<Str> ListAllNN() const;
    std::vector<Str> ListActiveNN() const;

    void GenerateEmptyImage(const Idx size_x, const Idx size_y, const Idx size_z);
    bool ConstructEnvironmentFromFile(const Str file_name);
    bool ConstructCostFromFile(const Str file_name);

    Vec3 IjkToRas(const IdxPoint& p) const;
    Vec3 IjkToRas(const Idx& i, const Idx& j, const Idx& k) const;

    IdxPoint RasToIjk(const Vec3& p) const;
    IdxPoint RasToIjk(const RealNum& r, const RealNum& a, const RealNum& s) const;

    void AddObstacle(const Vec3& p, const Str& image_name="default");
    void AddObstacle(const IdxPoint& p, const Str& image_name="default");

    void AddCost(const IdxPoint& p, const RealNum& cost);

    bool IsObstacle(const IdxPoint& p) const;
    bool IsObstacle(const Vec3& p) const;
    bool IsObstacleCenter(const Vec3& p) const;

    IdxPoint NearestObstacle(const IdxPoint& p) const;
    IdxPoint NearestObstacle(const Vec3& p) const;

    std::pair<Vec3, RealNum> NearestObstacleCenter(const Vec3& p) const;
    std::pair<Vec3, RealNum> NearestObstacleCenter(const Vec3& p, const Str& image_name) const;

    RealNum DistanceToObstacleCenter(const Vec3& p) const;
    RealNum DistanceToObstacleCenter(const Vec3& p, const Str& image_name) const;

    bool WithinImage(const Vec3& p) const;
    bool WithinImage(const IdxPoint& p) const;

    RealNum VoxelRadius() const;

    void ClearWhiteList();
    void SetWhiteList(bool flag);
    void AddToWhiteList(const Vec3& p, const RealNum r);
    bool InWhiteListArea(const Vec3& p) const;


    RealNum CostInCostMap(const Vec3& p) const;

    enum CostType {
        NO_COST=0,
        PATH_LENGTH,
        COST_MAP,
        DIST_TO_OBS,
        GOAL_ORIENTATION
    };

    RealNum PointCost(const Vec3& p) const;
    RealNum PointCost(const Vec3& p, const CostType cost_type) const;
    RealNum TrilinearInterpolatedCostFromMap(const Vec3& p) const;

    void SetCostType(const CostType type);
    Str CostTypeString();
    CostType ActiveCostType() const;
    RealNum CurveCost(const Vec3& sp, const Quat& sq, const Vec3& gp, const Quat& gq,
                      const RealNum rad, const RealNum resolution) const;
    RealNum CurveCost(const Vec3& sp, const Quat& sq, const Vec3& gp, const Quat& gq,
                      const RealNum rad, const RealNum resolution, const CostType cost_type) const;
    RealNum FinalStateCost(const Vec3& p, const Quat& q, const Vec3& gp, const Quat& gq) const;


    void SetMinDist(const RealNum dist);
    RealNum MinDist() const;

    bool CollisionFree(const Vec3& p) const;
    bool CollisionFree(const Vec3& p, const Str& image_name) const;

    bool ObstaclesEnabled() const;
    void EnableObstacles();
    void DisableObstacles();

    bool HitDetectionsEnabled() const;
    void EnableHitDetection();
    void DisableHitDetection();

    void SaveRasPtc(const Str& file_name) const;

    void EnableTrilinearInterpolation(const bool enable);
    RealNum MinCost() const;
    RealNum CostK() const;

    void SetWorkspace(const bool value=true);
    void SetWorkspace(const Idx& x, const Idx& y, const Idx& z, const bool value=true);
    bool Workspace(const Idx& x, const Idx& y, const Idx& z) const;

  private:
    Affine ijk_to_ras_{Affine::Identity()};
    Affine ras_to_ijk_{Affine::Identity()};
    RealNum voxel_rad_{0};
    IdxPoint image_size_{IdxPoint(0, 0, 0)};
    std::map<Str, SizeType> obstacle_idx_;
    RealNum nn_search_rad_{50};

    CostType cost_type_{NO_COST};

    RealNum min_dist_to_obs_{0};

    RealArray3 cost_array_;
    BoolArray3 workspace_;

    bool use_white_list_{false};
    std::vector<std::pair<Vec3, RealNum>> white_list_;

    bool use_obstacles_{true};
    bool use_hit_detection_{false};

    RealNum min_cost_{0.01};
    RealNum delta_{0.1};
    RealNum cost_k_{1.0};
    RealNum out_of_image_cost_{1000.0};
    bool use_trilinear_interpolation_{false};

    std::map<Str, ImgNNWithMask> all_nns_;
};

using EnvPtr = std::shared_ptr<ImageEnvironment>;

} // namespace unc::robotics::snp

#include "impl/image_environment.hpp"

#endif // SNP_IMAGE_ENVIRONMENG_H