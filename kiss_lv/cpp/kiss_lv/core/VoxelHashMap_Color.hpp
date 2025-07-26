// MIT License
//
// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
// Stachniss.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// NOTE: This implementation is heavily inspired in the original CT-ICP VoxelHashMap implementation,
// although it was heavily modifed and drastically simplified, but if you are using this module you
// should at least acknoowledge the work from CT-ICP by giving a star on GitHub
#pragma once

#include <tsl/robin_map.h>
#include <iostream>
#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <vector>
using Vector7d = Eigen::Matrix<double, 7, 1>;
namespace kiss_lv {

struct Color_VoxelHashMap {
	using Vector7dVector = std::vector<Vector7d>;
	using Vector7dVectorTuple = std::tuple<Vector7dVector, Vector7dVector>;
	using Vector4i = Eigen::Matrix<int, 4, 1>;
	using Voxel = Vector4i;
    
    struct Color_VoxelBlock {
        std::vector<Vector7d> points;
        inline void AddPoint(const Vector7d &point, int num_points_) {
            if (points.size() < static_cast<size_t>(num_points_)) points.push_back(point);
        }
    };

	struct Color_VoxelHash {
        size_t operator()(const Voxel &voxel) const {
            const uint32_t *vec = reinterpret_cast<const uint32_t *>(voxel.data());
            return voxel[3]*((1 << 20) - 1) & (vec[0] * 73856093 ^ vec[1] * 19349663 ^ vec[2] * 83492791);
        }
	};

    explicit Color_VoxelHashMap(double voxel_size, double max_distance)
        : voxel_size_(voxel_size),
          max_distance_(max_distance){}

    Vector7dVectorTuple Color_GetCorrespondences(const Vector7dVector &points, double max_correspondance_distance)const;   
    inline bool Color_Empty() const { return colormap_.empty(); }
    void Color_Update(const std::vector<Vector7d> &points, const Eigen::Vector3d &origin);
    void Color_Update(const std::vector<Vector7d> &points, const Sophus::SE3d &pose);
    void Color_AddPoints(const std::vector<Vector7d> &points);
    void Color_RemovePointsFarFromLocation(const Eigen::Vector3d &origin);
    void Get_adj_voxel_size(const double density){
    	max_points_per_voxel_ = std::clamp(static_cast<int>(density), 3, 100);
		num_range_1 = 1;
		range_1 = 27;
		scan_num_++;
		std::cout<<"max_points_per_voxel_"<<max_points_per_voxel_<<std::endl;
    }
    void Clean_color();
    std::vector<Vector7d> Pointcloud() const;
    double voxel_size_;
    double max_distance_;
    int num_range_1, range_1;
    int max_points_per_voxel_;
    double scan_num_;
    mutable int range;
    tsl::robin_map<Voxel, Color_VoxelBlock, Color_VoxelHash> colormap_;
};
}  // namespace kiss_lv
