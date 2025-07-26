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
#pragma once
#include <Eigen/Core>
#include <tuple>
#include <vector>
#include "kiss_lv/core/Threshold.hpp"
#include "kiss_lv/core/VoxelHashMap_Color.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
namespace kiss_lv::pipeline {
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector6dVector = std::vector<Vector6d>;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Vector7dVector = std::vector<Vector7d>;
struct KISSConfig {
    // map params
    double voxel_size = 1;
    double max_range = 100.0;
    double min_range = 5.0;
    int max_points_per_voxel = 10;
    std::string type = "LIVOX";

    // th parms
    double min_motion_th = 0.1;
    double initial_threshold = 2.0;
    // Motion compensation
    bool deskew = false;
};
class KissLV {
public:
	typedef pcl::PointXYZI PointTypeI;
	typedef pcl::PointCloud<PointTypeI> PointCloudXYZI;
	typedef pcl::PointXYZRGB ColorPointType;
	typedef pcl::PointCloud<ColorPointType>  PointCloudXYZRGB;
	double scan_num = 1;
	bool odomtry_error = false;
public:
    explicit KissLV(const KISSConfig &config)
        : config_(config),
          color_local_map_(config.voxel_size, config.max_range),
          adaptive_threshold_(config.initial_threshold, config.min_motion_th, config.max_range){}

    KissLV() : KissLV(KISSConfig{}) {}

public:
    Vector7dVector RegisterFrame(const Vector6dVector color_frame, double adj_voxel_size, double density);
    double GetAdaptiveThreshold(Sophus::SE3d &prediction, double th_size);
    bool HasMoved(Sophus::SE3d &prediction);
public:
    std::vector<Vector7d> LocalMap() const { return color_local_map_.Pointcloud(); }; 
    std::vector<Sophus::SE3d> poses() const { return poses_; };  

private:
    std::vector<Sophus::SE3d> poses_;  
    KISSConfig config_;    
    Color_VoxelHashMap color_local_map_; 
    AdaptiveThreshold adaptive_threshold_; 
};

}  // namespace kiss_lv::pipeline
