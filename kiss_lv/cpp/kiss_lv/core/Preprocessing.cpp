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
#include "Preprocessing.hpp"
#include <tbb/parallel_for.h>
#include <tbb/spin_mutex.h>
#include <tsl/robin_map.h>
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <sophus/se3.hpp>
#include <vector>

namespace {
using Vector3i = Eigen::Matrix<int, 3, 1>;
using Voxel = Vector3i;
struct VoxelHash {
    size_t operator()(const Voxel &voxel) const {
        const uint32_t *vec = reinterpret_cast<const uint32_t *>(voxel.data());
        return ((1 << 20) - 1) & (vec[0] * 73856093 ^ vec[1] * 19349663 ^ vec[2] * 83492791);
    }
};
}  // namespace

namespace kiss_lv {
Vector7dVector Color_VoxelDownsample(const Vector7dVector &frame,
                                             double voxel_size) {
    tsl::robin_map<Voxel, Vector7d, VoxelHash> grid;
    grid.reserve(frame.size());
    for (const auto &point : frame) {
		double voxel_size_c = voxel_size;
		if (point[3] == 255 && point[5]== 255)
			voxel_size_c = voxel_size*0.3;
	Voxel voxel;
	int voxel_x = static_cast<int>(point[0] / (voxel_size_c));
	int voxel_y = static_cast<int>(point[1] / (voxel_size_c));
	int voxel_z = static_cast<int>(point[2] / (voxel_size_c));
	voxel << voxel_x, voxel_y, voxel_z;
	if (grid.contains(voxel)) continue;
		else{grid.insert({voxel, point});}
    }
    Vector7dVector frame_dowsampled;
    frame_dowsampled.reserve(grid.size());
    for (const auto &[voxel, point] : grid) {
        (void)voxel;
        frame_dowsampled.emplace_back(point);
    }
    return frame_dowsampled;
}
Vector7dVectorTuple PL_VoxelDownsample(const Vector7dVector &frame,
                                       double voxel_size) {
    tsl::robin_map<Voxel, Vector7d, VoxelHash> grid; // grid used to store downsampled point cloud data
    grid.reserve(frame.size());
    tsl::robin_map<Voxel, Vector7d, VoxelHash> gridn; // grid used to store downsampled point cloud data
    gridn.reserve(frame.size());
    double voxel_size_c;
    for (const auto &point : frame) {
        if (point[3] == 255 || point[5] == 255) {
            voxel_size_c = voxel_size*0.3;
            Voxel voxel;
            int voxel_x = static_cast<int>(point[0] / voxel_size_c);
            int voxel_y = static_cast<int>(point[1] / voxel_size_c);
            int voxel_z = static_cast<int>(point[2] / voxel_size_c);
            voxel << voxel_x, voxel_y, voxel_z;
            if (grid.contains(voxel)) continue;
            else { grid.insert({ voxel, point }); }
        }
    	voxel_size_c = voxel_size;
        Voxel voxel;
        int voxel_x = static_cast<int>(point[0] / voxel_size_c);
        int voxel_y = static_cast<int>(point[1] / voxel_size_c);
        int voxel_z = static_cast<int>(point[2] / voxel_size_c);
        voxel << voxel_x, voxel_y, voxel_z;
        if (gridn.contains(voxel)) continue;
        else { gridn.insert({ voxel, point }); }
    }
    Vector7dVector pl_dowsampled;
    pl_dowsampled.reserve(grid.size());
    for (const auto &[voxel, point] : grid) {
        (void)voxel;
        pl_dowsampled.emplace_back(point);
    }
    Vector7dVector frame_dowsampled;
    frame_dowsampled.reserve(gridn.size());
    for (const auto &[voxel, point] : gridn) {
        (void)voxel;
        frame_dowsampled.emplace_back(point);
    }
    return std::make_tuple(pl_dowsampled, frame_dowsampled);
}
}  // namespace kiss_lv
