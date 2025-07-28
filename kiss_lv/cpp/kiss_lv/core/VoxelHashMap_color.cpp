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
#include "VoxelHashMap_Color.hpp"

#include <tbb/blocked_range.h>
#include <tbb/parallel_reduce.h>

#include <Eigen/Core>
#include <algorithm>
#include <limits>
#include <tuple>
#include <utility>
#include <vector>
using namespace std;
using Vector7d = Eigen::Matrix<double, 7, 1>;

namespace {
struct ResultTuple {
    ResultTuple(std::size_t n) {
        source.reserve(n);
        target.reserve(n);
    }
    std::vector<Eigen::Vector3d> source;
    std::vector<Eigen::Vector3d> target;
};
struct ColorResultTuple {
    ColorResultTuple(std::size_t n) {
        source.reserve(n);
        target.reserve(n);
    }
    std::vector<Vector7d> source;
    std::vector<Vector7d> target;
};


}  // namespace

namespace kiss_lv {
Color_VoxelHashMap::Vector7dVectorTuple Color_VoxelHashMap::Color_GetCorrespondences(
    const Vector7dVector &points, double max_correspondance_distance) const {
    auto GetClosestNeighboor = [&](const Vector7d &point) {
       	std::vector<Voxel> voxels;  
	    int max_points_per_voxel;
	    max_points_per_voxel = max_points_per_voxel_;
		voxels.reserve(range_1);
		auto kx = static_cast<int>(std::round(point[0] / voxel_size_));
		auto ky = static_cast<int>(std::round(point[1] / voxel_size_));
		auto kz = static_cast<int>(std::round(point[2] / voxel_size_));
		for (int i = kx-num_range_1 ; i <= kx +num_range_1; ++i) {
			for (int j = ky-num_range_1 ; j <= ky+num_range_1; ++j) {
				for (int k = kz-num_range_1 ; k <= kz +num_range_1; ++k) {
			    	Voxel voxel1;
			    	if (point[3]== 255 || point[5]==255)
			    		voxel1<< i, j, k,-1;
			    	else
			    		voxel1<< i, j, k,1;
			    	voxels.emplace_back(voxel1);
				}
			}
		}
        using Vector7dVector = std::vector<Vector7d>;
        Vector7dVector neighboors;
        neighboors.reserve(range_1*max_points_per_voxel);
        std::for_each(voxels.cbegin(), voxels.cend(), [&](const auto &voxel) {
            auto search = colormap_.find(voxel);
            if (search != colormap_.end()) {
                const auto &points = search->second.points;
		            if (!points.empty()) {
		                for (const auto &point2 : points) {
		                    	neighboors.emplace_back(point2); 
		                }
		            }

            }
        });
        Vector7d closest_neighbor;
        double closest_distance = std::numeric_limits<double>::max();
		std::for_each(neighboors.cbegin(), neighboors.cend(), [&](const auto &neighbor) {
			double distance;
			distance = ((neighbor.template head<3>() - point.template head<3>()).template cast<double>().norm());
			if (distance < closest_distance) {
				closest_distance = distance;
				closest_neighbor = neighbor;
				}
		});
        return closest_neighbor;
    };
	using points_iterator = std::vector<Vector7d>::const_iterator;
	const auto [source, target] = tbb::parallel_reduce(
		tbb::blocked_range<points_iterator>{points.cbegin(), points.cend()},
		ColorResultTuple(points.size()),
		[max_correspondance_distance, &GetClosestNeighboor](
		    const tbb::blocked_range<points_iterator>& r, ColorResultTuple res) -> ColorResultTuple {
		    auto& [src, tgt] = res;
		    src.reserve(r.size());
		    tgt.reserve(r.size());
		    for (const auto& point : r) {
		        Vector7d closest_neighboors = GetClosestNeighboor(point);
		        if ((closest_neighboors.template head<3>() - point.template head<3>()).template cast<double>().norm()<max_correspondance_distance){
			        src.emplace_back(point);
			        tgt.emplace_back(closest_neighboors);
			    }
		    }
		    return res;
		},
		[](ColorResultTuple a, const ColorResultTuple& b) -> ColorResultTuple {
		    auto& [src, tgt] = a;
		    const auto& [srcp, tgtp] = b;
		    src.insert(src.end(),
		               std::make_move_iterator(srcp.begin()), std::make_move_iterator(srcp.end()));
		    tgt.insert(tgt.end(),
		               std::make_move_iterator(tgtp.begin()), std::make_move_iterator(tgtp.end()));
		    return a;
		});


    return std::make_tuple(source, target);
}

std::vector<Vector7d> Color_VoxelHashMap::Pointcloud() const { 

    std::vector<Vector7d> points;
    points.reserve(max_points_per_voxel_ * colormap_.size());
    for (const auto &[voxel, voxel_block] : colormap_) {
        (void)voxel;
        for (const auto &point : voxel_block.points) {
            points.push_back(point);
        }
    }
    return points;
}


void Color_VoxelHashMap::Color_Update(const Vector7dVector &points, const Eigen::Vector3d &origin) {
    Color_AddPoints(points);
    Color_RemovePointsFarFromLocation(origin); 
}

void Color_VoxelHashMap::Color_Update(const Vector7dVector &points, const Sophus::SE3d &pose) {
	Vector7dVector vectors;
	vectors.reserve(points.size());
	std::transform(points.cbegin(), points.cend(), std::back_inserter(vectors),
		           [&](const auto &point) {
		               Eigen::Vector3d transformed_point = (pose * point.template head<3>()).template cast<double>();
		               Vector7d vector;
		               vector << transformed_point[0], transformed_point[1], transformed_point[2], point[3], point[4], point[5], point[6];
		               return vector;
		           });
	const Eigen::Vector3d &origin = pose.translation();
	Color_Update(vectors, origin);
}

double EuclideanDistance(const Vector7d& p1, const Vector7d& p2) {
    double dx = p1[0] - p2[0];
    double dy = p1[1] - p2[1];
    double dz = p1[2] - p2[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void Color_VoxelHashMap::Color_AddPoints(const std::vector<Vector7d> &points) {
    for (const auto &point : points) {
        int voxel_x = static_cast<int>(std::round(point[0] / voxel_size_));
        int voxel_y = static_cast<int>(std::round(point[1] / voxel_size_));
        int voxel_z = static_cast<int>(std::round(point[2] / voxel_size_));
        Voxel voxel;
        voxel << voxel_x, voxel_y, voxel_z, 1;
        if (point[3]== 255 || point[5]==255)
			voxel << voxel_x, voxel_y, voxel_z, -1;
		auto search2 = colormap_.find(voxel);
		if (search2 != colormap_.end()) {
			auto &voxel_block = search2.value();
			voxel_block.AddPoint(point, max_points_per_voxel_);
		} 
		else {
			colormap_.insert({voxel, Color_VoxelBlock{{point}}});
		}
    }
}
void Color_VoxelHashMap::Color_RemovePointsFarFromLocation(const Eigen::Vector3d &origin) {
    for (const auto &[voxel, voxel_block] : colormap_) {
        if (voxel[3]== -1){
            const auto &pt = voxel_block.points.front();
		    if (scan_num_-pt[6]>30) {
		    	colormap_.erase(voxel);
		    }
        }
        else{
            const auto &pt = voxel_block.points.front();
		    const auto max_distance2 = 5.0*max_distance_;
		    if ((pt.head<3>() - origin).norm() > (max_distance2) ) {
		    	colormap_.erase(voxel);
		    }
        }
       }
     }
}  // namespace kiss_lv
