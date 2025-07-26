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

#include "KissLV.hpp"
#include <Eigen/Core>
#include <tuple>
#include <vector>
#include <pcl/common/concatenate.h>
#include "kiss_lv/core/Preprocessing.hpp"
#include "kiss_lv/core/Registration.hpp"
#include "kiss_lv/core/VoxelHashMap_Color.hpp"
#include <fstream>
#include <chrono>
using namespace std;
namespace kiss_lv::pipeline {
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Vector7dVector = std::vector<Vector7d>;
using Vector7dVectorTuple = std::tuple<Vector7dVector, Vector7dVector>;
Vector7dVector KissLV::RegisterFrame(const Vector6dVector color_frame, double adj_voxel_size, double density) {
	Vector7dVector color_scan_num;
    for (const auto& vec6d : color_frame) {
        Vector7d vec7d;
        vec7d << vec6d, scan_num;
        color_scan_num.push_back(vec7d);
    }
    const auto vs1 = adj_voxel_size*0.3;
    const auto vs2 = adj_voxel_size;
	const auto cd_scan = kiss_lv::Color_VoxelDownsample(color_scan_num, vs1);
	const auto myTuple = kiss_lv::PL_VoxelDownsample(cd_scan, vs2);
	auto cd_scan2 = std::get<0>(myTuple);
	auto cd_scan3 = std::get<1>(myTuple);
	Sophus::SE3d prediction;
	double sigma = GetAdaptiveThreshold(prediction, adj_voxel_size);
    const auto last_pose = !poses_.empty() ? poses_.back() : Sophus::SE3d();
    const auto initial_guess = last_pose * prediction;
    color_local_map_.Get_adj_voxel_size(density);
    double th_w;
    Sophus::SE3d new_pose;
    Sophus::SE3d final_pose;
	th_w = sigma*vs1;
    double max_distance = sigma*vs2;
	new_pose = kiss_lv::Color_RegisterFrame(cd_scan2,
													cd_scan3,
                                                  color_local_map_,   
                                                  initial_guess, 
                                                  max_distance,
                                                  th_w);  
    const auto model_deviation = initial_guess.inverse() * new_pose;
    adaptive_threshold_.UpdateModelDeviation(model_deviation);
    scan_num++;

    color_local_map_.Color_Update(cd_scan, new_pose);

    poses_.push_back(new_pose);
	/*
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::ofstream foutC("/home/cxl/workspace/KISS_LV/src/kiss_lv/ros/results/time.txt", std::ios::app);
    int exp_num = static_cast<int>(pow(2/vs1+density*0, 3));
	foutC.setf(std::ios::fixed, std::ios::floatfield);
	// 保存结果的精度
	
	foutC.precision(3);
	foutC <<density << " "
		   <<adj_voxel_size<< " "
		   <<cd_scan2.size()<< " "
		   <<exp_num+8<< " "
	       <<duration<< std::endl;
	foutC.close();
	std::cout << "-----------------kiss-lv------------------" << std::endl;
    std::cout << "density：" << density << std::endl; 
    std::cout << "subsample1：" << vs1 << std::endl; 
    std::cout << "subsample2：" << vs2 << std::endl; 
   	std::cout << "cd_scan NUM：" << cd_scan.size() << std::endl;
	//std::cout << "cd_scan2 NUM：" << cd_scan2.size() << std::endl;
	*/
	std::cout << "th_w: " << th_w << std::endl;
    std::cout << "max_correspondance_distance: " << max_distance << std::endl;
	return cd_scan;
}
//-------------------------------------------------------------------------------------------------------
double KissLV::GetAdaptiveThreshold(Sophus::SE3d &prediction, double vs2_size) {
	if (!HasMoved(prediction)) {
		std::cout << "!!!!!!!!!Move too slowly!!!!!!!!!" << std::endl;
	    return config_.initial_threshold;
	}
	return adaptive_threshold_.ComputeThreshold(vs2_size);
}

bool KissLV::HasMoved(Sophus::SE3d &prediction) {
	const size_t N = poses_.size();
	if (N<2) return false;
	prediction = poses_[N - 2].inverse() * poses_[N - 1];
	const double motion = (prediction).translation().norm();
	return motion > config_.min_motion_th;
}

}  // namespace kiss_lv::pipeline
