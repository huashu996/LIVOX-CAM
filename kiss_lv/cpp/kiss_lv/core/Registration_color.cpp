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
#include "Registration.hpp"

#include <tbb/blocked_range.h>
#include <tbb/parallel_reduce.h>
#include <algorithm>
#include <cmath>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include <tuple>

namespace Eigen {
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix3_6d = Eigen::Matrix<double, 3, 6>;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
}  // namespace Eigen

namespace {

inline double square(double x) { return x * x; }

struct ResultTuple {
    ResultTuple() {
        JTJ.setZero();
        JTr.setZero();
    }

    ResultTuple operator+(const ResultTuple &other) {
        this->JTJ += other.JTJ;
        this->JTr += other.JTr;
        return *this;
    }

    Eigen::Matrix6d JTJ;
    Eigen::Vector6d JTr;
};
}
void Color_TransformPoints(const Sophus::SE3d &T, Vector7dVector &points) {
    std::transform(points.cbegin(), points.cend(), points.begin(),
                   [&](const auto &point) { 
                   Eigen::Vector4d homogeneous_point(point[0], point[1], point[2], 1.0);
				   Eigen::Vector4d transformed_homogeneous_point = T.matrix() * homogeneous_point;
				   Eigen::Vector7d transformed_point;
				   transformed_point << transformed_homogeneous_point[0], transformed_homogeneous_point[1], transformed_homogeneous_point[2], point[3], point[4], point[5],point[6];
  				   return transformed_point; });
}
//--------------------------------------------
Sophus::SE3d AlignClouds(const std::vector<Eigen::Vector7d> &source,
                         const std::vector<Eigen::Vector7d> &target,
                         double th) {
    auto compute_jacobian_and_residual = [&](auto i) { 
     	const Eigen::Vector3d pos_residual = (source[i].template head<3>() - target[i].template head<3>()).template cast<double>();
        Eigen::Matrix3_6d J_r;
        J_r.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        J_r.block<3, 3>(0, 3) = -1.0 * Sophus::SO3d::hat((source[i].template head<3>()).template cast<double>());
        return std::make_tuple(J_r, pos_residual);
    };
    const auto &[JTJ, JTr] = tbb::parallel_reduce(
        tbb::blocked_range<size_t>{0, source.size()},
        ResultTuple(),

        [&](const tbb::blocked_range<size_t> &r, ResultTuple J) -> ResultTuple {
            auto Weight = [&](double residual2) { return square(th) / square(th + residual2); }; 
            auto &[JTJ_private, JTr_private] = J;
            for (auto i = r.begin(); i < r.end(); ++i) {
                const auto &[J_r, pos_residual] = compute_jacobian_and_residual(i);
                double w = ((Weight(pos_residual.squaredNorm())));
                JTJ_private.noalias() += J_r.transpose() *w* J_r;
                JTr_private.noalias() += J_r.transpose() *w* pos_residual;
            }
            return J;
        },
        [&](ResultTuple a, const ResultTuple &b) -> ResultTuple { return a + b; });

    const Eigen::Vector6d x = JTJ.ldlt().solve(-JTr);
    return Sophus::SE3d::exp(x);
}
Sophus::SE3d Color_AlignClouds(const std::vector<Eigen::Vector7d> &source,
                         const std::vector<Eigen::Vector7d> &target) {
    auto compute_jacobian_and_residual = [&](auto i) { 
     	const Eigen::Vector3d pos_residual = (source[i].template head<3>() - target[i].template head<3>()).template cast<double>();
        Eigen::Matrix3_6d J_r;
        J_r.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        J_r.block<3, 3>(0, 3) = -1.0 * Sophus::SO3d::hat((source[i].template head<3>()).template cast<double>());
        return std::make_tuple(J_r, pos_residual);
    };
    const auto &[JTJ, JTr] = tbb::parallel_reduce(
        tbb::blocked_range<size_t>{0, source.size()},
        ResultTuple(),
        [&](const tbb::blocked_range<size_t> &r, ResultTuple J) -> ResultTuple {
            auto &[JTJ_private, JTr_private] = J;
            for (auto i = r.begin(); i < r.end(); ++i) {
                const auto &[J_r, pos_residual] = compute_jacobian_and_residual(i);
				auto color_residual = ((source[i].template tail<3>() - target[i].template tail<3>()).template cast<double>()).squaredNorm();
				auto tar_color_norm_sq = (target[i].template tail<3>().template cast<double>()).squaredNorm();
				double wc = tar_color_norm_sq / (tar_color_norm_sq + color_residual);
                JTJ_private.noalias() += J_r.transpose() *wc* J_r;
                JTr_private.noalias() += J_r.transpose() *wc* pos_residual;
            }
            return J;
        },
        [&](ResultTuple a, const ResultTuple &b) -> ResultTuple { return a + b; });
    const Eigen::Vector6d x = JTJ.ldlt().solve(-JTr);
    if (!x.allFinite()) {
        std::cerr << "[Color_AlignClouds] SE3::exp(x) failed due to NaN!" << std::endl;
        return Sophus::SE3d();
    }
    return Sophus::SE3d::exp(x);
}

constexpr int MAX_NUM_ITERATIONS_ = 200;
double ESTIMATION_THRESHOLD_=0.001;
double ESTIMATION_THRESHOLD_2 = 0.0001;
namespace kiss_lv {
Sophus::SE3d Color_RegisterFrame(
						   const Vector7dVector &key_frame,
						   const Vector7dVector &frame,
                           const Color_VoxelHashMap &color_voxel_map,
                           const Sophus::SE3d &initial_guess,
                           double max_correspondence_distance,
                           double kernel) {
    if (color_voxel_map.Color_Empty()) return initial_guess;
    // Equation (9)
    Vector7dVector key_source = key_frame; 
    Vector7dVector source = frame;
    std::cout<<"key_source---"<<key_source.size()<<std::endl;
    std::cout<<"source---"<<source.size()<<std::endl;
    Color_TransformPoints(initial_guess, key_source); 
    // ICP-loop
    Sophus::SE3d T_icp = Sophus::SE3d();
    Sophus::SE3d T_icp2 = Sophus::SE3d();
	for (int i = 0; i < MAX_NUM_ITERATIONS_; ++i) {
	    const auto &[k_src, k_tgt] = color_voxel_map.Color_GetCorrespondences(key_source, max_correspondence_distance);
		if (k_src.size()>20){
			auto estimation = Color_AlignClouds(k_src, k_tgt); 
			Color_TransformPoints(estimation, key_source);
			T_icp = estimation * T_icp;
			if (estimation.log().norm() < ESTIMATION_THRESHOLD_){
				std::cout<<"i---"<<i<<std::endl;
				break;
			}
		}
	}
	Color_TransformPoints(T_icp *initial_guess, source); 
	
	for (int j = 0; j < MAX_NUM_ITERATIONS_; ++j) {
        // Equation (10)
        const auto &[src, tgt] = color_voxel_map.Color_GetCorrespondences(source, max_correspondence_distance);
        if (src.size()>20){
		    auto estimation2 = AlignClouds(src, tgt, kernel); 
		    Color_TransformPoints(estimation2, source);
		    T_icp2 = estimation2 * T_icp2;

		    if (estimation2.log().norm() < ESTIMATION_THRESHOLD_2){
		    	std::cout<<"j---"<<j<<std::endl;
		    	break;
		    }
        }
    }
    
    return T_icp2*T_icp*initial_guess;
}

}  // namespace kiss_lv
