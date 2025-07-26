#pragma once
#ifndef KISS_LV_H
#define KISS_LV_H
//common
#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Core>
#include <vector>
#include <cmath>
#include <algorithm>
// ROS
#include "ros/ros.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Imu.h>
#include "sensor_msgs/PointCloud2.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "sensor_msgs/point_cloud2_iterator.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <cv_bridge/cv_bridge.h>
//thread
#include <mutex>
#include <thread>
#include <signal.h>
#include <condition_variable>
#include <memory>
//file
#include "kiss_lv/pipeline/KissLV.hpp"
#include "kiss_lv/core/VoxelHashMap_Color.hpp"
#include <livox_ros_driver/CustomMsg.h>
#include <tbb/parallel_for.h>
#include <lsd_tool.hpp>
#include <common_lib.h>

//pcl
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/random_sample.h>
//opencv
#include <opencv2/opencv.hpp>
#include <opencv2/line_descriptor/descriptor.hpp>
#include <opencv2/features2d.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/core/eigen.hpp>

using namespace std;
using namespace cv;
using namespace cv::ximgproc;
using PointField = sensor_msgs::PointField;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector6dVector = std::vector<Vector6d>;
using PointField = sensor_msgs::PointField;
MeasureGroup meas;
//----------------Class---------------------
class KISS_LV {
public:
    /// Ros node stuff
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    ros::Subscriber sub_com_img_;
    ros::Subscriber sub_img_;
    ros::Subscriber sub_groundtruth_;
    ros::Subscriber sub_lidar_;
    ros::Subscriber sub_pointcloud_;
    
    ros::Publisher repub_point;
    ros::Publisher odom_publisher_;
    ros::Publisher traj_publisher_;
    nav_msgs::Path path_msg_;
    ros::Publisher frame_publisher_;
    ros::Publisher kpoints_publisher_;
    ros::Publisher local_map_publisher_;
    ros::Publisher featherimage_pub;
    ros::Publisher heatimage_pub;
    int queue_size_{10000};
    ros::Time base_timestamp;
    ros::Time save_timestamp;
    ros::Time gps_base_timestamp;
    ros::Time gps_save_timestamp;
    nav_msgs::Odometry initial_odom;
    tf2_ros::Buffer tf_buffer;
	std::string target_frame = "base_link";
	sensor_msgs::PointCloud2 transformed_cloud;
    // KISS-ICP
    kiss_lv::pipeline::KissLV LVodometry_;
    kiss_lv::pipeline::KISSConfig config_;

	//yaml
	std::string odom_frame_{"odom"};
    std::string child_frame_{"base_link"};
    std::string lidar_type; 
	std::string lidar_topic;
	std::string image_topic;
	std::string ROOT_DIR; 
	bool en_cam;
    int W;
	int H;
	int line_th;
	int line_len;
	int line_wide;
	int point_th;
	int radius_size;
	int pcd_index = 0; 
	int save_frame_num_beg;
	int save_frame_num_end;
	bool Deskew = false;
	bool Save_path = false;
	bool pcd_save_en = false;
	bool use_cam = false;
	bool use_imu = false;
	bool first_scan=true;
	//sync
	double last_timestamp_lidar = -1;
	double last_timestamp_image = -1;
	double last_timestamp_GPS = -1;
	std::deque<sensor_msgs::PointCloud2::ConstPtr> lidar_buffer;
	std::deque<nav_msgs::Odometry> GPS_buffer;
	std::deque<sensor_msgs::ImageConstPtr> image_buffer;
	std::mutex mtx_buffer;
	std::condition_variable sig_buffer;
	std::thread Test_sync_thread_;
	bool b_exit = false;
	bool b_reset = false;
	bool flg_reset = false;
	bool first_message_received = false;
	bool lidar_pushed, flg_first_scan = true;
	double lidar_end_time = 0, first_lidar_time = 0.0;
	double lidar_mean_scantime = 0.0;
	int    scan_num = 0;
	int resize_num;
	int resize;
	double adj_voxel_size;
    double density;
    double exp_key_num = 1000;

	PointCloudXYZI::Ptr good_points;
	PointCloudXYZRGB::Ptr scan_keypoint_enhance;
	PointCloudXYZRGB::Ptr color_cloud;
	PointCloudXYZRGB::Ptr map_cloud;
	PointCloudXYZRGB::Ptr complete_map;
	PointCloudXYZRGB::Ptr save_map_points;

	Eigen::Matrix<double, 3, 3, Eigen::RowMajor> ros_intrisicMat;
	Eigen::Matrix<double, 4, 4, Eigen::RowMajor> ros_extrinsicMat_RT;
	Eigen::Matrix<double, 1, 5> ros_distCoeffs;
	std::vector< double > ros_extrinsicMat_RT_data, ros_intrisicMat_data, ros_distCoeffs_data;
	cv::Mat distCoeffs = cv::Mat(1, 5, cv::DataType<double>::type);
    cv::Mat extrinsicMat_RT = cv::Mat(4, 4, cv::DataType<double>::type);
    cv::Mat un_intrisicMat = cv::Mat(3, 3, cv::DataType<double>::type);
    cv::Mat intrisicMat = cv::Mat(3, 4, cv::DataType<double>::type);
    cv::Mat intrisicMat_Resize = cv::Mat(3, 4, cv::DataType<double>::type);
   

   	void livox_handler(const livox_ros_driver::CustomMsg::ConstPtr& livox_msg_in);
   	void ruby128_handler(const sensor_msgs::PointCloud::ConstPtr& cloud_msg);
   	void PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void Image_Callback(const sensor_msgs::ImageConstPtr &msg);
    void ComImage_Callback(const sensor_msgs::CompressedImageConstPtr& msg);
    bool Sync_packages(MeasureGroup &measgroup);
    void Register_Color_Frame(const sensor_msgs::PointCloud2::ConstPtr& lidar_msg, const sensor_msgs::ImageConstPtr &img_msg);
    void processLidarData(const sensor_msgs::PointCloud2::ConstPtr& lidar_msg, std::vector<Eigen::Vector3d> &deskew_scan);
    Vector6dVector Init_scan(const Vector6dVector& color_cloud, double &adj_voxel_size, double &density);
	void processCameraData(const sensor_msgs::ImageConstPtr &msg, cv::Mat &new_image, cv::Mat &feather_image, cv::Mat intrisicMat_Resize);
	Vector6dVector XYZRGB2Vector6dVector(const PointCloudXYZRGB::Ptr& pcl_cloud);
	void resetParameters();
    void LVO();
    KISS_LV(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
    	: nh_(nh), pnh_(pnh){
		pnh_.getParam("common/pcd_save_en", pcd_save_en);
		pnh_.getParam("common/Save_path", Save_path);
		pnh_.getParam("common/use_cam", use_cam);
		pnh_.getParam("common/ROOT_DIR", ROOT_DIR);
		pnh_.getParam("common/save_frame_num_beg", save_frame_num_beg);
		pnh_.getParam("common/save_frame_num_end", save_frame_num_end);
		pnh_.getParam("common/lidar_topic", lidar_topic);
		pnh_.getParam("common/image_topic", image_topic);
		pnh_.getParam("common/child_frame", child_frame_);
		pnh_.getParam("common/odom_frame", odom_frame_);
		
		
		pnh_.getParam("kiss_l/voxel_size", config_.voxel_size);
		pnh_.getParam("kiss_l/max_range", config_.max_range);
		pnh_.getParam("kiss_l/min_range", config_.min_range);
		pnh_.getParam("kiss_l/min_motion_th", config_.min_motion_th);
		pnh_.getParam("kiss_l/initial_threshold", config_.initial_threshold);
		pnh_.getParam("kiss_l/lidar_type", config_.type);
		pnh_.getParam("kiss_l/Deskew", config_.deskew);
		pnh_.getParam("kiss_l/Exp_key_num", exp_key_num);
		
		pnh_.getParam( "kiss_v/line_len", line_len);
		pnh_.getParam( "kiss_v/line_wide", line_wide);
		pnh_.getParam( "kiss_v/point_th", point_th);
		pnh_.getParam( "kiss_v/radius_size", radius_size);
		pnh_.getParam( "kiss_v/image_height", H);
		pnh_.getParam( "kiss_v/image_weight", W);
		pnh_.getParam( "kiss_v/distCoeffs", ros_distCoeffs_data);
		pnh_.getParam( "kiss_v/intrisicMat", ros_intrisicMat_data);
		pnh_.getParam( "kiss_v/extrinsicMat_RT", ros_extrinsicMat_RT_data);
		std::string image_com_topic;
		image_com_topic = std::string(image_topic).append("/compressed");
		LVodometry_ = kiss_lv::pipeline::KissLV(config_);
		if (config_.max_range < config_.min_range) {
		    ROS_WARN("[WARNING] max_range is smaller than min_range, setting min_range to 0.0");
		    config_.min_range = 0.0;
		}
		if ( ( ros_intrisicMat_data.size() != 9 ) || ( ros_distCoeffs_data.size() != 5 ) || ( ros_extrinsicMat_RT_data.size() != 16 ))
		{
		    cout << "Load parameter fail!!!, please check!!!" << endl;
		    std::this_thread::sleep_for( std::chrono::seconds( 3000000 ) );
		}
		ros_extrinsicMat_RT = Eigen::Map< Eigen::Matrix< double, 4, 4, Eigen::RowMajor > >( ros_extrinsicMat_RT_data.data());
		extrinsicMat_RT = cv::Mat(4, 4, CV_64F, ros_extrinsicMat_RT_data.data()).clone();
		ros_intrisicMat = Eigen::Map< Eigen::Matrix< double, 3, 3, Eigen::RowMajor > >( ros_intrisicMat_data.data());
		un_intrisicMat = cv::Mat(3, 3, CV_64F, ros_intrisicMat_data.data()).clone();
		cv::hconcat(un_intrisicMat, cv::Mat::zeros(3, 1, CV_64F), intrisicMat);
		ros_distCoeffs = Eigen::Map< Eigen::Matrix< double, 1, 5, Eigen::RowMajor > >( ros_distCoeffs_data.data());
		distCoeffs = cv::Mat(1, 5, CV_64F, ros_distCoeffs_data.data()).clone();
		resize_num = std::sqrt(std::round(static_cast<double>(H * W) / 160000));
		resize = (resize_num < 2) ? 1 : ((resize_num % 2 == 0) ? resize_num : resize_num + 1);
		cout<<"resize"<<resize<<endl;
		intrisicMat_Resize = intrisicMat / resize;
		intrisicMat_Resize.at<double>(2, 2) =1.0;
		if (child_frame_ != "base_link") {
			static tf2_ros::StaticTransformBroadcaster br;
		    geometry_msgs::TransformStamped alias_transform_msg;
		    alias_transform_msg.header.stamp = ros::Time::now();
		    alias_transform_msg.transform.translation.x = 65;
		    alias_transform_msg.transform.translation.y = 0.0;
		    alias_transform_msg.transform.translation.z = 0.0;
		    alias_transform_msg.transform.rotation.x = 0.0;
		    alias_transform_msg.transform.rotation.y = -sin(M_PI/4);
		    alias_transform_msg.transform.rotation.z = 0.0;
		    alias_transform_msg.transform.rotation.w = cos(M_PI/4);
		    alias_transform_msg.header.frame_id = child_frame_;
		    alias_transform_msg.child_frame_id = "base_link";
		    br.sendTransform(alias_transform_msg);
    	}
    	good_points.reset(new PointCloudXYZI());
    	scan_keypoint_enhance.reset(new PointCloudXYZRGB());
    	map_cloud.reset(new PointCloudXYZRGB());
    	save_map_points.reset(new PointCloudXYZRGB());
    	color_cloud.reset(new PointCloudXYZRGB());
    	complete_map.reset(new PointCloudXYZRGB());
		odom_publisher_ = pnh_.advertise<nav_msgs::Odometry>("odometry", queue_size_);
		frame_publisher_ = pnh_.advertise<sensor_msgs::PointCloud2>("frame", queue_size_);
		kpoints_publisher_ = pnh_.advertise<sensor_msgs::PointCloud2>("keypoints", queue_size_); 
		local_map_publisher_ = pnh_.advertise<sensor_msgs::PointCloud2>("local_map", queue_size_);
		featherimage_pub = pnh_.advertise<sensor_msgs::Image>("feather_image", 1);
		heatimage_pub = pnh_.advertise<sensor_msgs::Image>("heat_image", 1);
		path_msg_.header.frame_id = odom_frame_;
		traj_publisher_ = pnh_.advertise<nav_msgs::Path>("trajectory", queue_size_);
		if (config_.type == "LIVOX") {
			printf( "LIVOX\n" );
			sub_lidar_ = nh_.subscribe( lidar_topic, queue_size_, &KISS_LV::livox_handler, this);
			repub_point = nh_.advertise< sensor_msgs::PointCloud2 >( "/it_point", queue_size_);
			sub_pointcloud_ = nh_.subscribe<sensor_msgs::PointCloud2>("/it_point", queue_size_, &KISS_LV::PointCloudCallback, this, ros::TransportHints().tcpNoDelay());
		}
		else {
			printf( "Lidar type is wrong.\n" );
		}
        sub_img_ = nh_.subscribe(image_topic.c_str(), queue_size_, &KISS_LV::Image_Callback,this, ros::TransportHints().tcpNoDelay());
		sub_com_img_ = nh_.subscribe(image_com_topic, queue_size_, &KISS_LV::ComImage_Callback,this, ros::TransportHints().tcpNoDelay());
		Test_sync_thread_ = std::thread(&KISS_LV::LVO, this);
		//Print
		if(1)
        {
			cout << "[Ros_parameter]: common/use_cam: " <<use_cam<< endl;
			cout << "[Ros_parameter]: common/lidar_topic: " <<lidar_topic<< endl;
			cout << "[Ros_parameter]: common/image_topic: " <<image_topic<< endl;
			cout << "[Ros_parameter]:image_sizeW*H: " <<W<<"*"<<H<< endl;
			cout << "[Ros_parameter]: config_.voxel_size: " << config_.voxel_size << endl;
			cout << "[Ros_parameter]: config_.initial_threshold: " << config_.initial_threshold << endl;
			cout << "[Ros_parameter]: config_.min_motion_th: " << config_.min_motion_th << endl;
			cout << "[Ros_parameter]: config_.max_range: " <<  config_.max_range << endl;
			cout << "[Ros_parameter]: config_.min_range: " << config_.min_range << endl;
			cout << "[Ros_parameter]: config_.type: " << config_.type << endl;
			cout << "[Ros_parameter]: config_.deskew: " << config_.deskew << endl;
			ROS_INFO("KISS-LV ROS 1 Odometry Node Initialized");
        }
    }
     ~KISS_LV(){};
};
#endif // 
