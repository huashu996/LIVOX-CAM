#include "kiss_lv.hpp"
#include <opencv2/xphoto/white_balance.hpp>
std::mutex mutex_image_callback;
LSDOptions opts;
cv::Mat prev_descriptors;
std::vector<cv::KeyPoint> prev_keypoints;
std::vector<Vec4f> prev_lines_ls;
constexpr double mid_pose_timestamp{0.5};
int sub_image_typed = 0; // 0: TBD 1: sub_raw, 2: sub_comp
Vector6dVector Get_FeatrueScan(const std::vector<Eigen::Vector3d>& laser_data,
                               const cv::Mat& intrisicMat,
                               const cv::Mat& extrinsicMat_RT,
                               const cv::Mat& feather_image);
Vector6dVector Get_map_cloud(const std::vector<Eigen::Vector3d>& laser_data,
                                           const cv::Mat& intrisicMat,
                                           const cv::Mat& extrinsicMat_RT,
                                           const cv::Mat& new_image);                    
Vector6dVector Trans_Vector6(const std::vector<Eigen::Vector3d>& laser_data);  
PointCloudXYZRGB::Ptr Eigen7dToPointCloud2(const std::vector<Vector7d> &points);     
pcl::PointCloud<pcl::PointXYZRGB>::Ptr ConvertToXYZRGBPointCloud(const Vector6dVector& map_cloud);
cv::Mat ALTM_retinex(const cv::Mat& img);
cv::Mat get_image_keypoints(cv::Mat gray_image, int line_th, int line_len, int line_wide, int point_th, int radius_size);
double CalculatePointCloudVolume(Vector6dVector& frame, double voxel_size, int th_num, int &voxel_num);
std::vector<Eigen::Vector3d> PointCloud2ToEigen(const sensor_msgs::PointCloud2 &msg);
std::vector<double> Livox_time(const sensor_msgs::PointCloud2 &msg);
std::vector<Eigen::Vector3d> DeSkewScan(const std::vector<Eigen::Vector3d> &frame,
                                        const std::vector<double> &timestamps,
										const std::vector<Sophus::SE3d>& pose_deskew);
std::vector<Eigen::Vector3d> CorrectKITTIScan(const std::vector<Eigen::Vector3d> &frame);
std::vector<double> GetVelodyneTimestamps(const std::vector<Eigen::Vector3d> &points);
std::vector<double> GetTimestamps(const sensor_msgs::PointCloud2 &msg);
cv::Mat feather_image;
//-------------------------------------------------------------------------------------------------------------------------------------------
void KISS_LV::livox_handler(const livox_ros_driver::CustomMsg::ConstPtr& livox_msg_in) {
	PointCloudXYZI::Ptr pcl_data(new PointCloudXYZI);
	auto time_end = livox_msg_in->points.back().offset_time;
    for (unsigned int i = 0; i < livox_msg_in->point_num; ++i) {
        PointTypeI pt;
        pt.x = livox_msg_in->points[i].x;
        pt.y = livox_msg_in->points[i].y;
        pt.z = livox_msg_in->points[i].z;
        double time = livox_msg_in->points[i].offset_time / (double)time_end;
        pt.intensity = time;
        pcl_data->push_back(pt);
    }
    sensor_msgs::PointCloud2 T_pointcloud;
    pcl::toROSMsg(*pcl_data, T_pointcloud);
    T_pointcloud.header = livox_msg_in->header;
    repub_point.publish(T_pointcloud);
}
void KISS_LV::ruby128_handler(const sensor_msgs::PointCloud::ConstPtr& cloud_msg) {
    sensor_msgs::PointCloud2 cloud2_msg;
    sensor_msgs::convertPointCloudToPointCloud2(*cloud_msg, cloud2_msg);
    repub_point.publish(cloud2_msg);
}

void KISS_LV::PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg){
	double timestamp = msg->header.stamp.toSec();
	mtx_buffer.lock();
	ROS_DEBUG("get point cloud at time: %.6f", timestamp);
	if (timestamp < last_timestamp_lidar) 
	{
	    ROS_ERROR("lidar loop back, clear buffer");
	    lidar_buffer.clear();
	}
	last_timestamp_lidar = timestamp;
	lidar_buffer.push_back(msg);
	mtx_buffer.unlock();
	sig_buffer.notify_all();	
}

void KISS_LV::LVO(){
		while (ros::ok()) {
			MeasureGroup meas;
			std::unique_lock<std::mutex> lock(mtx_buffer);
			sig_buffer.wait(lock, [this, &meas]() -> bool { return Sync_packages(meas) || b_exit; });
			lock.unlock();
			if (b_exit) 
			{
			    ROS_INFO("b_exit=true, exit");
			    break;
			}
			if (b_reset) 
			{
			    ROS_WARN("reset when rosbag play back");
			    b_reset = false;
			    continue;
			}
			save_timestamp=meas.lidar_msg->header.stamp;
			auto start = std::chrono::steady_clock::now();
			KISS_LV::Register_Color_Frame(meas.lidar_msg, meas.image);
			auto end = std::chrono::steady_clock::now();
			std::chrono::duration<double, std::milli> elapsed = end - start;
			std::cout << "Register_Color_Frame took " << elapsed.count() << " ms" << std::endl;
	}
}
bool KISS_LV::Sync_packages(MeasureGroup &measgroup) 
	{
		if (lidar_buffer.empty()) 
		{
			return false;
		}
		if (!lidar_pushed){
			//----------------------use_cam---------------------
			if (use_cam){
				if (image_buffer.empty()) 
				{
					return false;
				}
				if ((image_buffer.front()->header.stamp.toSec() > lidar_buffer.back()->header.stamp.toSec())) 
				{
					lidar_buffer.clear();
					ROS_ERROR("clear lidar buffer, only happen at the beginning===============");
					return false;
				}
			
				if (image_buffer.back()->header.stamp.toSec() < lidar_buffer.front()->header.stamp.toSec()) 
				{
					return false;
				}
			}
			lidar_pushed = true;
		}
		measgroup.lidar_msg = lidar_buffer.front();
		lidar_buffer.pop_front();
		double lidar_time = measgroup.lidar_msg->header.stamp.toSec();
		double min_time_diff = std::numeric_limits<double>::max();

		for (const auto &image : image_buffer) {
			double image_time = image->header.stamp.toSec();
			if (image_time <= lidar_time) {
				double time_diff = std::abs(lidar_time - image_time);
				if (time_diff < min_time_diff) {
				    min_time_diff = time_diff;
				    measgroup.image = image;
				}
				image_buffer.pop_front();
			}
		}
		lidar_pushed = false;
		return true;
}

void KISS_LV::ComImage_Callback(const sensor_msgs::CompressedImageConstPtr& msg)
{
    std::unique_lock<std::mutex> lock2(mutex_image_callback);
    if (sub_image_typed == 2) {
        return;
    }
    sub_image_typed = 1;
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    sensor_msgs::ImageConstPtr image_msg = cv_ptr->toImageMsg();
    if (msg->header.stamp.toSec() < last_timestamp_image) {
        ROS_ERROR("img loop back, clear buffer");
        mtx_buffer.lock();
        image_buffer.clear();
        mtx_buffer.unlock();
    }

    mtx_buffer.lock();
    image_buffer.push_back(image_msg);
    last_timestamp_image = msg->header.stamp.toSec();
    mtx_buffer.unlock();

    sig_buffer.notify_all();
}
void KISS_LV::Image_Callback(const sensor_msgs::ImageConstPtr& image_msg) {
    std::unique_lock<std::mutex> lock2(mutex_image_callback);
    if (sub_image_typed == 2) {
        return;
    }
    sub_image_typed = 1;
    if (image_msg->header.stamp.toSec() < last_timestamp_image) {
        ROS_ERROR("img loop back, clear buffer");
        mtx_buffer.lock();
        image_buffer.clear();
        mtx_buffer.unlock();
    }

    mtx_buffer.lock();
    image_buffer.push_back(image_msg);
    last_timestamp_image = image_msg->header.stamp.toSec();
    mtx_buffer.unlock();

    sig_buffer.notify_all();
}

void KISS_LV::Register_Color_Frame(const sensor_msgs::PointCloud2::ConstPtr& lidar_msg, const sensor_msgs::ImageConstPtr &img_msg) {
	auto start = std::chrono::high_resolution_clock::now();
	std::vector<Eigen::Vector3d> deskew_scan;
	deskew_scan.clear();
	Vector6dVector color_cloud;
	Vector6dVector map_cloud;
	double adj_voxel_size, density;
	cv::Mat new_image;
	if (use_cam){
		std::thread lidarThread([&]() { processLidarData(lidar_msg, deskew_scan); });
		std::thread cameraThread([&]() { processCameraData(img_msg, new_image, feather_image, intrisicMat_Resize); });
		lidarThread.join();
		cameraThread.join();
		color_cloud = Get_FeatrueScan(deskew_scan, intrisicMat_Resize, extrinsicMat_RT, feather_image);
		map_cloud = Get_map_cloud(deskew_scan, intrisicMat, extrinsicMat_RT, new_image);

	}
	if (!use_cam) {
		processLidarData(lidar_msg, deskew_scan);
		color_cloud = Trans_Vector6(deskew_scan);
		map_cloud = color_cloud;
	}
	
	const auto Input_scan = Adaptive_spatial_Module(color_cloud, adj_voxel_size, density);
	const auto keypoint = LVodometry_.RegisterFrame(Input_scan, adj_voxel_size, density);

	const auto pose = LVodometry_.poses().back();
	auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::ofstream foutC("/home/cxl/workspace/KISS_LV/src/kiss_lv/ros/results/cost_time.txt", std::ios::app);
	foutC.setf(std::ios::fixed, std::ios::floatfield);
	foutC.precision(3);
	foutC <<duration << std::endl;
	foutC.close();
	auto save_local_map_ = LVodometry_.LocalMap();
	scan_keypoint_enhance = Eigen7dToPointCloud2(keypoint);
	save_map_points = Eigen7dToPointCloud2(save_local_map_);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	rgb_cloud = ConvertToXYZRGBPointCloud(map_cloud);
    const Eigen::Vector3d t_current = pose.translation();
    const Eigen::Quaterniond q_current = pose.unit_quaternion();

    //---------------------------------------Pub--------------------------------------------
    geometry_msgs::TransformStamped transform_msg;
    transform_msg.header.stamp = ros::Time::now();
    transform_msg.header.frame_id = odom_frame_;
    transform_msg.child_frame_id = child_frame_;
    transform_msg.transform.rotation.x = q_current.x();
    transform_msg.transform.rotation.y = q_current.y();
    transform_msg.transform.rotation.z = q_current.z();
    transform_msg.transform.rotation.w = q_current.w();
    transform_msg.transform.translation.x = t_current.x();
    transform_msg.transform.translation.y = t_current.y();
    transform_msg.transform.translation.z = t_current.z();
    tf_broadcaster_.sendTransform(transform_msg);
	//----------------------------------------------------------

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = save_timestamp;                                                                    
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = child_frame_;
    odom_msg.pose.pose.orientation.x = q_current.x();
    odom_msg.pose.pose.orientation.y = q_current.y();
    odom_msg.pose.pose.orientation.z = q_current.z();
    odom_msg.pose.pose.orientation.w = q_current.w();
    odom_msg.pose.pose.position.x = t_current.x();
    odom_msg.pose.pose.position.y = t_current.y();
    odom_msg.pose.pose.position.z = t_current.z();
    odom_publisher_.publish(odom_msg);
	
    // tum
    if (Save_path){
    	std::ofstream foutC(string(string(ROOT_DIR) + "/kiss_lv.txt"), std::ios::app);
		foutC.setf(std::ios::fixed, std::ios::floatfield);
		foutC.precision(5);
		foutC << save_timestamp << " "
		      << odom_msg.pose.pose.position.x << " "
		      << odom_msg.pose.pose.position.y << " "
		      << odom_msg.pose.pose.position.z << " "
		      << odom_msg.pose.pose.orientation.x << " "
		      << odom_msg.pose.pose.orientation.y << " "
		      << odom_msg.pose.pose.orientation.z << " "
		      << odom_msg.pose.pose.orientation.w << std::endl;
		foutC.close();
    }
    // publish trajectory msg
    scan_num++;
    if (scan_num%2==0){
		geometry_msgs::PoseStamped pose_msg;
		pose_msg.pose = odom_msg.pose.pose;
		pose_msg.header = odom_msg.header;
		path_msg_.poses.push_back(pose_msg);
		traj_publisher_.publish(path_msg_);
	}
	sensor_msgs::PointCloud2 rgb_pointscan;
	pcl::toROSMsg(*rgb_cloud, rgb_pointscan);
	rgb_pointscan.header.stamp = ros::Time::now();
	rgb_pointscan.header.frame_id = child_frame_;
	frame_publisher_.publish(rgb_pointscan);
	
	sensor_msgs::PointCloud2 key_pointcloud;
	pcl::toROSMsg(*scan_keypoint_enhance, key_pointcloud);
	key_pointcloud.header.stamp = ros::Time::now();
	key_pointcloud.header.frame_id = child_frame_;
	kpoints_publisher_.publish(key_pointcloud);
	
    sensor_msgs::PointCloud2 map_msg;
	pcl::toROSMsg(*save_map_points, map_msg);	
	map_msg.header.stamp = ros::Time::now();
	map_msg.header.frame_id = odom_frame_;
	local_map_publisher_.publish(map_msg);
	
	
	sensor_msgs::ImagePtr featherimage_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", feather_image).toImageMsg();
	featherimage_pub.publish(featherimage_msg);
	if (rgb_cloud->size() > 0 && pcd_save_en)
	{
		pcd_index++;
		if (pcd_index >= save_frame_num_beg){
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
			for (const auto& point : rgb_cloud->points)
			{
				Eigen::Vector3d transformed_point = (pose * Eigen::Vector3d(point.x, point.y, point.z)).cast<double>();
				pcl::PointXYZRGB transformed_pcl_point;
				transformed_pcl_point.x = transformed_point.x();
				transformed_pcl_point.y = transformed_point.y();
				transformed_pcl_point.z = transformed_point.z();
				transformed_pcl_point.r = point.r;
				transformed_pcl_point.g = point.g;
				transformed_pcl_point.b = point.b;
				transformed_rgb_cloud->push_back(transformed_pcl_point);
			}
			*complete_map += *transformed_rgb_cloud;
		string all_points_dir(string(string(ROOT_DIR) + "/scans_") + to_string(save_frame_num_end) + string(".ply"));
		pcl::PLYWriter ply_writer;
		if (pcd_index == save_frame_num_end)
		{
		    cout << "current scan saved to /PLY/:" << all_points_dir << endl;
		    ply_writer.write(all_points_dir, *complete_map);
		}
	    }
	}
	KISS_LV::resetParameters();

}

void KISS_LV::processLidarData(const sensor_msgs::PointCloud2::ConstPtr& lidar_msg, std::vector<Eigen::Vector3d> &deskew_scan) {
	const auto points = PointCloud2ToEigen(*lidar_msg);
	if (config_.deskew){
		const auto pose_deskew = LVodometry_.poses();
		if (config_.type == "LIVOX"){
			const auto timestamps = Livox_time(*lidar_msg);
			deskew_scan = DeSkewScan(points, timestamps, pose_deskew);
		}
	}
	if (!config_.deskew){
		deskew_scan = points;
	}
}

void KISS_LV::processCameraData(const sensor_msgs::ImageConstPtr &msg,
                                 cv::Mat &new_image,
                                 cv::Mat &feather_image,
                                 cv::Mat intrisicMat_Resize)
{
    try {
        static cv::Mat map1, map2;
        static bool map_initialized = false;
        static cv::Ptr<cv::xphoto::SimpleWB> wb = cv::xphoto::createSimpleWB();
        wb->setInputMin(0.0f);
        wb->setInputMax(255.0f);

        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        const cv::Mat& raw_img = cv_ptr->image;

        if (!map_initialized) {
            cv::initUndistortRectifyMap(un_intrisicMat, distCoeffs, cv::Mat(),
                                        un_intrisicMat, raw_img.size(), CV_16SC2, map1, map2);
            map_initialized = true;
        }

        cv::Mat undistorted;
        cv::remap(raw_img, undistorted, map1, map2, cv::INTER_LINEAR);
        new_image = ALTM_retinex(undistorted);

        cv::Mat small_img;
        cv::resize(new_image, small_img, cv::Size(W / resize, H / resize));
        wb->balanceWhite(small_img, small_img);

        cv::Mat gray;
        cv::cvtColor(small_img, gray, cv::COLOR_BGR2GRAY);
        cv::medianBlur(gray, gray, 3);
        feather_image = get_image_keypoints(gray, line_th, line_len, line_wide, point_th, radius_size);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}


void KISS_LV::resetParameters(){ 
	  good_points->clear();
	  scan_keypoint_enhance->clear();
	  color_cloud->clear();
	  map_cloud->clear();
	  save_map_points->clear();
}

std::vector<Eigen::Vector3d> PointCloud2ToEigen(const sensor_msgs::PointCloud2 &msg) {
    std::vector<Eigen::Vector3d> points;
    points.reserve(msg.height * msg.width);
    sensor_msgs::PointCloud2ConstIterator<float> msg_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> msg_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> msg_z(msg, "z");
    for (size_t i = 0; i < msg.height * msg.width; ++i, ++msg_x, ++msg_y, ++msg_z) {
        points.emplace_back(*msg_x, *msg_y, *msg_z);
    }
    return points;
}
std::vector<double> Livox_time(const sensor_msgs::PointCloud2 &msg) {
    std::vector<double> timestamps;
    timestamps.reserve(msg.height * msg.width);


    sensor_msgs::PointCloud2ConstIterator<float> intensity_it(msg, "intensity");

    for (size_t i = 0; i < msg.height * msg.width; ++i, ++intensity_it) {

        timestamps.push_back(static_cast<double>(*intensity_it));
    }

    return timestamps;
}
std::vector<Eigen::Vector3d> DeSkewScan(const std::vector<Eigen::Vector3d> &frame,
                                        const std::vector<double> &timestamps,
										const std::vector<Sophus::SE3d>& pose_deskew) {
    
    const size_t N = pose_deskew.size();
    if (N <= 2) return frame;
    const auto& start_pose = pose_deskew[N - 2];
    const auto& finish_pose = pose_deskew[N - 1];
    const auto delta_pose = (start_pose.inverse() * finish_pose).log();
    if (!delta_pose.allFinite()) {
        std::cerr << "[DeSkewScan] Invalid delta_pose (NaN or Inf detected)" << std::endl;
        return frame;
    }
    std::vector<Eigen::Vector3d> corrected_frame(frame.size());
    tbb::parallel_for(size_t(0), frame.size(), [&](size_t i) {
        const auto motion = Sophus::SE3d::exp((timestamps[i] - mid_pose_timestamp) * delta_pose);
        corrected_frame[i] = motion * frame[i];
    });
    return corrected_frame;
}
auto GetTimestampField(const sensor_msgs::PointCloud2 &msg) {
    PointField timestamp_field;
    for (const auto &field : msg.fields) {
        if ((field.name == "t" || field.name == "timestamp" || field.name == "time")) {
            timestamp_field = field;
        }
    }
    if (!timestamp_field.count) {
        throw std::runtime_error("Field 't', 'timestamp', or 'time'  does not exist");
    }
    return timestamp_field;
}
auto NormalizeTimestamps(const std::vector<double> &timestamps) {
    const double max_timestamp = *std::max_element(timestamps.cbegin(), timestamps.cend());
    // check if already normalized
    if (max_timestamp < 1.0) return timestamps;
    std::vector<double> timestamps_normalized(timestamps.size());
    std::transform(timestamps.cbegin(), timestamps.cend(), timestamps_normalized.begin(),
                   [&](const auto &timestamp) { return timestamp / max_timestamp; });
    return timestamps_normalized;
}
auto ExtractTimestampsFromMsg(const sensor_msgs::PointCloud2 &msg, const PointField &field) {
    // Extract timestamps from cloud_msg
    const size_t n_points = msg.height * msg.width;
    std::vector<double> timestamps;
    timestamps.reserve(n_points);

    // Option 1: Timestamps are unsigned integers -> epoch time.
    if (field.name == "t" || field.name == "timestamp") {
        sensor_msgs::PointCloud2ConstIterator<uint32_t> msg_t(msg, field.name);
        for (size_t i = 0; i < n_points; ++i, ++msg_t) {
            timestamps.emplace_back(static_cast<double>(*msg_t));
        }
        // Covert to normalized time, between 0.0 and 1.0
        return NormalizeTimestamps(timestamps);
    }

    // Option 2: Timestamps are floating point values between 0.0 and 1.0
    // field.name == "timestamp"
    sensor_msgs::PointCloud2ConstIterator<double> msg_t(msg, field.name);
    for (size_t i = 0; i < n_points; ++i, ++msg_t) {
        timestamps.emplace_back(*msg_t);
    }
    return timestamps;
}
std::vector<double> GetTimestamps(const sensor_msgs::PointCloud2 &msg) {
    auto timestamp_field = GetTimestampField(msg);
    std::vector<double> timestamps = ExtractTimestampsFromMsg(msg, timestamp_field);

    return timestamps;
}


std::vector<double> GetVelodyneTimestamps(const std::vector<Eigen::Vector3d> &points) {
    std::vector<double> timestamps;
    timestamps.reserve(points.size());
    std::for_each(points.cbegin(), points.cend(), [&](const auto &point) {
        const double yaw = -std::atan2(point.y(), point.x());
        timestamps.emplace_back(0.5 * (yaw / M_PI + 1.0));
    });
    return timestamps;
}
std::vector<Eigen::Vector3d> CorrectKITTIScan(const std::vector<Eigen::Vector3d> &frame) {
    constexpr double VERTICAL_ANGLE_OFFSET = (0.205 * M_PI) / 180.0;
    std::vector<Eigen::Vector3d> corrected_frame(frame.size());
    tbb::parallel_for(size_t(0), frame.size(), [&](size_t i) {
        const auto &pt = frame[i];
        const Eigen::Vector3d rotationVector = pt.cross(Eigen::Vector3d(0., 0., 1.));
        corrected_frame[i] =
            Eigen::AngleAxisd(VERTICAL_ANGLE_OFFSET, rotationVector.normalized()) * pt;
    });
    return corrected_frame;
}
Vector6dVector Get_FeatrueScan(const std::vector<Eigen::Vector3d>& laser_data,
                               const cv::Mat& intrisicMat,
                               const cv::Mat& extrinsicMat_RT,
                               const cv::Mat& feather_image) {
    int H = feather_image.rows;
    int W = feather_image.cols;
    Vector6dVector pl_points;

    Eigen::Matrix<double, 3, 3> intrinsic;
    Eigen::Matrix<double, 4, 4> extrinsic;
    cv::cv2eigen(intrisicMat, intrinsic);
    cv::cv2eigen(extrinsicMat_RT, extrinsic);

    #pragma omp parallel
    {
        Vector6dVector local_points;

        #pragma omp for nowait
        for (int i = 0; i < laser_data.size(); ++i) {
            const auto& pt = laser_data[i];


            if (pt[0] <= 0)
                continue;

            Eigen::Vector4d pointLidar(pt[0], pt[1], pt[2], 1.0);
            Eigen::Vector4d tempPoint = extrinsic * pointLidar;

            if (tempPoint[2] <= 0) continue;

            Eigen::Vector3d imgPoint = intrinsic * tempPoint.head<3>() / tempPoint[2];

            int u = static_cast<int>(imgPoint[0]);
            int v = static_cast<int>(imgPoint[1]);
            if (u >= 0 && u < W && v >= 0 && v < H) {
                Eigen::Vector3d color(55, 100, 55);

                const uchar* pixel = feather_image.ptr<uchar>(v) + 3 * u;
                uchar b = pixel[0];
                uchar g = pixel[1];
                uchar r = pixel[2];

                if ((b == 255 && r == 55) || (b == 55 && r == 255)) {
                    color[0] = static_cast<double>(r);
                    color[1] = 55.0;
                    color[2] = static_cast<double>(b);
                }

                Vector6d point;
                point << pt, color;
                local_points.push_back(point);
            }
        }
        #pragma omp critical
        pl_points.insert(pl_points.end(), local_points.begin(), local_points.end());
    }

    return pl_points;
}
Vector6dVector Get_map_cloud(const std::vector<Eigen::Vector3d>& laser_data,
                             const cv::Mat& intrisicMat,
                             const cv::Mat& extrinsicMat_RT,
                             const cv::Mat& new_image) {
    int H = new_image.rows;
    int W = new_image.cols;
    Vector6dVector rgb_points;

    Eigen::Matrix3d intrinsic;
    Eigen::Matrix4d extrinsic;
    cv::cv2eigen(intrisicMat, intrinsic);
    cv::cv2eigen(extrinsicMat_RT, extrinsic);

    #pragma omp parallel
    {
        Vector6dVector local_points;

        #pragma omp for nowait
        for (int i = 0; i < laser_data.size(); ++i) {
            const auto& pt = laser_data[i];
            Eigen::Vector4d pt_lidar(pt.x(), pt.y(), pt.z(), 1.0);

            Eigen::Vector4d pt_cam = extrinsic * pt_lidar;
            if (pt_cam[2] <= 0) continue;

            Eigen::Vector3d pt_img = intrinsic * pt_cam.head<3>() / pt_cam[2];

            int u = static_cast<int>(pt_img[0]);
            int v = static_cast<int>(pt_img[1]);

            if (u >= 0 && u < W && v >= 0 && v < H) {
                const uchar* pixel = new_image.ptr<uchar>(v) + 3 * u;
                uchar b = pixel[0];
                uchar g = pixel[1];
                uchar r = pixel[2];

                Vector6d rgb_point;
                rgb_point << pt.x(), pt.y(), pt.z(),
                             static_cast<double>(r),
                             static_cast<double>(g),
                             static_cast<double>(b);
                local_points.push_back(rgb_point);
            }
        }
        #pragma omp critical
        rgb_points.insert(rgb_points.end(), local_points.begin(), local_points.end());
    }
    return rgb_points;
}
Vector6dVector Trans_Vector6(const std::vector<Eigen::Vector3d>& laser_data){
	std::vector<Vector6d> laser_data_extended;
	laser_data_extended.reserve(laser_data.size());
	for (const auto& point : laser_data) {
		Vector6d extended_point;
		extended_point << point, 55, 150, 55;
		laser_data_extended.push_back(extended_point);
	}
	return laser_data_extended;
}

Vector6dVector KISS_LV::Adaptive_spatial_Module(const Vector6dVector& color_cloud, double &adj_voxel_size, double &density) {
    Vector6dVector filtered_vector_array;
    int point_num = 0;
    density = 0.0;
    adj_voxel_size = 0.0;
    
    for (const auto& vector : color_cloud) {
        double distance = std::sqrt(vector(0) * vector(0) + vector(1) * vector(1) + vector(2) * vector(2));
        if (distance > config_.min_range && distance < config_.max_range) {
            filtered_vector_array.push_back(vector);
            point_num++;
        }
    }
    int voxel_num;
    double exp_voxel_num = point_num/exp_key_num;
    double init_volume = CalculatePointCloudVolume(filtered_vector_array, config_.voxel_size, 3, voxel_num);
    density = ceil(point_num / voxel_num);
    double vc = pow(exp_voxel_num*config_.voxel_size*config_.voxel_size*config_.voxel_size/density, 1.0/3.0);
    if (!filtered_vector_array.empty()) {
        adj_voxel_size = vc;
    }
    else {
        adj_voxel_size = config_.voxel_size;
        density = 20;
    }
    std::cout<<"+++++++++++++++"<<std::endl;
    cout<<"point_num--"<<point_num<<endl;
    cout<<"adj_voxel_size--"<<adj_voxel_size<<endl;
    cout<<"density--"<<density<<endl;
    return filtered_vector_array;
}

struct VoxelInfo {
    std::set<int> pointIndices;
    bool isSparse = false;
};
struct Vector3iComparator {
    bool operator()(const Eigen::Vector3i& lhs, const Eigen::Vector3i& rhs) const {
        for (int i = 0; i < 3; ++i) {
            if (lhs[i] < rhs[i]) return true;
            if (lhs[i] > rhs[i]) return false;
        }
        return false;
    }
};
double CalculatePointCloudVolume(Vector6dVector& frame, double voxel_size, int min_neighbors, int &voxel_num) {
    voxel_num = 0;
    std::map<Eigen::Vector3i, VoxelInfo, Vector3iComparator> voxel_info_map;
    for (int i = 0; i < frame.size(); ++i) {
        Eigen::Vector3i voxel_indices = (frame[i].head<3>() / voxel_size).cast<int>();
        voxel_info_map[voxel_indices].pointIndices.insert(i);
    }
	for (auto& voxel : voxel_info_map) {
		int point_count = voxel.second.pointIndices.size();
		voxel.second.isSparse = (point_count <= min_neighbors);
	}
    double total_voxel_volume = voxel_size * voxel_size * voxel_size;
    double covered_volume = 0.0;
    Vector6dVector non_sparse_point_cloud;
    for (const auto& voxel : voxel_info_map) {
        if (!voxel.second.isSparse) {
        	voxel_num++;
            covered_volume += total_voxel_volume;
            for (int index : voxel.second.pointIndices) {
                non_sparse_point_cloud.push_back(frame[index]);
            }
        }
    }
    frame = non_sparse_point_cloud;
    return covered_volume;
}

PointCloudXYZRGB::Ptr Eigen7dToPointCloud2(const std::vector<Vector7d> &points) {
    PointCloudXYZRGB::Ptr output_cloud(new PointCloudXYZRGB()); 
    for (const auto& point : points)
    {
        ColorPointType pcl_point;
        pcl_point.x = point[0];
        pcl_point.y = point[1];
        pcl_point.z = point[2];
        pcl_point.r = point[3];
        pcl_point.g = point[4];
        pcl_point.b = point[5];
        output_cloud->push_back(pcl_point);
    }
    return output_cloud;
}
cv::Mat ALTM_retinex(const cv::Mat& img)
{
    cv::Mat ycrcb;
    cv::cvtColor(img, ycrcb, cv::COLOR_BGR2YCrCb);

    std::vector<cv::Mat> channels(3);
    cv::split(ycrcb, channels);

    double tile_size = std::max(img.cols * 32.0 / 1280.0, 4.0);
    cv::Size tile_grid_size(static_cast<int>(tile_size), static_cast<int>(tile_size));

    static thread_local cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(1.0, tile_grid_size);

    clahe->apply(channels[0], channels[0]);

    cv::merge(channels, ycrcb);
    cv::Mat enhanced;
    cv::cvtColor(ycrcb, enhanced, cv::COLOR_YCrCb2BGR);
    return enhanced;
}

std::vector<Vec4f> detectLineFeatures(Mat gray_image, int line_len)
{
    opts.refine       = 1;  
	opts.scale        = 0.8; 
	opts.sigma_scale  = 1.5;	
	opts.quant        = 2.0;
	opts.ang_th       = 22.5;	
	opts.log_eps      = 0;
	opts.density_th   = 0.6;
	opts.n_bins       = 1024;
	opts.min_length = 0.125;
	cv::Ptr<cv::LineSegmentDetector> ls = cv::createLineSegmentDetector(opts.refine,
	                                                           opts.scale,
	                                                           opts.sigma_scale,
	                                                           opts.quant,
	                                                           opts.ang_th,
	                                                           opts.log_eps,
	                                                           opts.density_th,
	                                                           opts.n_bins);
    std::vector<Vec4f> current_lines;
    std::vector<Vec4f> len_current_lines;
    ls->detect(gray_image, current_lines);
    for (auto line : current_lines) {
        cv::Vec4f cur_line = line;
        float length = cv::norm(cv::Point2f(cur_line[0], cur_line[1]) - cv::Point2f(cur_line[2], cur_line[3]));
        if (length > line_len) {
            len_current_lines.push_back(cur_line);
        }
    }
    return len_current_lines;
}

std::vector<cv::KeyPoint> trackORBFeatures(const cv::Mat gray_image, int point_th) {
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    cv::Ptr<cv::ORB> orb = cv::ORB::create();
    orb->setMaxFeatures(point_th);
    orb->setFastThreshold(15);
    orb->setScaleFactor(1.2);

    orb->detectAndCompute(gray_image, cv::noArray(), keypoints, descriptors);

    return keypoints;
}
cv::Mat get_image_keypoints(cv::Mat gray_image, int line_th, int line_len, int line_wide, int point_th, int radius_size){
	std::vector<cv::Vec4f> lines_ls;
	std::vector<cv::KeyPoint> keypoints;
	cv::cvtColor(gray_image, feather_image, cv::COLOR_GRAY2BGR);
	tbb::parallel_for(tbb::blocked_range<int>(0, 2), [&](const tbb::blocked_range<int>& r) {
	    if (r.begin() == 0) {
	        lines_ls = detectLineFeatures(gray_image, line_len);
	    } 
	    
	    else {
	        keypoints = trackORBFeatures(gray_image, point_th);
	    }
	});

	cv::Scalar lineColor(55, 55, 255);
	cv::Scalar keypointColor(255, 55, 55); 
	// draw line
	int lineThickness = line_wide; 
	for (const auto& line : lines_ls) {
	    cv::Point pt1(line[0], line[1]);
	    cv::Point pt2(line[2], line[3]);
	    cv::line(feather_image, pt1, pt2, lineColor,lineThickness);
	}
	for (const auto& keypoint : keypoints) {
		cv::Point2f point = keypoint.pt;
		int side_length = radius_size * 2;
		cv::Point2f upper_left(point.x - side_length / 2, point.y - side_length / 2);
		cv::Point2f bottom_right(point.x + side_length / 2, point.y + side_length / 2);
		cv::rectangle(feather_image, upper_left, bottom_right, keypointColor, cv::FILLED);
	}
	return feather_image;
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr ConvertToXYZRGBPointCloud(const Vector6dVector& map_cloud) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (const auto& vec : map_cloud) {
        pcl::PointXYZRGB point;
        point.x = static_cast<float>(vec[0]);
        point.y = static_cast<float>(vec[1]);
        point.z = static_cast<float>(vec[2]);

        uint8_t r = static_cast<uint8_t>(vec[3]);
        uint8_t g = static_cast<uint8_t>(vec[4]);
        uint8_t b = static_cast<uint8_t>(vec[5]);
        uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                        static_cast<uint32_t>(g) << 8 |
                        static_cast<uint32_t>(b));
        point.rgb = *reinterpret_cast<float*>(&rgb);

        cloud->push_back(point);
    }
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::RandomSample<pcl::PointXYZRGB> random_sample;
	random_sample.setInputCloud(cloud);
	random_sample.setSample(10000);
	random_sample.filter(*cloud_filtered);
    return cloud_filtered;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "KISS_LV_main");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    KISS_LV * fast_lio_instance = new KISS_LV(nh, nh_private);
    ros::Rate rate(5000);
    ros::spin();
}
