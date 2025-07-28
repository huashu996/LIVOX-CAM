# **LIVOX-CAM: Adaptive Coarse-to-Fine Visual-assisted LiDAR Odometry for Solid-State LiDAR**

## 📌 What is LIVOX-CAM

LIVOX-CAM is a visual-assisted LiDAR odometry based on KISS-ICP, specifically tailored for small field-of-view (FoV) solid-state LiDAR sensors. The system comprises a two-stage architecture: a front-end module for robust key point extraction and a back-end module implementing coarse-to-fine iterative pose optimization. Our system is capable of operating stably in autonomous driving, unstructured environments, handheld scenarios, degenerate cases, and high-altitude mapping tasks.

### ✅ 1. System overview

<div align="center">
  <img src="kiss_lv/doc/1.png" width="800">
</div>

# Multi-layer local map visualization
<center> 
    ![local_map](kiss_lv/doc/local_map.gif)
</center>

# Support for color dense mapping
<center> 
    ![colored_map](kiss_lv/doc/colored_map.gif)
</center>

### ✅ 2. Dataset Demo

| Dataset | Demo |
|-----------|--------|
| Garden dataset     | ![](kiss_lv/doc/garden.gif) |
| Geode dataset      | ![](kiss_lv/doc/geode.gif) | 
| Tunnel-degraded scenarios      | ![](kiss_lv/doc/geode_de.gif) | 
| M3DGR dataset      | ![](kiss_lv/doc/m3gdr.gif) |
| MARS-LVIG dataset      | ![](kiss_lv/doc/mars.gif) |
| High-altitude degraded scenarios      | ![](kiss_lv/doc/seua.gif) |
| SEU dataset      | ![](kiss_lv/doc/seug.gif) |
| Dark scene      | ![](kiss_lv/doc/dark.gif) |
---

## ⚙️ Install

Recommended System **Ubuntu 20.04 + ROS Noetic**

### 🔧 Dependency

```bash
# 1. Livox SDK
cd livox_sdk/build
cmake ..
make -j8
sudo make install

# 2. fmt
cd fmt/build
cmake ..
make -j8
sudo make install

# 3. Eigen
cd eigen/build
cmake ..
make -j8
sudo make install

# 4. Sophus
sudo apt-get install ros-noetic-sophus

# 5. Ceres Solver
sudo apt-get install libceres-dev

# 6. PCL
sudo apt install libpcl-dev
sudo apt install pcl-tools

# 7. OpenCV
sudo apt install libopencv-dev python3-opencv

# 8. Others
sudo apt-get install ros-noetic-tf2-sensor-msgs
sudo apt-get install ros-noetic-eigen-conversions
sudo apt-get install liboctomap-dev
sudo apt install ros-noetic-octomap ros-noetic-octomap-rviz
```

### 🔧 Build
```bash
git clone 
cd livox_cam
catkin build
source devel/setup.bash
```

### 🔧 Run
After modifying the config file for your environment, you can run Onion-LO. Here is an example to test it with a Livox LiDAR.
```bash
roslaunch kiss-lv livox.launch
rosbag play your_data.bag
```


