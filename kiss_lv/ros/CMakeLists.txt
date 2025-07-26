# MIT License
#
# Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
# Stachniss.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
cmake_minimum_required(VERSION 2.8.3)
project(kiss_lv)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -std=c++14 -O3 -lboost_system -msse2 -msse3 -pthread -Wenum-compare") # -Wall
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../cpp/kiss_lv/)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../cpp/kiss_lv ${CMAKE_CURRENT_BINARY_DIR}/kiss_lv)
else()
  cmake_minimum_required(VERSION 3.18)
  message(STATUS "Performing out-of-tree build, fetching KISS-ICP v${CMAKE_PROJECT_VERSION} Release from Github")
  include(FetchContent)
  FetchContent_Declare(
    ext_kiss_lv_core PREFIX kiss_lv_core
    URL https://github.com/PRBonn/kiss-icp/archive/refs/tags/v${CMAKE_PROJECT_VERSION}.tar.gz SOURCE_SUBDIR
        cpp/kiss_lv)
  FetchContent_MakeAvailable(ext_kiss_lv_core)
endif()

if("$ENV{ROS_VERSION}" STREQUAL "1")
  message(STATUS "KISS-ICP-ROS1 wrapper will be compiled")
  find_package(
    catkin REQUIRED
    COMPONENTS geometry_msgs
    		   livox_ros_driver
    		   cv_bridge
               nav_msgs
               sensor_msgs
               roscpp
               rosbag
               std_msgs
               pcl_conversions
               tf2
               tf2_ros)
  catkin_package()
  include_directories(
	  ${catkin_INCLUDE_DIRS}
	  ${EIGEN3_INCLUDE_DIR}
	  ${PCL_INCLUDE_DIRS}
	  ${livox_ros_driver_INCLUDE_DIRS}
	  ./include/
  )
  # ROS1 node
  
  add_executable(odometry_node ros1/kiss_lv.cpp)
  target_include_directories(odometry_node PUBLIC include ${catkin_INCLUDE_DIRS})
  target_link_libraries(odometry_node kiss_lv::pipeline ${catkin_LIBRARIES})
  install(TARGETS odometry_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

else()
  message(FATAL_ERROR "catkin or colcon not found KISS-ICP-ROS disabled")
endif()
