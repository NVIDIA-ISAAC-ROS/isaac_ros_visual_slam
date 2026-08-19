# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.22.1)
project(isaac_ros_visual_slam LANGUAGES C CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

execute_process(COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE)
message( STATUS "Architecture: ${ARCHITECTURE}" )

find_package(Boost COMPONENTS thread chrono REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
include(ExternalProject)

# Ensure CUDA imported targets (e.g., CUDA::nvtx3) are available
find_package(CUDAToolkit REQUIRED)
find_package(isaac_ros_common REQUIRED)
find_package(isaac_ros_nitros REQUIRED)
find_package(isaac_ros_nitros_image_type REQUIRED)
find_package(isaac_ros_visual_slam_interfaces REQUIRED)
find_library(ISAAC_ROS_COMMON_LIBRARY isaac_ros_common REQUIRED)
find_library(ISAAC_ROS_NITROS_LIBRARY isaac_ros_nitros REQUIRED)
find_library(ISAAC_ROS_NITROS_IMAGE_TYPE_LIBRARY
  isaac_ros_nitros_image_type REQUIRED)
find_library(ISAAC_ROS_VISUAL_SLAM_INTERFACES_TYPESUPPORT_CPP_LIBRARY
  isaac_ros_visual_slam_interfaces__rosidl_typesupport_cpp REQUIRED)
get_filename_component(CUVSLAM_CUDA_ROOT "${CUDAToolkit_BIN_DIR}" DIRECTORY)

# NVTX
option(USE_NVTX "Enable NVTX markers for improved profiling (if available)" ON)
if(USE_NVTX)
    add_definitions(-DUSE_NVTX)
    link_directories("${CUDA_TOOLKIT_ROOT_DIR}/lib64")
    link_libraries("CUDA::nvtx3")
endif()

# Dependencies
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${isaac_common_INCLUDE_DIRS})

set(VISUAL_SLAM_NITROS_LIBRARIES
  CUDA::cudart
  ${ISAAC_ROS_COMMON_LIBRARY}
  ${ISAAC_ROS_NITROS_LIBRARY}
  ${ISAAC_ROS_NITROS_IMAGE_TYPE_LIBRARY}
  ${ISAAC_ROS_VISUAL_SLAM_INTERFACES_TYPESUPPORT_CPP_LIBRARY}
)

get_filename_component(CUVSLAM_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../cuvslam" REALPATH)
if(NOT EXISTS "${CUVSLAM_SOURCE_DIR}/CMakeLists.txt")
  message(FATAL_ERROR
    "cuVSLAM source checkout missing at ${CUVSLAM_SOURCE_DIR}. "
    "Run `git submodule update --init --recursive "
    "ros_ws/src/isaac_ros_visual_slam/cuvslam`.")
endif()

if(CMAKE_BUILD_TYPE)
  set(CUVSLAM_BUILD_TYPE "${CMAKE_BUILD_TYPE}")
else()
  set(CUVSLAM_BUILD_TYPE Release)
endif()

set(CUVSLAM_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}/cuvslam")
set(CUVSLAM_LIBRARY "${CUVSLAM_BINARY_DIR}/bin/libcuvslam.so")
set(CUVSLAM_API_LAUNCHER "${CUVSLAM_BINARY_DIR}/bin/cuvslam_api_launcher")
set(CUVSLAM_GIT_SAFE_DIR_ENV
  GIT_CONFIG_COUNT=1
  GIT_CONFIG_KEY_0=safe.directory
  GIT_CONFIG_VALUE_0=${CUVSLAM_SOURCE_DIR}
)

externalproject_add(cuvslam_external
  SOURCE_DIR "${CUVSLAM_SOURCE_DIR}"
  BINARY_DIR "${CUVSLAM_BINARY_DIR}"
  CMAKE_ARGS
    -DCMAKE_BUILD_TYPE=${CUVSLAM_BUILD_TYPE}
    -DCMAKE_INSTALL_PREFIX=${CUVSLAM_BINARY_DIR}/install
    -DCUDAToolkit_ROOT=${CUVSLAM_CUDA_ROOT}
    -DCUVSLAM_BUILD_SHARED_LIB=ON
    -DUSE_CUDA=ON
    -DUSE_NVTX=${USE_NVTX}
    -DUSE_RERUN=OFF
    -DUSE_CERES=OFF
    -DUSE_CUNLS=OFF
  BUILD_COMMAND ${CMAKE_COMMAND} -E env
    ${CUVSLAM_GIT_SAFE_DIR_ENV}
    ${CMAKE_COMMAND} --build <BINARY_DIR> --target cuvslam_api_launcher
  INSTALL_COMMAND ""
  BUILD_BYPRODUCTS
    "${CUVSLAM_LIBRARY}"
    "${CUVSLAM_API_LAUNCHER}"
)

add_library(cuvslam SHARED IMPORTED GLOBAL)
set_target_properties(cuvslam PROPERTIES
  IMPORTED_LOCATION "${CUVSLAM_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${CUVSLAM_SOURCE_DIR}/libs"
)
add_dependencies(cuvslam cuvslam_external)

# visual_slam_node
ament_auto_add_library(
  visual_slam_node SHARED
  src/visual_slam_node.cpp
  src/impl/cuvslam_ros_conversion.cpp
  src/impl/landmarks_vis_helper.cpp
  src/impl/localizer_vis_helper.cpp
  src/impl/pose_cache.cpp
  src/impl/posegraph_vis_helper.cpp
  src/impl/visual_slam_impl.cpp
  src/impl/viz_helper.cpp
)
target_link_libraries(visual_slam_node
  ${VISUAL_SLAM_NITROS_LIBRARIES}
  cuvslam
  Boost::thread
  Boost::chrono
)
add_dependencies(visual_slam_node cuvslam_external)
rclcpp_components_register_nodes(visual_slam_node "nvidia::isaac_ros::visual_slam::VisualSlamNode")
set(node_plugins "${node_plugins}nvidia::isaac_ros::visual_slam::VisualSlamNode;$<TARGET_FILE:visual_slam_node>\n")

# isaac_ros_visual_slam executable
ament_auto_add_executable(${PROJECT_NAME}
  src/visual_slam_main.cpp
)
target_link_libraries(${PROJECT_NAME}
  visual_slam_node
  ${VISUAL_SLAM_NITROS_LIBRARIES}
)

# API launcher executable
install(PROGRAMS
  ${CUVSLAM_API_LAUNCHER}
  DESTINATION lib/${PROJECT_NAME}
)

# Install cuVSLAM
install(FILES ${CUVSLAM_LIBRARY} DESTINATION lib)

set_target_properties(visual_slam_node PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  BUILD_RPATH_USE_ORIGIN TRUE
  INSTALL_RPATH_USE_LINK_PATH TRUE)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # The FindPythonInterp and FindPythonLibs modules are removed
  if(POLICY CMP0148)
    cmake_policy(SET CMP0148 OLD)
  endif()

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/isaac_ros_visual_slam_pol_multi_cam_imu.py)
  add_launch_test(test/isaac_ros_visual_slam_pol_rgbd_cam.py)
  add_launch_test(test/isaac_ros_visual_slam_pol_single_cam_imu.py)
  add_launch_test(test/isaac_ros_visual_slam_srv_get_all_poses.py)
  add_launch_test(test/isaac_ros_visual_slam_srv_load_map.py)
  # TODO(lgulich): Enable this test when we have better test data.
  # add_launch_test(test/isaac_ros_visual_slam_srv_localize_in_map.py)
  add_launch_test(test/isaac_ros_visual_slam_srv_reset.py)
  add_launch_test(test/isaac_ros_visual_slam_srv_save_map.py)
  add_launch_test(test/isaac_ros_visual_slam_srv_set_slam_pose.py)

  ament_add_gtest(${PROJECT_NAME}_test_message_stream_sequencer test/test_message_stream_sequencer.cpp)
  target_include_directories(${PROJECT_NAME}_test_message_stream_sequencer PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}_test_message_stream_sequencer
    std_msgs
  )
endif()


# Embed versioning information into installed files
ament_index_get_resource(ISAAC_ROS_COMMON_CMAKE_PATH isaac_ros_common_cmake_path isaac_ros_common)
include("${ISAAC_ROS_COMMON_CMAKE_PATH}/isaac_ros_common-version-info.cmake")
generate_version_info(${PROJECT_NAME})

ament_auto_package(INSTALL_TO_SHARE config launch rviz)
