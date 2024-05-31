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

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Dependencies
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# cuVSLAM (resolve path from ament_index)
ament_index_get_resource(CUVSLAM_RELATIVE_PATH cuvslam isaac_ros_nitros)
ament_index_get_prefix_path(AMENT_INDEX_PREFIX_PATHS)
foreach(PREFIX_PATH IN LISTS AMENT_INDEX_PREFIX_PATHS)
    if(EXISTS "${PREFIX_PATH}/${CUVSLAM_RELATIVE_PATH}")
      set(CUVSLAM "${PREFIX_PATH}/${CUVSLAM_RELATIVE_PATH}")
      break()
    endif()
endforeach()

include_directories(${CUVSLAM}/include)
add_library(cuvslam SHARED IMPORTED)
set_property(TARGET cuvslam PROPERTY IMPORTED_LOCATION ${CUVSLAM}/lib/libcuvslam.so)

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
target_link_libraries(visual_slam_node cuvslam)
rclcpp_components_register_nodes(visual_slam_node "nvidia::isaac_ros::visual_slam::VisualSlamNode")
set(node_plugins "${node_plugins}nvidia::isaac_ros::visual_slam::VisualSlamNode;$<TARGET_FILE:visual_slam_node>\n")

# isaac_ros_visual_slam executable
ament_auto_add_executable(${PROJECT_NAME}
  src/visual_slam_main.cpp
)
target_link_libraries(${PROJECT_NAME} visual_slam_node)

# Install cuVSLAM
install(DIRECTORY ${CUVSLAM}/lib DESTINATION share/cuvslam)

set_target_properties(visual_slam_node PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  BUILD_RPATH_USE_ORIGIN TRUE
  INSTALL_RPATH_USE_LINK_PATH TRUE
  INSTALL_RPATH "$ORIGIN/../share/cuvslam")


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # The FindPythonInterp and FindPythonLibs modules are removed
  if(POLICY CMP0148)
    cmake_policy(SET CMP0148 OLD)
  endif()

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/isaac_ros_visual_slam_pol_single_cam_imu.py)
  add_launch_test(test/isaac_ros_visual_slam_pol_multi_cam_imu.py)
  add_launch_test(test/isaac_ros_visual_slam_srv_reset.py)
  add_launch_test(test/isaac_ros_visual_slam_srv_get_all_poses.py)
  add_launch_test(test/isaac_ros_visual_slam_srv_set_slam_pose.py)

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test_message_buffer test/test_message_buffer.cpp)
  target_include_directories(${PROJECT_NAME}_test_message_buffer PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}_test_message_buffer
    std_msgs
  )

  ament_add_gtest(${PROJECT_NAME}_test_message_stream_sequencer test/test_message_stream_sequencer.cpp)
  target_include_directories(${PROJECT_NAME}_test_message_stream_sequencer PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}_test_message_stream_sequencer
    std_msgs
  )

  ament_add_gtest(${PROJECT_NAME}_test_message_stream_synchronizer
    test/test_message_stream_synchronizer.cpp
  )
  target_include_directories(${PROJECT_NAME}_test_message_stream_synchronizer PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}_test_message_stream_synchronizer
    rclcpp
    geometry_msgs
  )
endif()

ament_auto_package(INSTALL_TO_SHARE config launch rviz)
