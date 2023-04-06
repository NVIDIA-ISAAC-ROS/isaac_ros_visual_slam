# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

cmake_minimum_required(VERSION 3.23.2)
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

# Elbrus (resolve path from ament_index)
ament_index_get_resource(ELBRUS_RELATIVE_PATH elbrus isaac_ros_nitros)
ament_index_get_prefix_path(AMENT_INDEX_PREFIX_PATHS)
foreach(PREFIX_PATH IN LISTS AMENT_INDEX_PREFIX_PATHS)
    if(EXISTS "${PREFIX_PATH}/${ELBRUS_RELATIVE_PATH}")
      set(ELBRUS "${PREFIX_PATH}/${ELBRUS_RELATIVE_PATH}")
      break()
    endif()
endforeach()

include_directories(${ELBRUS}/include)
add_library(elbrus SHARED IMPORTED)
if( ${ARCHITECTURE} STREQUAL "x86_64" )
  set_property(TARGET elbrus PROPERTY IMPORTED_LOCATION ${ELBRUS}/lib_x86_64_cuda_11_8/libelbrus.so)
elseif( ${ARCHITECTURE} STREQUAL "aarch64" )
  set_property(TARGET elbrus PROPERTY IMPORTED_LOCATION ${ELBRUS}/lib_aarch64_jetpack51/libelbrus.so)
endif()

# visual_slam_node
ament_auto_add_library(
  visual_slam_node SHARED
  src/visual_slam_node.cpp
  src/impl/elbrus_ros_convertion.cpp
  src/impl/landmarks_vis_helper.cpp
  src/impl/localizer_vis_helper.cpp
  src/impl/pose_cache.cpp
  src/impl/posegraph_vis_helper.cpp
  src/impl/visual_slam_impl.cpp
  src/impl/viz_helper.cpp
  src/impl/qos.cpp
)
target_link_libraries(visual_slam_node elbrus)
rclcpp_components_register_nodes(visual_slam_node "isaac_ros::visual_slam::VisualSlamNode")
set(node_plugins "${node_plugins}isaac_ros::visual_slam::VisualSlamNode;$<TARGET_FILE:visual_slam_node>\n")

# isaac_ros_visual_slam executable
ament_auto_add_executable(${PROJECT_NAME}
  src/visual_slam_main.cpp
)
target_link_libraries(${PROJECT_NAME} visual_slam_node)

# Install Elbrus
if( ${ARCHITECTURE} STREQUAL "x86_64" )
  install(DIRECTORY ${ELBRUS}/lib_x86_64_cuda_11_8/ DESTINATION share/elbrus)
elseif( ${ARCHITECTURE} STREQUAL "aarch64" )
  install(DIRECTORY ${ELBRUS}/lib_aarch64_jetpack51/ DESTINATION share/elbrus)
endif()
set_target_properties(visual_slam_node PROPERTIES BUILD_WITH_INSTALL_RPATH TRUE)
set_target_properties(visual_slam_node PROPERTIES INSTALL_RPATH "$ORIGIN/../share/elbrus")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/isaac_ros_visual_slam_pol.py)
endif()

ament_auto_package(INSTALL_TO_SHARE launch params)
