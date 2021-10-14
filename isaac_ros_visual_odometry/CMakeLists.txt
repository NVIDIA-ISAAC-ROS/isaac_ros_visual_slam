# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

cmake_minimum_required(VERSION 3.5)
project(isaac_ros_visual_odometry LANGUAGES C CXX)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

execute_process(COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE)
message( STATUS "Architecture: ${ARCHITECTURE}" )

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Eigen
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Elbrus
include_directories(elbrus/include)
add_library(elbrus SHARED IMPORTED)
if( ${ARCHITECTURE} STREQUAL "x86_64" )
  set_property(TARGET elbrus PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/elbrus/lib_x86_64/libelbrus.so)
elseif( ${ARCHITECTURE} STREQUAL "aarch64" )
  set_property(TARGET elbrus PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/elbrus/lib_aarch64_xavier/libelbrus.so)
endif()

# visual_odometry_node
ament_auto_add_library(visual_odometry_node SHARED src/visual_odometry_node.cpp)
target_compile_definitions(visual_odometry_node
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
target_link_libraries(visual_odometry_node elbrus)
rclcpp_components_register_nodes(visual_odometry_node "isaac_ros::visual_odometry::VisualOdometryNode")
set(node_plugins "${node_plugins}isaac_ros::visual_odometry::VisualOdometryNode;$<TARGET_FILE:visual_odometry_node>\n")

# isaac_ros_visual_odometry executable
ament_auto_add_executable(${PROJECT_NAME}
  src/visual_odometry_main.cpp
)
target_link_libraries(${PROJECT_NAME} visual_odometry_node)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install Elbrus
if( ${ARCHITECTURE} STREQUAL "x86_64" )
  install(DIRECTORY elbrus/lib_x86_64/ DESTINATION share/elbrus)
elseif( ${ARCHITECTURE} STREQUAL "aarch64" )
  install(DIRECTORY elbrus/lib_aarch64_xavier/ DESTINATION share/elbrus)
endif()
set_target_properties(visual_odometry_node PROPERTIES BUILD_WITH_INSTALL_RPATH TRUE)
set_target_properties(visual_odometry_node PROPERTIES INSTALL_RPATH "$ORIGIN/../share/elbrus")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # Ignore copyright notices since we use custom JetPack EULA
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/isaac_ros_visual_odometry_pol.py)
endif()

ament_auto_package(INSTALL_TO_SHARE launch)
