// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2021-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#ifndef ISAAC_ROS_VISUAL_SLAM__IMPL__TYPES_HPP_
#define ISAAC_ROS_VISUAL_SLAM__IMPL__TYPES_HPP_

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "isaac_ros_visual_slam_interfaces/action/load_map_and_localize.hpp"
#include "isaac_ros_visual_slam_interfaces/action/save_map.hpp"
#include "isaac_ros_visual_slam_interfaces/msg/visual_slam_status.hpp"
#include "isaac_ros_visual_slam_interfaces/srv/get_all_poses.hpp"
#include "isaac_ros_visual_slam_interfaces/srv/reset.hpp"
#include "isaac_ros_visual_slam_interfaces/srv/set_odometry_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using ImageType = sensor_msgs::msg::Image;
using CameraInfoType = sensor_msgs::msg::CameraInfo;
using ImuType = sensor_msgs::msg::Imu;
using PointCloud2Type = sensor_msgs::msg::PointCloud2;

using PoseType = geometry_msgs::msg::Pose;
using PoseStampedType = geometry_msgs::msg::PoseStamped;
using PoseWithCovarianceStampedType = geometry_msgs::msg::PoseWithCovarianceStamped;
using PoseArrayType = geometry_msgs::msg::PoseArray;

using MarkerType = visualization_msgs::msg::Marker;
using MarkerArrayType = visualization_msgs::msg::MarkerArray;

using OdometryType = nav_msgs::msg::Odometry;
using PathType = nav_msgs::msg::Path;

using VisualSlamStatusType = isaac_ros_visual_slam_interfaces::msg::VisualSlamStatus;

using SrvReset = isaac_ros_visual_slam_interfaces::srv::Reset;
using SrvGetAllPoses = isaac_ros_visual_slam_interfaces::srv::GetAllPoses;
using SrvSetOdometryPose = isaac_ros_visual_slam_interfaces::srv::SetOdometryPose;

using ActionSaveMap = isaac_ros_visual_slam_interfaces::action::SaveMap;
using ActionLoadMapAndLocalize = isaac_ros_visual_slam_interfaces::action::LoadMapAndLocalize;

#endif  // ISAAC_ROS_VISUAL_SLAM__IMPL__TYPES_HPP_
