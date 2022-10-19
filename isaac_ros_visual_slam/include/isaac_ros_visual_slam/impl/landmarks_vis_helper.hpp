// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2021-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef ISAAC_ROS_VISUAL_SLAM__IMPL__LANDMARKS_VIS_HELPER_HPP_
#define ISAAC_ROS_VISUAL_SLAM__IMPL__LANDMARKS_VIS_HELPER_HPP_

#include <memory>
#include <string>

#include "viz_helper.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace isaac_ros
{
namespace visual_slam
{

// visualize Landmarks
struct LandmarksVisHelper : public VisHelper
{
public:
  enum ColorMode
  {
    CM_RGB_MODE = 0,
    CM_BW_MODE = 1,
    CM_RED_MODE = 2,
    CM_GREEN_MODE = 3,
    CM_WEIGHT_BW_MODE = 4,
  };

  LandmarksVisHelper(
    ELBRUS_DataLayer layer,
    uint32_t max_landmarks_count = 1024,
    ColorMode color_mode = CM_RGB_MODE, uint32_t period_ms = 500);

  ~LandmarksVisHelper() override;

  void Init(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher,
    ELBRUS_TrackerHandle elbrus_handle,
    const tf2::Transform & base_link_pose_elbrus,
    const rclcpp::Node & node,
    const std::string & frame_id
  );

  void Reset() override;

  void Run();

protected:
  ELBRUS_DataLayer layer_ = LL_OBSERVATIONS;
  uint32_t max_landmarks_count_ = 1024;

  uint64_t last_timestamp_ns_ = 0;

  ColorMode color_mode_ = CM_RGB_MODE;

  uint32_t period_ms_ = 500;

  // Point cloud to visualize tracks
  using pointcloud_msg_t = std::unique_ptr<sensor_msgs::msg::PointCloud2>;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

}  // namespace visual_slam
}  // namespace isaac_ros

#endif  // ISAAC_ROS_VISUAL_SLAM__IMPL__LANDMARKS_VIS_HELPER_HPP_
