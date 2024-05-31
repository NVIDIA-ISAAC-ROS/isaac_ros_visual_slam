// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2021-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#include <memory>
#include <string>
#include <random>
#include <utility>

#include "isaac_ros_visual_slam/impl/has_subscribers.hpp"
#include "isaac_ros_visual_slam/impl/landmarks_vis_helper.hpp"
#include "isaac_ros_visual_slam/impl/types.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace visual_slam
{

LandmarksVisHelper::LandmarksVisHelper(
  CUVSLAM_DataLayer layer,
  uint32_t max_landmarks_count,
  LandmarksVisHelper::ColorMode color_mode,
  uint32_t period_ms)
: layer_(layer), max_landmarks_count_(max_landmarks_count), color_mode_(color_mode), period_ms_(
    period_ms) {}

LandmarksVisHelper::~LandmarksVisHelper() {Exit();}

void LandmarksVisHelper::Init(
  const rclcpp::Publisher<PointCloud2Type>::SharedPtr publisher,
  CUVSLAM_TrackerHandle cuvslam_handle,
  const tf2::Transform & canonical_pose_cuvslam,
  const std::string & frame_id
)
{
  publisher_ = publisher;
  VisHelper::Init(cuvslam_handle, canonical_pose_cuvslam, frame_id);
  thread_ = std::thread{&LandmarksVisHelper::Run, this};
}

void LandmarksVisHelper::Reset()
{
  CUVSLAM_DisableReadingDataLayer(cuvslam_handle_, layer_);
  publisher_.reset();
}

void LandmarksVisHelper::Run()
{
  std::unique_lock<std::mutex> locker(mutex_);
  // cuvslam_handle_ will be null after Exit() was called
  while (cuvslam_handle_) {
    cond_var_.wait_for(
      locker,
      std::chrono::milliseconds(period_ms_));
    if (!cuvslam_handle_) {break;}
    if (!rclcpp::ok()) {break;}
    if (!HasSubscribers(publisher_)) {
      // no subcribers so disable reading
      CUVSLAM_DisableReadingDataLayer(cuvslam_handle_, layer_);
      continue;
    }

    // enable reading
    if (CUVSLAM_EnableReadingDataLayer(
        cuvslam_handle_, layer_,
        max_landmarks_count_) != CUVSLAM_SUCCESS) {continue;}

    // read data
    CUVSLAM_LandmarkInfoArrayRef landmarks;
    if (CUVSLAM_StartReadingLandmarks(cuvslam_handle_, layer_, &landmarks) != CUVSLAM_SUCCESS) {
      continue;
    }
    if (last_timestamp_ns_ == landmarks.timestamp_ns) {
      // don't need to publish now
      CUVSLAM_FinishReadingLandmarks(cuvslam_handle_, layer_);
      continue;
    }
    last_timestamp_ns_ = landmarks.timestamp_ns;

    // publish points
    pointcloud_msg_t pc_msg = std::make_unique<PointCloud2Type>();

    pc_msg->header.frame_id = frame_id_;
    pc_msg->header.stamp = rclcpp::Time(landmarks.timestamp_ns, RCL_SYSTEM_TIME);
    pc_msg->height = 1;
    pc_msg->width = landmarks.num;

    pc_msg->is_bigendian = false;
    pc_msg->is_dense = false;

    sensor_msgs::PointCloud2Modifier modifier(*pc_msg.get());
    modifier.setPointCloud2Fields(
      4,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "rgb", 1, sensor_msgs::msg::PointField::UINT32);
    modifier.resize(landmarks.num);

    sensor_msgs::PointCloud2Iterator<float> iter_x(*pc_msg.get(), "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*pc_msg.get(), "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*pc_msg.get(), "z");
    sensor_msgs::PointCloud2Iterator<uint32_t> iter_rgb(*pc_msg.get(), "rgb");

    for (uint32_t i = 0; i < landmarks.num; i++, ++iter_x, ++iter_y, ++iter_z, ++iter_rgb) {
      float w = landmarks.landmarks[i].weight;
      tf2::Vector3 xyz(landmarks.landmarks[i].x, landmarks.landmarks[i].y,
        landmarks.landmarks[i].z);
      xyz = canonical_pose_cuvslam_ * xyz;

      *iter_x = xyz[0];
      *iter_y = xyz[1];
      *iter_z = xyz[2];

      std::mt19937 generator(landmarks.landmarks[i].id);
      std::uniform_int_distribution<int> distribution(92, 255);
      uint32_t rgb = 0xFF000000;
      switch (color_mode_) {
        case CM_RGB_MODE:
          {
            rgb |=
              (distribution(generator) << 16) +
              (distribution(generator) << 8) +
              distribution(generator);
            break;
          }
        case CM_BW_MODE:
          {
            int bw = distribution(generator);
            rgb |= (bw << 16) + (bw << 8) + bw;
            break;
          }
        case CM_RED_MODE:
          {
            int r = 255;
            rgb |= r << 16;
            break;
          }
        case CM_GREEN_MODE:
          {
            int green = distribution(generator);
            rgb |= (42 << 16) + (green << 8) + 42;
            break;
          }
        case CM_WEIGHT_BW_MODE:
          {
            int bw = w * 255;
            rgb |= (bw << 16) + (bw << 8) + bw;
            break;
          }
      }
      *iter_rgb = rgb;
    }
    CUVSLAM_FinishReadingLandmarks(cuvslam_handle_, layer_);

    // Pointcloud publishing
    publisher_->publish(std::move(pc_msg));
  }
}

}  // namespace visual_slam
}  // namespace isaac_ros
}  // namespace nvidia
