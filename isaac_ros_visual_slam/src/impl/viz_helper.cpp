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

#include <string>

#include "isaac_ros_visual_slam/impl/viz_helper.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace visual_slam
{

// VisHelper
VisHelper::VisHelper()
: logger_(rclcpp::get_logger("VisualSlam: VisHelper"))
{}

void VisHelper::Init(
  std::shared_ptr<cuvslam::Slam> & cuvslam_slam,
  const tf2::Transform & canonical_pose_cuvslam,
  const std::string & frame_id,
  const rclcpp::Logger & logger)
{
  cuvslam_slam_ = cuvslam_slam;
  canonical_pose_cuvslam_ = canonical_pose_cuvslam;
  frame_id_ = frame_id;
  logger_ = logger;
}

void VisHelper::Exit()
{
  if (!cuvslam_slam_) {
    return;
  }
  {
    std::unique_lock<std::mutex> locker(mutex_);

    Reset();

    cuvslam_slam_.reset();
    frame_id_ = "";

    cond_var_.notify_all();
  }
  try {
    if (thread_.joinable()) {
      thread_.join();
    }
  } catch (std::system_error & e) {
  }
}

}  // namespace visual_slam
}  // namespace isaac_ros
}  // namespace nvidia
