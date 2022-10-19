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

#ifndef ISAAC_ROS_VISUAL_SLAM__IMPL__VIZ_HELPER_HPP_
#define ISAAC_ROS_VISUAL_SLAM__IMPL__VIZ_HELPER_HPP_

#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

#include "elbrus.h"  // NOLINT - include .h without directory
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Transform.h"

namespace isaac_ros
{
namespace visual_slam
{

// Base class for async helpers
class VisHelper
{
public:
  VisHelper();
  virtual ~VisHelper() {}

  void Init(
    ELBRUS_TrackerHandle elbrus_handle,
    const tf2::Transform & base_link_pose_elbrus,
    const rclcpp::Node & node,
    const std::string & frame_id
  );

  void Exit();

protected:
  virtual void Reset() = 0;

protected:
  ELBRUS_TrackerHandle elbrus_handle_ = nullptr;
  tf2::Transform base_link_pose_elbrus_;
  const rclcpp::Node * node_ = nullptr;
  std::string frame_id_;

  std::thread thread_;
  std::mutex mutex_;
  std::condition_variable cond_var_;
};

}  // namespace visual_slam
}  // namespace isaac_ros

#endif  // ISAAC_ROS_VISUAL_SLAM__IMPL__VIZ_HELPER_HPP_
