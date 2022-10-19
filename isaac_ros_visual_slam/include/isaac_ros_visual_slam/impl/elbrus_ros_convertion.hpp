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

#ifndef ISAAC_ROS_VISUAL_SLAM__IMPL__ELBRUS_ROS_CONVERTION_HPP_
#define ISAAC_ROS_VISUAL_SLAM__IMPL__ELBRUS_ROS_CONVERTION_HPP_

#include "elbrus.h"  // NOLINT - include .h without directory
#include "tf2/LinearMath/Transform.h"

namespace isaac_ros
{
namespace visual_slam
{

ELBRUS_Pose ToElbrusPose(const tf2::Transform & tf_mat);
tf2::Transform FromElbrusPose(const ELBRUS_Pose & elbrus_pose);

tf2::Transform ChangeBasis(
  const tf2::Transform & target_pose_source,
  const tf2::Transform & source_pose_source);

}  // namespace visual_slam
}  // namespace isaac_ros

#endif  // ISAAC_ROS_VISUAL_SLAM__IMPL__ELBRUS_ROS_CONVERTION_HPP_
