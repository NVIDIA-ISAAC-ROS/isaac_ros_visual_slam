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

#ifndef ISAAC_ROS_VISUAL_SLAM__IMPL__POSE_CACHE_HPP_
#define ISAAC_ROS_VISUAL_SLAM__IMPL__POSE_CACHE_HPP_

#include <array>
#include <list>
#include <utility>

#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"

namespace nvidia
{
namespace isaac_ros
{
namespace visual_slam
{
class PoseCache
{
public:
  void Reset();
  void Add(int64_t timestamp, const tf2::Transform & pose);

  bool GetVelocity(
    double & x, double & y, double & z,
    double & roll, double & pitch, double & yaw) const;
  bool GetCovariance(std::array<double, 6 * 6> & cov) const;

protected:
  const size_t num_poses_to_keep_ = 10;
  std::list<std::pair<int64_t, tf2::Transform>> poses_;
};

class VelocityCache
{
public:
  void Reset();
  void Add(
    const double & x, const double & y, const double & z, const double & roll,
    const double & pitch, const double & yaw);

  bool GetCovariance(std::array<double, 6 * 6> & cov) const;

private:
  const size_t num_velocities_to_keep_ = 10;
  std::list<std::array<double, 6>> velocities_;
};

}  // namespace visual_slam
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_VISUAL_SLAM__IMPL__POSE_CACHE_HPP_
