/**
 * Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef ISAAC_ROS_VISUAL_SLAM__IMPL__POSE_CACHE_HPP_
#define ISAAC_ROS_VISUAL_SLAM__IMPL__POSE_CACHE_HPP_

#include <array>
#include <list>
#include <utility>

#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Quaternion.h"

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

#endif  // ISAAC_ROS_VISUAL_SLAM__IMPL__POSE_CACHE_HPP_
