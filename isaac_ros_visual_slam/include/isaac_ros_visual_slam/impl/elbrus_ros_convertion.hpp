/**
 * Copyright (c) 2021-2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

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
