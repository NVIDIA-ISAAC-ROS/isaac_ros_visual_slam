/**
 * Copyright (c) 2021-2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

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
