/**
 * Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include <string>

#include "isaac_ros_visual_slam/impl/viz_helper.hpp"

namespace isaac_ros
{
namespace visual_slam
{

// VisHelper

VisHelper::VisHelper() {}
void VisHelper::Init(
  ELBRUS_TrackerHandle elbrus_handle,
  const tf2::Transform & base_link_pose_elbrus,
  const rclcpp::Node & node,
  const std::string & frame_id)
{
  elbrus_handle_ = elbrus_handle;
  base_link_pose_elbrus_ = base_link_pose_elbrus;
  node_ = &node;
  frame_id_ = frame_id;
}

void VisHelper::Exit()
{
  if (!elbrus_handle_) {
    return;
  }
  {
    std::unique_lock<std::mutex> locker(mutex_);

    Reset();

    elbrus_handle_ = 0;
    node_ = nullptr;
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
