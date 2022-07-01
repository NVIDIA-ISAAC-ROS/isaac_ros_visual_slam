/**
 * Copyright (c) 2021-2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef ISAAC_ROS_VISUAL_SLAM__IMPL__HAS_SUBSCRIBERS_HPP_
#define ISAAC_ROS_VISUAL_SLAM__IMPL__HAS_SUBSCRIBERS_HPP_

#include <memory>

#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"

namespace isaac_ros
{
namespace visual_slam
{
template<class T>
bool HasSubscribers(
  const rclcpp::Node & node,
  const std::shared_ptr<rclcpp::Publisher<T>> & publisher)
{
  if (!publisher) {
    return false;
  }
  try {
    size_t subscribers = 0;
    subscribers = node.count_subscribers(publisher->get_topic_name());
    return subscribers != 0;
  } catch (...) {
    rcutils_reset_error();
    RCLCPP_DEBUG(node.get_logger(), "HasSubscribers(): Exception while counting subscribers");
  }
  return false;
}

}  // namespace visual_slam
}  // namespace isaac_ros

#endif  // ISAAC_ROS_VISUAL_SLAM__IMPL__HAS_SUBSCRIBERS_HPP_
