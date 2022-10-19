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
