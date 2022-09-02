/**
 * Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef ISAAC_ROS_VISUAL_SLAM__IMPL__QOS_HPP_
#define ISAAC_ROS_VISUAL_SLAM__IMPL__QOS_HPP_

#include <string>

#include <rclcpp/rclcpp.hpp>

namespace isaac_ros
{
namespace visual_slam
{

rmw_qos_profile_t parseQosString(const std::string & str);

}  // namespace visual_slam
}  // namespace isaac_ros

#endif  // ISAAC_ROS_VISUAL_SLAM__IMPL__QOS_HPP_
