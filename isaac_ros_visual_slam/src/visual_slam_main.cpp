/**
 * Copyright (c) 2021-2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include <memory>

#include "isaac_ros_visual_slam/visual_slam_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  rclcpp::NodeOptions visual_slam_options;
  visual_slam_options.arguments(
  {
    "--ros-args",
    "-p", "rectified_images:=true",
    "-p", "denoise_input_images:=false",
  });
  auto visual_slam_node = std::make_shared<isaac_ros::visual_slam::VisualSlamNode>(
    visual_slam_options);
  exec.add_node(visual_slam_node);

  exec.spin();

  rclcpp::shutdown();
  return 0;
}
