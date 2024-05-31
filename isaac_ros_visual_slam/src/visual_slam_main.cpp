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
  auto visual_slam_node = std::make_shared<nvidia::isaac_ros::visual_slam::VisualSlamNode>(
    visual_slam_options);
  exec.add_node(visual_slam_node);

  exec.spin();

  rclcpp::shutdown();
  return 0;
}
