/**
 * Copyright (c) 2021-2022, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef ISAAC_ROS_VISUAL_SLAM__IMPL__POSEGRAPH_VIS_HELPER_HPP_
#define ISAAC_ROS_VISUAL_SLAM__IMPL__POSEGRAPH_VIS_HELPER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_array.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "viz_helper.hpp"

namespace isaac_ros
{
namespace visual_slam
{

// visualize Pose Graph
struct PoseGraphVisHelper : public VisHelper
{
public:
  PoseGraphVisHelper(
    ELBRUS_DataLayer layer,
    uint32_t max_items_count = 1024,
    uint32_t period_ms = 500
  );
  ~PoseGraphVisHelper() override;

  void Init(
    const rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_nodes,
    const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_edges,
    const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_edges2,
    ELBRUS_TrackerHandle elbrus_handle,
    const tf2::Transform & base_link_pose_elbrus,
    const rclcpp::Node & node,
    const std::string & frame_id);

  void Reset() override;

  void Run();

protected:
  ELBRUS_DataLayer layer_ = LL_POSE_GRAPH;
  uint32_t max_items_count_ = 1024;
  uint32_t period_ms_ = 500;

  uint64_t last_timestamp_ns_ = 0;

  using nodes_msg_t = std::unique_ptr<geometry_msgs::msg::PoseArray>;

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_nodes_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_edges_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_edges2_;
};

}  // namespace visual_slam
}  // namespace isaac_ros

#endif  // ISAAC_ROS_VISUAL_SLAM__IMPL__POSEGRAPH_VIS_HELPER_HPP_
