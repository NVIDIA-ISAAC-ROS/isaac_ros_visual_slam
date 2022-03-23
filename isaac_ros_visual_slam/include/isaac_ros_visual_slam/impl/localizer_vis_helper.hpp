/**
 * Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef ISAAC_ROS_VISUAL_SLAM__IMPL__LOCALIZER_VIS_HELPER_HPP_
#define ISAAC_ROS_VISUAL_SLAM__IMPL__LOCALIZER_VIS_HELPER_HPP_

#include <string>

#include "visualization_msgs/msg/marker_array.hpp"
#include "viz_helper.hpp"

namespace isaac_ros
{
namespace visual_slam
{

// visualize Localizer state
struct LocalizerVisHelper : public VisHelper
{
public:
  LocalizerVisHelper(
    uint32_t max_items_count = 1024,
    uint32_t period_ms = 500
  );
  ~LocalizerVisHelper() override;

  void Init(
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_localizer_probes,
    ELBRUS_TrackerHandle elbrus_handle,
    const tf2::Transform & base_link_pose_elbrus,
    const rclcpp::Node & node,
    const std::string & frame_id);

  void Reset() override;

  void Run();

  void SetResult(bool succesed, const tf2::Transform & pose);

protected:
  uint32_t max_items_count_ = 1024;
  uint32_t period_ms_ = 500;

  uint64_t last_timestamp_ns_ = 0;
  uint64_t last_num_probes_ = 0;
  bool reset_required_ = false;

  bool localization_finished_ = false;
  bool have_result_pose_ = false;
  tf2::Transform result_pose_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_localizer_probes_;
};

}  // namespace visual_slam
}  // namespace isaac_ros

#endif  // ISAAC_ROS_VISUAL_SLAM__IMPL__LOCALIZER_VIS_HELPER_HPP_
