/**
 * Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef ISAAC_ROS_VISUAL_SLAM__IMPL__LANDMARKS_VIS_HELPER_HPP_
#define ISAAC_ROS_VISUAL_SLAM__IMPL__LANDMARKS_VIS_HELPER_HPP_

#include <memory>
#include <string>

#include "viz_helper.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace isaac_ros
{
namespace visual_slam
{

// visualize Landmarks
struct LandmarksVisHelper : public VisHelper
{
public:
  enum ColorMode
  {
    CM_RGB_MODE = 0,
    CM_BW_MODE = 1,
    CM_RED_MODE = 2,
    CM_GREEN_MODE = 3,
    CM_WEIGHT_BW_MODE = 4,
  };

  LandmarksVisHelper(
    ELBRUS_DataLayer layer,
    uint32_t max_landmarks_count = 1024,
    ColorMode color_mode = CM_RGB_MODE, uint32_t period_ms = 500);

  ~LandmarksVisHelper() override;

  void Init(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher,
    ELBRUS_TrackerHandle elbrus_handle,
    const tf2::Transform & base_link_pose_elbrus,
    const rclcpp::Node & node,
    const std::string & frame_id
  );

  void Reset() override;

  void Run();

protected:
  ELBRUS_DataLayer layer_ = LL_OBSERVATIONS;
  uint32_t max_landmarks_count_ = 1024;

  uint64_t last_timestamp_ns_ = 0;

  ColorMode color_mode_ = CM_RGB_MODE;

  uint32_t period_ms_ = 500;

  // Point cloud to visualize tracks
  using pointcloud_msg_t = std::unique_ptr<sensor_msgs::msg::PointCloud2>;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

}  // namespace visual_slam
}  // namespace isaac_ros

#endif  // ISAAC_ROS_VISUAL_SLAM__IMPL__LANDMARKS_VIS_HELPER_HPP_
