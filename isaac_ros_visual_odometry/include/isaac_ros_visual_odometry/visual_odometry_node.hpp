/**
 * Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef ISAAC_ROS_VISUAL_ODOMETRY__VISUAL_ODOMETRY_NODE_HPP_
#define ISAAC_ROS_VISUAL_ODOMETRY__VISUAL_ODOMETRY_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "isaac_ros_visual_odometry_interfaces/msg/visual_odometry_transform_stamped.hpp"
#include "message_filters/subscriber.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace isaac_ros
{
namespace visual_odometry
{

class VisualOdometryNode : public rclcpp::Node
{
public:
  explicit VisualOdometryNode(const rclcpp::NodeOptions options = rclcpp::NodeOptions());
  ~VisualOdometryNode();

private:
  // Enable rectified mode(default mode). If disabled only pure visual odometry will be invoked.
  // Otherwise (enabled), VisualSLAM will be used to perform a Pose Graph Optimization
  // as well as to build and use an internal Map for real-time Loop Closure.
  // Loop Closure increases tracking accuracy and robustness but also increases CPU usage.
  bool enable_rectified_pose_;

  // Enable image denoising.
  // Disable if the input images have already passed through a denoising filter.
  bool denoise_input_images_;

  // Enable fast and robust left-to-right tracking for rectified
  // cameras with principal points on the horizontal line.
  bool rectified_images_;

  // Enable IMU data acquisition and integration
  bool enable_imu_;

  // Enable this flag to dump the image frames, timestamp and camerainfo at the location specified
  // by debug_dump_path_
  bool enable_debug_mode_;

  // Path to directory to store the dump data. It will be used when enable_debug_mode_ is true.
  const std::string debug_dump_path_;

  // Defines the name of the left camera frame.
  // It is used to calculate the left_pose_right camera extrinsics.
  const std::string left_camera_frame_;

  // Defines the name of the right camera frame.
  // It is used to calculate the left_pose_right camera extrinsics.
  const std::string right_camera_frame_;

  // Defines the name of the IMU camera frame used to calculate left_camera_pose_imu
  // The IMU to left camera transformation
  const std::string imu_frame_;

  // The gravitational force vector
  const std::vector<double> gravitational_force_;

  // Name of the fixed frame for the output pose
  const std::string fixed_frame_;

  // Name of the current frame for the output pose
  const std::string current_smooth_frame_;

  // Name of the current frame for the output pose
  const std::string current_rectified_frame_;

  // Message filter queue size
  const uint msg_filter_queue_size_;

  // Unit in seconds
  const int32_t msg_filter_max_interval_duration_;

  // Publishers and subscribers
  const rclcpp::Publisher<isaac_ros_visual_odometry_interfaces::msg::VisualOdometryTransformStamped>
  ::SharedPtr visual_odometry_msg_pub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> left_image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> left_camera_info_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> right_image_sub_;
  message_filters::Subscriber<sensor_msgs::msg::CameraInfo> right_camera_info_sub_;
  const rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  // Callback function for images
  void TrackCameraPose(
    const sensor_msgs::msg::Image::ConstSharedPtr & msg_left_img,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg_left_camera_info,
    const sensor_msgs::msg::Image::ConstSharedPtr & msg_right_img,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg_right_camera_info);

  // Callback funtion for imu
  void ReadImuData(const sensor_msgs::msg::Imu::ConstSharedPtr msg_imu);

  struct VisualOdometryImpl;
  std::unique_ptr<VisualOdometryImpl> impl_;
};
}  // namespace visual_odometry
}  // namespace isaac_ros

#endif  // ISAAC_ROS_VISUAL_ODOMETRY__VISUAL_ODOMETRY_NODE_HPP_
