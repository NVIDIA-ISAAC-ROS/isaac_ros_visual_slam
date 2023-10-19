// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2021-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef ISAAC_ROS_VISUAL_SLAM__VISUAL_SLAM_NODE_HPP_
#define ISAAC_ROS_VISUAL_SLAM__VISUAL_SLAM_NODE_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "isaac_ros_visual_slam/impl/types.hpp"
#include "message_filters/subscriber.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace visual_slam
{

struct IMUParams
{
  double gyroscope_noise_density;       // rad / (s * srqt(Hz))
  double gyroscope_random_walk;         // rad / (s ^ 2 * srqt(Hz))
  double accelerometer_noise_density;   // m / (s ^ 2 * srqt(Hz))
  double accelerometer_random_walk;     // m / (s ^ 3 * srqt(Hz))
  double calibration_frequency;
};

class VisualSlamNode : public rclcpp::Node
{
public:
  explicit VisualSlamNode(const rclcpp::NodeOptions options = rclcpp::NodeOptions());
  virtual ~VisualSlamNode();

private:
  // Enable image denoising.
  // Disable if the input images have already passed through a denoising filter.
  bool denoise_input_images_;

  // Enable fast and robust left-to-right tracking for rectified
  // cameras with principal points on the horizontal line.
  bool rectified_images_;

  // Enable IMU fusion mode
  bool enable_imu_fusion_;

  // Data structure to store imu noise params
  IMUParams imu_params_;

  // Minimum value of acceptable jitter (delta between current and previous timestamps)
  // Ideally it should be equal to (1 / fps_value). cuVSLAM library will print out warning messages
  // if rate of incoming image pair is lower than the threshold value.
  double img_jitter_threshold_ms_;

  // Minimum value of acceptable jitter (delta between current and previous timestamps)
  // Ideally it should be equal to (1 / fps_value).
  double imu_jitter_threshold_ms_;

  // If enabled, prints logs from cuVSLAM library
  bool enable_verbosity_;

  // If enabled, slam poses will be modified such that the camera moves on a horizontal plane.
  bool force_planar_mode_;


  // Enable view observations
  bool enable_observations_view_;

  // Enable view landmarks for map and loop closure events
  bool enable_landmarks_view_;

  // Enable this flag to dump the image frames, timestamp and camerainfo at the location specified
  // by debug_dump_path_
  bool enable_debug_mode_;

  // Path to directory to store the dump data. It will be used when enable_debug_mode_ is true.
  const std::string debug_dump_path_;

  // Use localization and mapping.
  // If it is false, it is only Visual Odometry and no map will get produced
  // Default: true
  bool enable_localization_n_mapping_;

  // Enable slam data visualization (Landmarks, Pose Graph, etc).
  // Demands additional hardware resources
  bool enable_slam_visualization_;

  // Name of frame (baselink) to calculate transformation between baselink and left camera
  // Default is empty, which means the value of base_frame_ will be used.
  // If input_base_frame_ and base_frame_ are both empty the left camera is assumed to be in
  // the robot's center.
  const std::string input_base_frame_;

  // Defines the name of the left camera frame.
  // It is used to calculate the left_pose_right camera extrinsics.
  // Default is empty, which means the left camera is in the robot's center and
  // left_pose_right will be calculated from CameraInfo
  const std::string input_left_camera_frame_;

  // Defines the name of the right camera frame.
  // It is used to calculate the left_pose_right camera extrinsics.
  // Default is empty, which means left and right cameras has identity rotation and
  // horizontally aligned, so left_pose_right will be calculated from CameraInfo
  // See camera matrix info for more details : https://docs.ros2.org/humble/api/sensor_msgs/msg/CameraInfo.html
  const std::string input_right_camera_frame_;

  // Defines the name of the IMU camera frame used to calculate left_camera_pose_imu
  // The IMU to left camera transformation
  const std::string input_imu_frame_;

  // Publish output frames hierarchy. Default is true.
  bool publish_odom_to_base_tf_;
  bool publish_map_to_odom_tf_;

  // Invert the odom -> base_frame transform published to the tf tree.
  // This is useful for maintaining the one-parent rule in systems with multiple odometry sources.
  bool invert_odom_to_base_tf_ = false;
  bool invert_map_to_odom_tf_ = false;

  // Output frames hierarchy:
  // map - odom - base_link - ... - camera
  const std::string map_frame_ = "map";
  const std::string odom_frame_ = "odom";
  const std::string base_frame_ = "base_link";

  // When override_publishing_stamp_ is true, it will help in using a ros bag file with current
  // timestamp and ignoring the recording time.
  bool override_publishing_stamp_ = false;

  // Max size of the buffer for pose trail visualization
  const uint path_max_size_ = 1024;

  // Message filter queue size
  const uint msg_filter_queue_size_;

  // The QoS used for the image subscriptions
  const rmw_qos_profile_t image_qos_;

  // Subscribers
  message_filters::Subscriber<ImageType> left_image_sub_;
  message_filters::Subscriber<CameraInfoType> left_camera_info_sub_;
  message_filters::Subscriber<ImageType> right_image_sub_;
  message_filters::Subscriber<CameraInfoType> right_camera_info_sub_;
  const rclcpp::Subscription<ImuType>::SharedPtr imu_sub_;

  // Publishers: Visual SLAM
  const rclcpp::Publisher<VisualSlamStatusType>::SharedPtr visual_slam_status_pub_;
  const rclcpp::Publisher<PoseStampedType>::SharedPtr tracking_vo_pose_pub_;
  const rclcpp::Publisher<PoseWithCovarianceStampedType>::SharedPtr
    tracking_vo_pose_covariance_pub_;
  const rclcpp::Publisher<OdometryType>::SharedPtr tracking_odometry_pub_;
  const rclcpp::Publisher<PathType>::SharedPtr tracking_vo_path_pub_;
  const rclcpp::Publisher<PathType>::SharedPtr tracking_slam_path_pub_;

  // Visualization for odometry
  const rclcpp::Publisher<PointCloud2Type>::SharedPtr vis_observations_pub_;
  const rclcpp::Publisher<MarkerType>::SharedPtr vis_gravity_pub_;
  const rclcpp::Publisher<MarkerArrayType>::SharedPtr vis_vo_velocity_pub_;
  const rclcpp::Publisher<OdometryType>::SharedPtr vis_slam_odometry_pub_;

  // Visualization for map and "loop closure"
  const rclcpp::Publisher<PointCloud2Type>::SharedPtr vis_landmarks_pub_;
  const rclcpp::Publisher<PointCloud2Type>::SharedPtr vis_loop_closure_pub_;
  const rclcpp::Publisher<PoseArrayType>::SharedPtr vis_posegraph_nodes_pub_;
  const rclcpp::Publisher<MarkerType>::SharedPtr vis_posegraph_edges_pub_;
  const rclcpp::Publisher<MarkerType>::SharedPtr vis_posegraph_edges2_pub_;

  // Visualization for localization
  const rclcpp::Publisher<MarkerArrayType>::SharedPtr vis_localizer_pub_;
  const rclcpp::Publisher<PointCloud2Type>::SharedPtr vis_localizer_landmarks_pub_;
  const rclcpp::Publisher<PointCloud2Type>::SharedPtr vis_localizer_observations_pub_;
  const rclcpp::Publisher<PointCloud2Type>::SharedPtr vis_localizer_loop_closure_pub_;

  // Slam Services
  const rclcpp::Service<SrvReset>::SharedPtr reset_srv_;
  const rclcpp::Service<SrvGetAllPoses>::SharedPtr get_all_poses_srv_;
  const rclcpp::Service<SrvSetOdometryPose>::SharedPtr set_odometry_pose_srv_;

  void CallbackReset(
    const std::shared_ptr<SrvReset::Request> req,
    std::shared_ptr<SrvReset::Response> res);
  void CallbackGetAllPoses(
    const std::shared_ptr<SrvGetAllPoses::Request> req,
    std::shared_ptr<SrvGetAllPoses::Response> res);
  void CallbackSetOdometryPose(
    const std::shared_ptr<SrvSetOdometryPose::Request> req,
    std::shared_ptr<SrvSetOdometryPose::Response> res);

  // SaveMap Action
  rclcpp_action::Server<ActionSaveMap>::SharedPtr save_map_server_;
  using GoalHandleSaveMap = rclcpp_action::ServerGoalHandle<ActionSaveMap>;
  rclcpp_action::GoalResponse CallbackSaveMapGoal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ActionSaveMap::Goal> goal);
  rclcpp_action::CancelResponse CallbackSaveMapCancel(
    const std::shared_ptr<GoalHandleSaveMap> goal_handle);
  void CallbackSaveMapAccepted(
    const std::shared_ptr<GoalHandleSaveMap> goal_handle);

  // LoadMapAndLocalize Action
  rclcpp_action::Server<ActionLoadMapAndLocalize>::SharedPtr load_map_and_localize_server_;
  using GoalHandleLoadMapAndLocalize = rclcpp_action::ServerGoalHandle<ActionLoadMapAndLocalize>;
  rclcpp_action::GoalResponse CallbackLoadMapAndLocalizeGoal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr
    <const ActionLoadMapAndLocalize::Goal> goal);
  rclcpp_action::CancelResponse CallbackLoadMapAndLocalizeCancel(
    const std::shared_ptr<GoalHandleLoadMapAndLocalize> goal_handle);
  void CallbackLoadMapAndLocalizeAccepted(
    const std::shared_ptr<GoalHandleLoadMapAndLocalize> goal_handle);

  // Callback function for images
  void ReadImageData(
    const ImageType::ConstSharedPtr & msg_left_img,
    const CameraInfoType::ConstSharedPtr & msg_left_camera_info,
    const ImageType::ConstSharedPtr & msg_right_img,
    const CameraInfoType::ConstSharedPtr & msg_right_camera_info);

  // Callback funtion for imu
  void ReadImuData(const ImuType::ConstSharedPtr msg_imu);

  // Callback function for interleaved imu and image messages

  void ComputePose(
    const std::vector<ImuType::ConstSharedPtr> imu_msgs,
    const std::pair<ImageType::ConstSharedPtr, ImageType::ConstSharedPtr> image_pair);

  struct VisualSlamImpl;
  std::unique_ptr<VisualSlamImpl> impl_;

  bool valid_cuvslam_api_ = true;
};

}  // namespace visual_slam
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_VISUAL_SLAM__VISUAL_SLAM_NODE_HPP_
