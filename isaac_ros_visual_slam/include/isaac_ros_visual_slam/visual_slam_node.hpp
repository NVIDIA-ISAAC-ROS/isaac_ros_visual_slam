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

#ifndef ISAAC_ROS_VISUAL_SLAM__VISUAL_SLAM_NODE_HPP_
#define ISAAC_ROS_VISUAL_SLAM__VISUAL_SLAM_NODE_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "isaac_ros_visual_slam/impl/types.hpp"
#include "message_filters/subscriber.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
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
  // Functional Parameters:
  // Number of cameras used. If a single stereocamera is used this has to be set to 2.
  // This is the single source of truth, ie. if you set num_cameras=2 but pass 4
  // input_camera_optical_frames we will only use the first 2.
  const uint num_cameras_;

  // Multicamera mode: moderate (0), performance (1) or precision (2).
  // Multicamera odometry settings will be adjusted depending on a chosen strategy.
  // Default is performance (1).
  const uint multicam_mode_;

  // The minimum number of images used. This configures how many camera images can be dropped
  // without stopping tracking.
  const uint min_num_images_;

  // Maximum value of acceptable timestamp deltas between the camera images to still consider their
  // timestamp identical. Should be < 0.5 / fps_value.
  const double sync_matching_threshold_ms_;

  // Number of pixels on the outer edges of the frame will be masked
  // during the feature selection stage. Values are defined for the left camera
  // of the stereo pair (camera at even index in the list of cameras). For the right camera
  // (camera at odd index in the list of cameras), the left and right borders will be mirrored.
  const uint border_mask_top_;
  const uint border_mask_bottom_;
  const uint border_mask_left_;
  const uint border_mask_right_;

  // Disable this if the input images have already passed through a denoising filter.
  const bool enable_image_denoising_;

  // Enable fast and robust left-to-right tracking for rectified cameras with
  // principal points on the horizontal line. Disable this if input images are
  // not already rectified.
  const bool rectified_images_;

  // If enabled, visual odometry poses will be modified such that the camera moves
  // on a horizontal plane.
  const bool enable_ground_constraint_in_odometry_;

  // If enabled, slam poses will be modified such that the camera moves on a horizontal plane.
  const bool enable_ground_constraint_in_slam_;

  // Use localization and mapping (SLAM).
  // If it is false, it is only Visual Odometry and no map will get produced.
  const bool enable_localization_n_mapping_;

  // Enable IMU fusion mode
  const bool enable_imu_fusion_;

  // Enable IMU debug mode
  const bool enable_debug_imu_mode_;

  // Data structure to store imu noise params
  const IMUParams imu_params_;

  // Minimum value of acceptable jitter (delta between current and previous timestamps)
  // Ideally it should be equal to (1 / fps_value). cuVSLAM library will print out warning messages
  // if rate of incoming image pair is lower than the threshold value.
  const double image_jitter_threshold_ms_;

  // Minimum value of acceptable jitter (delta between current and previous timestamps)
  // Ideally it should be equal to (1 / fps_value).
  const double imu_jitter_threshold_ms_;

  // This node assumes the following frame hierarchy:
  // map - odom - base - ... - camera_i
  const std::string map_frame_;
  const std::string odom_frame_;
  const std::string base_frame_;

  // Per default the frames from the camera info messages will be used. Use this parameter to
  // override this.
  const std::vector<std::string> camera_optical_frames_;

  // Per default the frame from the imu message will be used. Use this parameter to override this.
  const std::string imu_frame_;

  // Message Parameters:
  // Buffer size of image buffer used in synchronizer.
  const uint image_buffer_size_;

  // Buffer size of imu buffer.
  const uint imu_buffer_size_;

  // The QoS used for the image and subscriptions
  const rclcpp::QoS image_qos_;
  const rclcpp::QoS imu_qos_;

  // Output Parameters:
  // Enable this to override the timestamps of all outputs to the current time.
  // This is helpful when playing back with rosbags and allows to ignore the
  // recording time.
  const bool override_publishing_stamp_;

  // Publish output frames hierarchy.
  const bool publish_map_to_odom_tf_;
  const bool publish_odom_to_base_tf_;

  // Invert the odom -> base transform published to the tf tree.
  // This is useful for maintaining the one-parent rule in systems with multiple odometry sources.
  const bool invert_map_to_odom_tf_;
  const bool invert_odom_to_base_tf_;

  // Debug/Visualization Parameters:
  // Enable slam data visualization (Landmarks, Pose Graph, etc).
  // Demands additional hardware resources.
  const bool enable_slam_visualization_;

  // Enable view observations.
  const bool enable_observations_view_;

  // Enable view landmarks for map and loop closure events.
  const bool enable_landmarks_view_;

  // Max size of the buffer for pose trail visualization.
  const uint path_max_size_;

  // Verbosity for cuvslam, the larger the more verbose.
  const int verbosity_;

  // Enable this flag to dump the received images, imu messages, timestamps and
  // camera infos at the location specified by debug_dump_path_.
  const bool enable_debug_mode_;

  // Path to directory to store the dump data. Only used when enable_debug_mode_ is true.
  const std::string debug_dump_path_;

  // Subscribers
  const std::vector<std::shared_ptr<NitrosImageViewSubscriber>> image_subs_;
  const std::vector<rclcpp::Subscription<CameraInfoType>::SharedPtr> camera_info_subs_;
  const rclcpp::Subscription<ImuType>::SharedPtr imu_sub_;

  // Publishers: Visual SLAM
  const rclcpp::Publisher<VisualSlamStatusType>::SharedPtr visual_slam_status_pub_;
  const rclcpp::Publisher<PoseStampedType>::SharedPtr tracking_vo_pose_pub_;
  const rclcpp::Publisher<PoseWithCovarianceStampedType>::SharedPtr
    tracking_vo_pose_covariance_pub_;
  const rclcpp::Publisher<OdometryType>::SharedPtr tracking_odometry_pub_;
  const rclcpp::Publisher<PathType>::SharedPtr tracking_vo_path_pub_;
  const rclcpp::Publisher<PathType>::SharedPtr tracking_slam_path_pub_;
  const rclcpp::Publisher<DiagnosticArrayType>::SharedPtr diagnostics_pub_;

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
  const rclcpp::Service<SrvSetSlamPose>::SharedPtr set_slam_pose_srv_;

  void CallbackReset(
    const std::shared_ptr<SrvReset::Request> req,
    std::shared_ptr<SrvReset::Response> res);
  void CallbackGetAllPoses(
    const std::shared_ptr<SrvGetAllPoses::Request> req,
    std::shared_ptr<SrvGetAllPoses::Response> res);
  void CallbackSetSlamPose(
    const std::shared_ptr<SrvSetSlamPose::Request> req,
    std::shared_ptr<SrvSetSlamPose::Response> res);

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

  // Callback functions for sensors
  void CallbackImu(const ImuType::ConstSharedPtr & msg);
  void CallbackImage(int index, const ImageType & msg);
  void CallbackCameraInfo(int index, const CameraInfoType::ConstSharedPtr & msg);

  struct VisualSlamImpl;
  std::unique_ptr<VisualSlamImpl> impl_;
};

}  // namespace visual_slam
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_VISUAL_SLAM__VISUAL_SLAM_NODE_HPP_
