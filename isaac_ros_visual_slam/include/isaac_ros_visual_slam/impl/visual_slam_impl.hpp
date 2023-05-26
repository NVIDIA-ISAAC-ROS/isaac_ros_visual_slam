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

#ifndef ISAAC_ROS_VISUAL_SLAM__IMPL__VISUAL_SLAM_IMPL_HPP_
#define ISAAC_ROS_VISUAL_SLAM__IMPL__VISUAL_SLAM_IMPL_HPP_

#include <list>
#include <memory>
#include <string>

#include "cuvslam.h"  // NOLINT - include .h without directory
#include "cv_bridge/cv_bridge.h"
#include "isaac_ros_visual_slam/impl/landmarks_vis_helper.hpp"
#include "isaac_ros_visual_slam/impl/localizer_vis_helper.hpp"
#include "isaac_ros_visual_slam/impl/posegraph_vis_helper.hpp"
#include "isaac_ros_visual_slam/impl/pose_cache.hpp"
#include "isaac_ros_visual_slam/impl/limited_vector.hpp"
#include "isaac_ros_visual_slam/visual_slam_node.hpp"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/exact_time.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace isaac_ros
{
namespace visual_slam
{

constexpr int32_t kNumCameras = 2;
constexpr int32_t kMaxNumCameraParameters = 12;

constexpr int32_t LEFT_CAMERA_IDX = 0;
constexpr int32_t RIGHT_CAMERA_IDX = 1;

double NanoToMilliSeconds(int64_t nano_sec);

struct VisualSlamNode::VisualSlamImpl
{
  explicit VisualSlamImpl(VisualSlamNode & node);

  ~VisualSlamImpl();

  void Init(
    VisualSlamNode & node,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg_left_ci,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg_right_ci);
  void Exit();

  CUVSLAM_Configuration GetConfiguration(
    const VisualSlamNode & node,
    const CUVSLAM_Pose & imu_from_camera);

  void SetcuVSLAMCameraPose(CUVSLAM_Camera & cam, const CUVSLAM_Pose & cuvslam_pose);

  void ReadCameraIntrinsics(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg_ci,
    CUVSLAM_Camera & cuvslam_camera,
    std::array<float, kMaxNumCameraParameters> & intrinsics);

  CUVSLAM_Image TocuVSLAMImage(
    int32_t camera_index, const cv::Mat & image,
    const int64_t & acqtime_ns);

  CUVSLAM_ImuMeasurement TocuVSLAMImuMeasurement(
    const sensor_msgs::msg::Imu::ConstSharedPtr & msg_imu);

  void PublishFrameTransform(
    rclcpp::Time stamp,
    const tf2::Transform & pose,
    const std::string & target,
    const std::string & source);

  tf2::Transform GetFrameTransform(
    rclcpp::Time stamp,
    const std::string & target,
    const std::string & source);

  void PublishOdometryVelocity(
    rclcpp::Time stamp,
    const std::string & frame_id,
    const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher);

  void PublishGravity(
    rclcpp::Time stamp,
    const std::string & frame_id,
    const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher);

  bool IsInitialized();

  static void LocalizeInExistDbResponse(
    void * context, CUVSLAM_Status status,
    const CUVSLAM_Pose * pose_in_db);

  static void SaveToSlamDbResponse(void * context, CUVSLAM_Status status);

  bool IsJitterAboveThreshold(double threshold);

  // Name of the logger that will used to retrieve the logger.
  const std::string logger_name;

  // Clock shared pointer to access clock.
  const std::shared_ptr<rclcpp::Clock> node_clock;

  // Message filter policy to be applied to the synchronizer
  // to synchronize the incoming messages on the topics.
  using ExactTime = message_filters::sync_policies::ExactTime<
    sensor_msgs::msg::Image,
    sensor_msgs::msg::CameraInfo,
    sensor_msgs::msg::Image,
    sensor_msgs::msg::CameraInfo>;
  using Synchronizer = message_filters::Synchronizer<ExactTime>;
  std::shared_ptr<Synchronizer> sync;

  // cuVSLAM handle to call cuVSLAM API.
  CUVSLAM_TrackerHandle cuvslam_handle = nullptr;

  // Define number of cameras for cuVSLAM. 2 for stereo camera.
  std::array<CUVSLAM_Camera, kNumCameras> cuvslam_cameras;

  // Define camera parameters for stereo camera.
  std::array<float, kMaxNumCameraParameters> left_intrinsics;
  std::array<float, kMaxNumCameraParameters> right_intrinsics;
  // Transformation converting from
  // ROS Frame    (x-forward, y-left, z-up) to
  // cuVSLAM Frame (x-right, y-up, z-backward)
  const tf2::Transform cuvslam_pose_base_link;

  // Transformation converting from
  // cuVSLAM Frame (x-right, y-up, z-backward) to
  // ROS Frame    (x-forward, y-left, z-up)
  const tf2::Transform base_link_pose_cuvslam;

  // Start pose of visual odometry
  tf2::Transform start_odom_pose = tf2::Transform::getIdentity();

  // From robot center to left camera in ROS frame. Usually identity.
  tf2::Transform left_pose_base_link = tf2::Transform::getIdentity();

  // Buffer for tf2.
  std::unique_ptr<tf2_ros::Buffer> tf_buffer{nullptr};

  // Listener for tf2. It takes in data from tf2 buffer.
  std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};

  // Publisher for tf2.
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_publisher{nullptr};

  limited_vector<geometry_msgs::msg::PoseStamped> vo_path;
  limited_vector<geometry_msgs::msg::PoseStamped> slam_path;

  // Point cloud to visualize tracks
  LandmarksVisHelper observations_vis_helper;
  // Point cloud to visualize landmarks
  LandmarksVisHelper landmarks_vis_helper;
  // Loop closure landmarks
  LandmarksVisHelper lc_landmarks_vis_helper;
  // PoseGraph visualization
  PoseGraphVisHelper pose_graph_helper;
  // Localizer visualization
  LocalizerVisHelper localizer_helper;
  // Localizer: map landmarks
  LandmarksVisHelper localizer_landmarks_vis_helper;
  // Localizer: tracked landmarks
  LandmarksVisHelper localizer_observations_vis_helper;
  // Localizer: loop closure landmarks
  LandmarksVisHelper localizer_lc_landmarks_vis_helper;

  // Pose cache. For velocity calculation.
  PoseCache pose_cache;

  // Velocity cache. For velocity covariance calculation.
  VelocityCache velocity_cache;

  // Container to calulate execution time statictics.
  limited_vector<double> track_execution_times;

  // Container to store delta between camera frames timestamp.
  limited_vector<double> delta_ts;

  // Last timestamp of the image message in nanoseconds
  int64_t last_img_ts;

  // Asynchronous response for CUVSLAM_LocalizeInExistDb()
  struct LocalizeInExistDbContext
  {
    VisualSlamNode * node = nullptr;
    std::shared_ptr<GoalHandleLoadMapAndLocalize> goal_handle;
  };
  std::list<LocalizeInExistDbContext> localize_in_exist_db_contexts;

  // Asynchronous response for CUVSLAM_SaveToSlamDb()
  struct SaveToSlamDbContext
  {
    VisualSlamNode * node = nullptr;
    std::shared_ptr<GoalHandleSaveMap> goal_handle;
  };
  std::list<SaveToSlamDbContext> save_to_slam_db_contexts;
};

}  // namespace visual_slam
}  // namespace isaac_ros

#endif  // ISAAC_ROS_VISUAL_SLAM__IMPL__VISUAL_SLAM_IMPL_HPP_
