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

#ifndef ISAAC_ROS_VISUAL_SLAM__IMPL__VISUAL_SLAM_IMPL_HPP_
#define ISAAC_ROS_VISUAL_SLAM__IMPL__VISUAL_SLAM_IMPL_HPP_

#include <list>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "cuvslam.h"  // NOLINT - include .h without directory
#include "ground_constraint.h"  // NOLINT - include .h without directory
#include "cv_bridge/cv_bridge.h"
#include "isaac_ros_visual_slam/impl/landmarks_vis_helper.hpp"
#include "isaac_ros_visual_slam/impl/localizer_vis_helper.hpp"
#include "isaac_ros_visual_slam/impl/message_stream_synchronizer.hpp"
#include "isaac_ros_visual_slam/impl/message_stream_sequencer.hpp"
#include "isaac_ros_visual_slam/impl/posegraph_vis_helper.hpp"
#include "isaac_ros_visual_slam/impl/pose_cache.hpp"
#include "isaac_ros_visual_slam/impl/types.hpp"
#include "isaac_ros_visual_slam/impl/limited_vector.hpp"
#include "isaac_ros_visual_slam/visual_slam_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace nvidia
{
namespace isaac_ros
{
namespace visual_slam
{

constexpr int32_t kMaxNumCameraParameters = 12;

struct VisualSlamNode::VisualSlamImpl
{
  explicit VisualSlamImpl(VisualSlamNode & vslam_node);

  ~VisualSlamImpl();

  // Helpers to initialize the cuvslam tracker.
  void Initialize();

  bool IsReadyForInitialization() const;

  bool IsInitialized() const;

  void Exit();

  // Create the configuration for the cuvslam tracker.
  CUVSLAM_Configuration CreateConfiguration(const CUVSLAM_Pose & cv_base_link_pose_cv_imu);

  // Helper function to publish a transform to the tf tree.
  void PublishFrameTransform(
    rclcpp::Time stamp, const tf2::Transform & pose, const std::string & target,
    const std::string & source);

  // Helper to get a transform from the tf tree.
  tf2::Transform GetFrameTransform(
    rclcpp::Time stamp, const std::string & target, const std::string & source);

  // Helper to publish the estimated velocity from odometry.
  void PublishOdometryVelocity(
    rclcpp::Time stamp, const std::string & frame_id,
    const rclcpp::Publisher<MarkerArrayType>::SharedPtr publisher);

  // Helper to publish the estimated gravity vector.
  void PublishGravity(
    rclcpp::Time stamp, const std::string & frame_id,
    const rclcpp::Publisher<MarkerType>::SharedPtr publisher);

  // Callback for cuvslam's API to localize in an existing map.
  static void LocalizeInExistDbResponse(
    void * context, CUVSLAM_Status status, const CUVSLAM_Pose * pose_in_db);

  // Callback for cuvslam's API to store a map to disk.
  static void SaveToSlamDbResponse(void * context, CUVSLAM_Status status);

  // Callbacks for subscribers.
  void CallbackImu(const ImuType::ConstSharedPtr & msg);

  void CallbackImage(int index, const ImageType & image_view);

  void CallbackCameraInfo(int index, const CameraInfoType::ConstSharedPtr & msg);

  // Callback for synchronizer.
  void CallbackSynchronizedImages(
    int64_t current_ts, const std::vector<std::pair<int, ImageType>> & idx_and_image_msgs);

  // Callback for sequencer
  void UpdatePose(
    const std::vector<ImuType::ConstSharedPtr> & imu_msgs,
    const std::vector<std::pair<int, ImageType>> & idx_and_image_msgs);

  // Reference to the ros node.
  VisualSlamNode & node;

  // Synchronizer used to sync all image messages.
  using Synchronizer = MessageStreamSynchronizer<ImageType>;
  Synchronizer sync;

  // Sequencer used to sequence imu and image messages.
  using Sequencer = MessageStreamSequencer<ImuType::ConstSharedPtr,
      std::vector<std::pair<int, ImageType>>>;
  Sequencer sequencer;

  // cuVSLAM handle to call cuVSLAM API.
  CUVSLAM_TrackerHandle cuvslam_handle = nullptr;
  CUVSLAM_GroundConstraintHandle ground_constraint_handle = nullptr;

  // Define number of cameras for cuVSLAM. 2 for stereo camera.
  std::vector<CUVSLAM_Camera> cuvslam_cameras;

  // Define camera parameters for all cameras.
  std::vector<std::array<float, kMaxNumCameraParameters>> intrinsics;

  // Helper classes for tf listening and publishing.
  std::unique_ptr<tf2_ros::Buffer> tf_buffer{nullptr};
  std::unique_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_publisher{nullptr};
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_publisher{nullptr};

  limited_vector<PoseStampedType> vo_path;
  limited_vector<PoseStampedType> slam_path;

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

  // Timestamp of the last time CUVSLAM_Track was called in nanoseconds.
  int64_t last_track_ts;

  // Initial messages required for initialization.
  std::optional<ImuType::ConstSharedPtr> initial_imu_message;
  std::map<int, std::optional<CameraInfoType::ConstSharedPtr>> initial_camera_info_messages;

  // Asynchronous response for CUVSLAM_LocalizeInExistDb()
  struct LocalizeInExistDbContext
  {
    VisualSlamNode::VisualSlamImpl * impl = nullptr;
    std::shared_ptr<GoalHandleLoadMapAndLocalize> goal_handle;
  };

  // Asynchronous response for CUVSLAM_SaveToSlamDb()
  struct SaveToSlamDbContext
  {
    std::shared_ptr<GoalHandleSaveMap> goal_handle;
  };
};

}  // namespace visual_slam
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_VISUAL_SLAM__IMPL__VISUAL_SLAM_IMPL_HPP_
