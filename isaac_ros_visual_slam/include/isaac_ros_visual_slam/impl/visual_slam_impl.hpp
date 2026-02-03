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
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#define BOOST_THREAD_PROVIDES_FUTURE
#define BOOST_THREAD_PROVIDES_FUTURE_CONTINUATION
#include "boost/outcome/result.hpp"
#include "boost/thread.hpp"
#include "boost/thread/future.hpp"
#include "cuvslam/cuvslam2.h"
#include "cuvslam/ground_constraint2.h"
#include "cv_bridge/cv_bridge.hpp"
#include "isaac_common/messaging/message_stream_synchronizer.hpp"
#include "isaac_ros_visual_slam/impl/landmarks_vis_helper.hpp"
#include "isaac_ros_visual_slam/impl/limited_vector.hpp"
#include "isaac_ros_visual_slam/impl/localizer_vis_helper.hpp"
#include "isaac_ros_visual_slam/impl/message_stream_sequencer.hpp"
#include "isaac_ros_visual_slam/impl/pose_cache.hpp"
#include "isaac_ros_visual_slam/impl/posegraph_vis_helper.hpp"
#include "isaac_ros_visual_slam/impl/types.hpp"
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
  // Asynchronous response for CUVSLAM_LocalizeInExistDb()
  struct LocalizeInExistDbContext
  {
    using Response = boost::outcome_v2::result<cuvslam::Pose, std::string>;
    boost::promise<Response> response_promise;
  };


  explicit VisualSlamImpl(VisualSlamNode & vslam_node);

  ~VisualSlamImpl();

  // Helpers to initialize the cuvslam tracker.
  void Initialize();

  bool IsReadyForInitialization() const;

  bool IsInitialized() const;

  void Exit();

  // Create the configuration for the cuvslam tracker.
  cuvslam::Odometry::Config CreateOdometryConfiguration();

  // Create the configuration for the cuvslam slam.
  cuvslam::Slam::Config CreateSlamConfiguration();

  // Helper function to publish a transform to the tf tree.
  void PublishFrameTransform(
    rclcpp::Time stamp, const tf2::Transform & pose, const std::string & target,
    const std::string & source);

  // Helper to get latest transform from the tf tree.
  tf2::Transform GetLatestTransform(
    const std::string & target, const std::string & source);

  // Helper to publish the estimated velocity from odometry.
  void PublishOdometryVelocity(
    rclcpp::Time stamp, const std::string & frame_id,
    const rclcpp::Publisher<MarkerArrayType>::SharedPtr publisher);

  // Helper to publish the estimated gravity vector.
  void PublishGravity(
    rclcpp::Time stamp, const std::string & frame_id,
    const rclcpp::Publisher<MarkerType>::SharedPtr publisher);

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

  // Save the current map to disk.
  bool SaveMap(const std::string & map_folder_path);

  // Core functionality to localize in a map. Expects and returns the pose in cuvslam conventions.
  boost::future<LocalizeInExistDbContext::Response> CuvslamInternalLocalizeInMapAsync(
    const std::string & map_folder_path,
    const cuvslam::Pose & pose_hint);

  // Localize in an existing map asynchronously. Expects and returns the pose in ROS conventions.
  boost::future<std::optional<PoseType>> LocalizeInMapAsync(
    const std::string & map_folder_path,
    const PoseType & pose_hint,
    const std::string & frame_id);

  // Synchonous wrapper for LocalizeInMapAsync.
  std::optional<PoseType> LocalizeInMap(
    const std::string & map_folder_path,
    const PoseType & pose_hint,
    const std::string & frame_id);

  // Check and handle the localization future status
  void CheckLocalizationStatus();

  // Reference to the ros node.
  VisualSlamNode & node;

  // Synchronizer used to sync all image messages.
  using Synchronizer = isaac_common::messaging::MessageStreamSynchronizer<ImageType>;
  Synchronizer sync;

  // Sequencer used to sequence imu and image messages.
  using Sequencer = MessageStreamSequencer<ImuType::ConstSharedPtr,
      std::vector<std::pair<int, ImageType>>>;
  Sequencer sequencer;

  // cuVSLAM API objects
  std::unique_ptr<cuvslam::Odometry> cuvslam_odometry;
  cuvslam::Odometry::State odometry_state;
  std::shared_ptr<cuvslam::Slam> cuvslam_slam;  // Shared pointer for async visualization helpers
  std::unique_ptr<cuvslam::GroundConstraint> ground_constraint;

  // Define cameras for cuVSLAM. 2 for stereo camera.
  std::vector<cuvslam::Camera> cuvslam_cameras;

  // Helper classes for tf listening and publishing.
  std::unique_ptr<tf2_ros::Buffer> tf_buffer{nullptr};
  std::unique_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_publisher{nullptr};
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_publisher{nullptr};

  limited_vector<PoseStampedType> vo_path;
  limited_vector<PoseStampedType> slam_path;

  // Point cloud to visualize tracks
  // LandmarksVisHelper observations_vis_helper;
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

  LocalizeInExistDbContext localize_in_exist_db_context;
  bool localized_in_exist_map_ = false;

  // Store the localization future to prevent it from being destroyed before callback completes
  std::optional<boost::future<std::optional<PoseType>>> localization_future_;
  mutable std::mutex localization_mutex_;
};

}  // namespace visual_slam
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_VISUAL_SLAM__IMPL__VISUAL_SLAM_IMPL_HPP_
