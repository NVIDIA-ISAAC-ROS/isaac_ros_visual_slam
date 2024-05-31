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

#include <algorithm>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include "eigen3/Eigen/Dense"
#include "isaac_ros_nitros/types/type_utility.hpp"
#include "isaac_ros_visual_slam/impl/cuvslam_ros_conversion.hpp"
#include "isaac_ros_visual_slam/impl/has_subscribers.hpp"
#include "isaac_ros_visual_slam/impl/stopwatch.hpp"
#include "isaac_ros_visual_slam/impl/types.hpp"
#include "isaac_ros_visual_slam/impl/visual_slam_impl.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace
{

void PrintConfiguration(const rclcpp::Logger & logger, const CUVSLAM_Configuration & cfg)
{
  RCLCPP_INFO(logger, "Use use_gpu: %s", cfg.use_gpu ? "true" : "false");
  RCLCPP_INFO(logger, "Enable IMU Fusion: %s", cfg.enable_imu_fusion ? "true" : "false");
  if (cfg.enable_imu_fusion) {
    RCLCPP_INFO(
      logger, "gyroscope_noise_density: %f",
      cfg.imu_calibration.gyroscope_noise_density);
    RCLCPP_INFO(
      logger, "gyroscope_random_walk: %f",
      cfg.imu_calibration.gyroscope_random_walk);
    RCLCPP_INFO(
      logger, "accelerometer_noise_density: %f",
      cfg.imu_calibration.accelerometer_noise_density);
    RCLCPP_INFO(
      logger, "accelerometer_random_walk: %f",
      cfg.imu_calibration.accelerometer_random_walk);
    RCLCPP_INFO(
      logger, "frequency: %f",
      cfg.imu_calibration.frequency);
  }
}

bool InitGroundConstraint(CUVSLAM_GroundConstraintHandle * ground, const rclcpp::Logger & logger)
{
  const CUVSLAM_Pose identity = nvidia::isaac_ros::visual_slam::TocuVSLAMPose(
    tf2::Transform::getIdentity());
  const CUVSLAM_Pose world_pose_ground = identity;
  const CUVSLAM_Pose initial_pose_on_ground = identity;
  const CUVSLAM_Pose initial_pose_in_space = identity;

  const CUVSLAM_Status status = CUVSLAM_GroundConstraintCreate(
    ground, &world_pose_ground, &initial_pose_on_ground, &initial_pose_in_space);
  if (status != CUVSLAM_SUCCESS) {
    RCLCPP_ERROR(logger, "Failed to initialize cuVSLAM ground constraint: %d", status);
    return false;
  }
  return true;
}

bool PoseToGround(CUVSLAM_GroundConstraintHandle ground, CUVSLAM_Pose & pose)
{
  if (CUVSLAM_GroundConstraintAddNextPose(ground, &pose) != CUVSLAM_SUCCESS) {
    return false;
  }
  if (CUVSLAM_GroundConstraintGetPoseOnGround(ground, &pose) != CUVSLAM_SUCCESS) {
    return false;
  }
  return true;
}

}  // namespace


namespace nvidia
{
namespace isaac_ros
{
namespace visual_slam
{

VisualSlamNode::VisualSlamImpl::VisualSlamImpl(VisualSlamNode & vslam_node)
: node(vslam_node),
  sync(node.num_cameras_, 1e6 * node.sync_matching_threshold_ms_, node.min_num_images_,
    node.image_buffer_size_),
  sequencer(node.imu_buffer_size_, node.imu_jitter_threshold_ms_, node.image_buffer_size_,
    node.image_jitter_threshold_ms_),
  tf_buffer(std::make_unique<tf2_ros::Buffer>(node.get_clock())),
  tf_listener(std::make_unique<tf2_ros::TransformListener>(*tf_buffer)),
  tf_publisher(std::make_unique<tf2_ros::TransformBroadcaster>(&node)),
  tf_static_publisher(std::make_unique<tf2_ros::StaticTransformBroadcaster>(&node)),
  vo_path(node.path_max_size_),
  slam_path(node.path_max_size_),
  observations_vis_helper(LL_OBSERVATIONS, 2048, LandmarksVisHelper::CM_RGB_MODE, 16),
  landmarks_vis_helper(LL_MAP, 1024 * 32, LandmarksVisHelper::CM_BW_MODE, 100),
  lc_landmarks_vis_helper(LL_LOOP_CLOSURE, 2048, LandmarksVisHelper::CM_RED_MODE, 16),
  pose_graph_helper(LL_POSE_GRAPH, 2048, 100),
  localizer_helper(8 * 2048, 100),
  localizer_landmarks_vis_helper(LL_LOCALIZER_MAP, 1024 * 32, LandmarksVisHelper::CM_GREEN_MODE,
    16),
  localizer_observations_vis_helper(LL_LOCALIZER_OBSERVATIONS, 1024 * 32,
    LandmarksVisHelper::CM_WEIGHT_BW_MODE, 16),
  localizer_lc_landmarks_vis_helper(LL_LOCALIZER_LOOP_CLOSURE, 2048,
    LandmarksVisHelper::CM_RED_MODE, 16),
  track_execution_times(100),
  last_track_ts(-1)
{
  CUVSLAM_SetVerbosity(node.verbosity_);

  sequencer.RegisterCallback(
    std::bind(
      &VisualSlamNode::VisualSlamImpl::UpdatePose, this,
      std::placeholders::_1, std::placeholders::_2));

  sync.RegisterCallback(
    std::bind(
      &VisualSlamNode::VisualSlamImpl::CallbackSynchronizedImages, this,
      std::placeholders::_1, std::placeholders::_2));
}

VisualSlamNode::VisualSlamImpl::~VisualSlamImpl()
{
  Exit();
}

// Flag to check the status of initialization.
bool VisualSlamNode::VisualSlamImpl::IsInitialized() const
{
  return cuvslam_handle != nullptr;
}

bool VisualSlamNode::VisualSlamImpl::IsReadyForInitialization() const
{
  // We need at least one message of every sensor to initialize.
  if (node.enable_imu_fusion_ && !initial_imu_message) {
    return false;
  }

  if (initial_camera_info_messages.size() == static_cast<size_t>(node.num_cameras_)) {
    return true;
  }
  return false;
}

void VisualSlamNode::VisualSlamImpl::Initialize()
{
  if (IsInitialized()) {
    RCLCPP_WARN(node.get_logger(), "VisualSlamImpl was already initialized.");
    return;
  }

  RCLCPP_INFO(node.get_logger(), "Initializing cuVSLAM.");

  if (node.enable_ground_constraint_in_odometry_ &&
    !InitGroundConstraint(&ground_constraint_handle, node.get_logger()))
  {
    return;
  }

  // Set the base frame.
  std::string base_frame;
  if (!node.base_frame_.empty()) {
    base_frame = node.base_frame_;
  } else {
    base_frame = initial_camera_info_messages.at(0).value()->header.frame_id;
  }

  // Set the camera frames, either from parameters or from camera info message.
  std::vector<std::string> camera_optical_frames;
  if (!node.camera_optical_frames_.empty()) {
    camera_optical_frames = node.camera_optical_frames_;
  } else {
    camera_optical_frames.reserve(node.num_cameras_);
    for (const auto & [idx, camera_info_msg] : initial_camera_info_messages) {
      camera_optical_frames.push_back(camera_info_msg.value()->header.frame_id);
    }
  }

  // Setup camera extrinsics (and intrinsics).
  cuvslam_cameras.resize(node.num_cameras_);
  intrinsics.resize(node.num_cameras_);

  for (const auto & [idx, camera_info_msg] : initial_camera_info_messages) {
    const rclcpp::Time stamp(camera_info_msg.value()->header.stamp);
    cuvslam_cameras[idx].parameters = intrinsics[idx].data();
    FillIntrinsics(camera_info_msg.value(), cuvslam_cameras[idx]);
    const tf2::Transform base_link_pose_camera_optical = GetFrameTransform(
      stamp, base_frame,
      camera_optical_frames[idx]);
    if (node.rectified_images_) {
      const tf2::Matrix3x3 rectification_matrix(
        camera_info_msg.value()->r[0], camera_info_msg.value()->r[1], camera_info_msg.value()->r[2],
        camera_info_msg.value()->r[3], camera_info_msg.value()->r[4], camera_info_msg.value()->r[5],
        camera_info_msg.value()->r[6], camera_info_msg.value()->r[7], camera_info_msg.value()->r[8]
      );
      const tf2::Transform camera_optical_pose_camera_optical_rectified(
        rectification_matrix.inverse());
      const tf2::Transform base_link_pose_camera_rectified_optical = base_link_pose_camera_optical *
        camera_optical_pose_camera_optical_rectified;
      FillExtrinsics(base_link_pose_camera_rectified_optical, cuvslam_cameras[idx]);
    } else {
      FillExtrinsics(base_link_pose_camera_optical, cuvslam_cameras[idx]);
    }
  }


  CUVSLAM_CameraRig cam_rig;
  cam_rig.cameras = cuvslam_cameras.data();
  cam_rig.num_cameras = cuvslam_cameras.size();
  uint32_t border_mask_top = node.border_mask_top_;
  uint32_t border_mask_bottom = node.border_mask_bottom_;
  uint32_t border_mask_left = node.border_mask_left_;
  uint32_t border_mask_right = node.border_mask_right_;

  for (int i = 0; i < cam_rig.num_cameras; i++) {
    auto roi = initial_camera_info_messages.at(i).value()->roi;
    uint32_t img_width = initial_camera_info_messages.at(i).value()->width;
    uint32_t img_height = initial_camera_info_messages.at(i).value()->height;

    if (roi.x_offset == 0 && roi.y_offset == 0 && roi.width == 0 && roi.height == 0) {
      // The default setting of roi (all values 0) is considered the same as
      // full resolution (roi.width = width, roi.height = height).
      roi.width = img_width;
      roi.height = img_height;
    }

    auto & cam = cuvslam_cameras[i];

    if (!node.rectified_images_ && (border_mask_top != 0 || border_mask_bottom != 0 ||
      border_mask_left != 0 || border_mask_right != 0))
    {
      cam.border_top = std::clamp(border_mask_top, 0U, img_height);
      cam.border_bottom = std::clamp(border_mask_bottom, 0U, img_height);
      if (i % 2 == 0) {
        cam.border_left = std::clamp(border_mask_left, 0U, img_width);
        cam.border_right = std::clamp(border_mask_right, 0U, img_width);
      } else {
        cam.border_left = std::clamp(border_mask_right, 0U, img_width);
        cam.border_right = std::clamp(border_mask_left, 0U, img_width);
      }
    } else {
      cam.border_top = std::clamp(roi.y_offset, 0U, img_height);
      cam.border_bottom = std::clamp(img_height - roi.y_offset - roi.height, 0U, img_height);
      cam.border_left = std::clamp(roi.x_offset, 0U, img_width);
      cam.border_right = std::clamp(img_width - roi.x_offset - roi.width, 0U, img_width);
    }
  }

  // Create the IMU configuration. If the IMU is disabled we just use the identity transform.
  tf2::Transform cv_base_link_pose_cv_imu;
  cv_base_link_pose_cv_imu.setIdentity();
  if (node.enable_imu_fusion_) {
    const std::string imu_frame =
      !node.imu_frame_.empty() ? node.imu_frame_ : initial_imu_message.value()->header.frame_id;

    // Convert the base_pose_imu from ROS to cuVSLAM frame
    const rclcpp::Time stamp(initial_imu_message.value()->header.stamp);
    cv_base_link_pose_cv_imu =
      ChangeBasis(cuvslam_pose_canonical, GetFrameTransform(stamp, base_frame, imu_frame));
  }

  // Create cuvslam tracker.
  const CUVSLAM_Configuration configuration =
    CreateConfiguration(TocuVSLAMPose(cv_base_link_pose_cv_imu));
  PrintConfiguration(node.get_logger(), configuration);

  Stopwatch stopwatch_tracker;
  StopwatchScope ssw_tracker(stopwatch_tracker);
  CUVSLAM_TrackerHandle tracker;
  const CUVSLAM_Status status = CUVSLAM_CreateTracker(&tracker, &cam_rig, &configuration);

  RCLCPP_INFO(node.get_logger(), "Time taken by CUVSLAM_CreateTracker(): %f", ssw_tracker.Stop());
  if (status != CUVSLAM_SUCCESS) {
    RCLCPP_ERROR(node.get_logger(), "Failed to initialize CUVSLAM tracker: %d", status);
    return;
  }

  pose_cache.Reset();
  velocity_cache.Reset();

  // Initialize visualization helpers.
  if (node.enable_slam_visualization_) {
    observations_vis_helper.Init(
      node.vis_observations_pub_, tracker, canonical_pose_cuvslam, node.map_frame_);
    landmarks_vis_helper.Init(
      node.vis_landmarks_pub_, tracker, canonical_pose_cuvslam, node.map_frame_);
    lc_landmarks_vis_helper.Init(
      node.vis_loop_closure_pub_, tracker, canonical_pose_cuvslam, node.map_frame_);
    pose_graph_helper.Init(
      node.vis_posegraph_nodes_pub_, node.vis_posegraph_edges_pub_,
      node.vis_posegraph_edges2_pub_, tracker, canonical_pose_cuvslam, node.map_frame_);
    localizer_helper.Init(
      node.vis_localizer_pub_, tracker, canonical_pose_cuvslam, node.map_frame_);
    localizer_landmarks_vis_helper.Init(
      node.vis_localizer_landmarks_pub_, tracker, canonical_pose_cuvslam, node.map_frame_);
    localizer_observations_vis_helper.Init(
      node.vis_localizer_observations_pub_, tracker, canonical_pose_cuvslam, node.map_frame_);
    localizer_lc_landmarks_vis_helper.Init(
      node.vis_localizer_loop_closure_pub_, tracker, canonical_pose_cuvslam, node.map_frame_);
  }
  cuvslam_handle = tracker;
  RCLCPP_INFO(node.get_logger(), "cuVSLAM tracker was successfully initialized.");
}

void VisualSlamNode::VisualSlamImpl::Exit()
{
  observations_vis_helper.Exit();
  landmarks_vis_helper.Exit();
  lc_landmarks_vis_helper.Exit();
  pose_graph_helper.Exit();
  localizer_helper.Exit();
  localizer_landmarks_vis_helper.Exit();
  localizer_observations_vis_helper.Exit();
  localizer_lc_landmarks_vis_helper.Exit();

  if (cuvslam_handle != nullptr) {
    CUVSLAM_DestroyTracker(cuvslam_handle);
    cuvslam_handle = nullptr;
    RCLCPP_INFO(node.get_logger(), "cuVSLAM tracker was destroyed");
  }

  if (ground_constraint_handle != nullptr) {
    CUVSLAM_GroundConstraintDestroy(ground_constraint_handle);
    ground_constraint_handle = nullptr;
    RCLCPP_INFO(node.get_logger(), "cuVSLAM ground constraint was destroyed");
  }

  intrinsics.clear();

  pose_cache.Reset();
  velocity_cache.Reset();

  initial_imu_message.reset();
  initial_camera_info_messages.clear();
}

CUVSLAM_Configuration VisualSlamNode::VisualSlamImpl::CreateConfiguration(
  const CUVSLAM_Pose & cv_base_link_pose_cv_imu)
{
  CUVSLAM_Configuration configuration;
  CUVSLAM_InitDefaultConfiguration(&configuration);
  configuration.multicam_mode = node.multicam_mode_;
  configuration.use_motion_model = 1;
  configuration.use_denoising = node.enable_image_denoising_ ? 1 : 0;
  configuration.horizontal_stereo_camera = node.rectified_images_ ? 1 : 0;
  configuration.enable_observations_export = node.enable_observations_view_ ? 1 : 0;
  if (node.enable_debug_mode_) {
    configuration.debug_dump_directory = node.debug_dump_path_.c_str();
  }
  configuration.enable_landmarks_export = node.enable_landmarks_view_ ? 1 : 0;
  configuration.enable_localization_n_mapping = node.enable_localization_n_mapping_ ? 1 : 0;
  configuration.enable_reading_slam_internals = node.enable_slam_visualization_ ? 1 : 0;
  configuration.slam_sync_mode = 0;
  configuration.enable_imu_fusion = node.enable_imu_fusion_ ? 1 : 0;
  configuration.debug_imu_mode = node.enable_debug_imu_mode_ ? 1 : 0;
  configuration.max_frame_delta_ms = node.image_jitter_threshold_ms_;
  configuration.planar_constraints = node.enable_ground_constraint_in_slam_ ? 1 : 0;

  CUVSLAM_ImuCalibration imu_calibration;
  imu_calibration.rig_from_imu = cv_base_link_pose_cv_imu;
  imu_calibration.gyroscope_noise_density = node.imu_params_.gyroscope_noise_density;
  imu_calibration.gyroscope_random_walk = node.imu_params_.gyroscope_random_walk;
  imu_calibration.accelerometer_noise_density = node.imu_params_.accelerometer_noise_density;
  imu_calibration.accelerometer_random_walk = node.imu_params_.accelerometer_random_walk;
  imu_calibration.frequency = node.imu_params_.calibration_frequency;
  configuration.imu_calibration = imu_calibration;

  return configuration;
}

// Helper function to publish source frame pose wrt target frame to the tf tree
void VisualSlamNode::VisualSlamImpl::PublishFrameTransform(
  rclcpp::Time stamp, const tf2::Transform & pose,
  const std::string & target, const std::string & source)
{
  geometry_msgs::msg::TransformStamped target_pose_source;
  target_pose_source.header.stamp = stamp;
  target_pose_source.header.frame_id = target;
  target_pose_source.child_frame_id = source;
  target_pose_source.transform = tf2::toMsg(pose);
  tf_publisher->sendTransform(target_pose_source);
}

// Helper function to get child frame pose wrt parent frame from the tf tree
tf2::Transform VisualSlamNode::VisualSlamImpl::GetFrameTransform(
  rclcpp::Time stamp, const std::string & target, const std::string & source)
{
  geometry_msgs::msg::TransformStamped transform_stamped;
  tf2::Transform pose;

  // Time out duration for TF tree lookup before throwing an exception.
  constexpr int32_t kTimeOutSeconds = 1;
  constexpr uint32_t kTimeOutNanoSeconds = 0;
  try {
    if (!tf_buffer->canTransform(
        target, source, stamp, rclcpp::Duration(kTimeOutSeconds, kTimeOutNanoSeconds)))
    {
      RCLCPP_ERROR(
        node.get_logger(), "Transform is impossible. canTransform(%s->%s) returns false",
        target.c_str(), source.c_str());
    }
    transform_stamped = tf_buffer->lookupTransform(
      target, source, stamp, rclcpp::Duration(kTimeOutSeconds, kTimeOutNanoSeconds));
    tf2::fromMsg(transform_stamped.transform, pose);
  } catch (tf2::TransformException & ex) {
    RCLCPP_INFO(
      node.get_logger(), "Could not transform %s to %s: %s",
      source.c_str(), target.c_str(), ex.what());
    throw std::runtime_error("Could not find the requested transform!");
  }
  return pose;
}

void VisualSlamNode::VisualSlamImpl::PublishOdometryVelocity(
  rclcpp::Time stamp, const std::string & frame_id,
  const rclcpp::Publisher<MarkerArrayType>::SharedPtr publisher)
{
  geometry_msgs::msg::Twist twist;

  pose_cache.GetVelocity(
    twist.linear.x, twist.linear.y, twist.linear.z,
    twist.angular.x, twist.angular.y, twist.angular.z);

  MarkerArrayType markers;
  markers.markers.resize(2);

  float width = 0.02f;
  {
    MarkerType & m = markers.markers[0];
    m.header.frame_id = frame_id;
    m.header.stamp = stamp;
    m.ns = "linear velocity";
    m.id = 0;
    m.action = MarkerType::ADD;
    m.type = MarkerType::ARROW;
    m.pose.position.x = 0;
    m.pose.position.y = 0;
    m.pose.position.z = 0;
    m.pose.orientation.x = 0;
    m.pose.orientation.y = 0;
    m.pose.orientation.z = 0;
    m.pose.orientation.w = 1;
    m.scale.x = width * 1;
    m.scale.y = width * 2;
    m.scale.z = 0.0;
    m.color.a = 0.6;
    m.color.r = 0.8;
    m.color.g = 0.2;
    m.color.b = 0.3;
    geometry_msgs::msg::Point p1;
    p1.x = 0;
    p1.y = 0;
    p1.z = 0;
    geometry_msgs::msg::Point p2;
    p2.x = twist.linear.x;
    p2.y = twist.linear.y;
    p2.z = twist.linear.z;
    m.points.resize(2);
    m.points[0] = p1;
    m.points[1] = p2;
  }
  {
    MarkerType & m = markers.markers[1];
    m.header.frame_id = frame_id;
    m.header.stamp = stamp;
    m.ns = "angular velocity";
    m.id = 0;
    m.action = MarkerType::ADD;
    m.type = MarkerType::ARROW;
    m.pose.position.x = 0;
    m.pose.position.y = 0;
    m.pose.position.z = 0;
    m.pose.orientation.x = 0;
    m.pose.orientation.y = 0;
    m.pose.orientation.z = 0;
    m.pose.orientation.w = 1;
    m.scale.x = width * 1;
    m.scale.y = width * 2;
    m.scale.z = 0.0;
    m.color.a = 1;
    m.color.r = 0.2;
    m.color.g = 0.8;
    m.color.b = 0.3;
    geometry_msgs::msg::Point p1;
    p1.x = 0;
    p1.y = 0;
    p1.z = 0;
    geometry_msgs::msg::Point p2;
    p2.x = twist.angular.x;
    p2.y = twist.angular.y;
    p2.z = twist.angular.z;
    m.points.resize(2);
    m.points[0] = p1;
    m.points[1] = p2;
  }
  publisher->publish(markers);
}

void VisualSlamNode::VisualSlamImpl::PublishGravity(
  rclcpp::Time stamp, const std::string & frame_id,
  const rclcpp::Publisher<MarkerType>::SharedPtr publisher)
{
  CUVSLAM_Gravity gravity_in_cuvslam;

  if (CUVSLAM_SUCCESS != CUVSLAM_GetLastGravity(cuvslam_handle, &gravity_in_cuvslam)) {
    // Don't publish anything if cuvslam hasn't calculated gravity yet. After the start
    // or loss of visual tracking, cuvslam needs some time to align optical and imu sensors.
    // Only after that is it able to calculate the gravity vector.
    return;
  }

  const tf2::Vector3 g_cuvslam(gravity_in_cuvslam.x, gravity_in_cuvslam.y, gravity_in_cuvslam.z);
  const tf2::Vector3 g_base_link = canonical_pose_cuvslam * g_cuvslam;

  MarkerType m;
  m.header.frame_id = frame_id;
  m.header.stamp = stamp;
  m.ns = "gravity";
  m.id = 0;
  m.action = MarkerType::ADD;
  m.type = MarkerType::ARROW;
  m.pose.position.x = 0;    // arrow start
  m.pose.position.y = 0;
  m.pose.position.z = 0;
  m.pose.orientation.x = 0;
  m.pose.orientation.y = 0;
  m.pose.orientation.z = 0;
  m.pose.orientation.w = 1;
  m.scale.x = 0.1;    // length
  m.scale.y = 0.2;
  m.scale.z = 0.0;
  m.color.a = 0.6;
  m.color.r = 1.0;
  m.color.g = 1.0;
  m.color.b = 1.0;
  geometry_msgs::msg::Point p1;
  p1.x = 0;
  p1.y = 0;
  p1.z = 0;
  geometry_msgs::msg::Point p2;
  p2.x = g_base_link.x();
  p2.y = g_base_link.y();
  p2.z = g_base_link.z();
  m.points.resize(2);
  m.points[0] = p1;
  m.points[1] = p2;

  publisher->publish(std::move(m));
}


void VisualSlamNode::VisualSlamImpl::CallbackImu(const ImuType::ConstSharedPtr & msg)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "VisualSlamNode::VisualSlamImpl::CallbackImu", nvidia::isaac_ros::nitros::CLR_RED);
  if (IsInitialized()) {
    const rclcpp::Time timestamp(msg->header.stamp);
    sequencer.CallbackStream1(timestamp.nanoseconds(), msg);
  } else {initial_imu_message = msg;}
  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}

void VisualSlamNode::VisualSlamImpl::CallbackImage(int index, const ImageType & image_view)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "VisualSlamNode::VisualSlamImpl::CallbackImage",
    nvidia::isaac_ros::nitros::CLR_YELLOW);

  if (IsInitialized()) {
    const rclcpp::Time timestamp = NitrosTimeStamp::value(image_view.GetMessage());
    sync.AddMessage(index, timestamp.nanoseconds(), image_view);
  }
  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}

void VisualSlamNode::VisualSlamImpl::CallbackCameraInfo(
  int index, const CameraInfoType::ConstSharedPtr & msg)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "VisualSlamNode::VisualSlamImpl::CallbackCameraInfo", nvidia::isaac_ros::nitros::CLR_YELLOW);

  if (!IsInitialized()) {
    initial_camera_info_messages[index] = msg;
    if (IsReadyForInitialization()) {Initialize();}
  }

  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}

void VisualSlamNode::VisualSlamImpl::CallbackSynchronizedImages(
  int64_t latest_ts, const std::vector<std::pair<int, ImageType>> & idx_and_image_msgs)
{
  sequencer.CallbackStream2(latest_ts, idx_and_image_msgs);
}

void VisualSlamNode::VisualSlamImpl::UpdatePose(
  const std::vector<ImuType::ConstSharedPtr> & imu_msgs,
  const std::vector<std::pair<int, ImageType>> & idx_and_image_msgs)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "VisualSlamNode::VisualSlamImpl::UpdatePose", nvidia::isaac_ros::nitros::CLR_MAGENTA);
  Stopwatch stopwatch;
  StopwatchScope ssw(stopwatch);

  // Get the latest timestamp from images. We assume that the vector is never empty.
  const auto max_element = std::max_element(
    idx_and_image_msgs.begin(), idx_and_image_msgs.end(),
    [](std::pair<int, ImageType> msg1, std::pair<int, ImageType> msg2) {
      auto ts1 = NitrosTimeStamp::value(msg1.second.GetMessage());
      auto ts2 = NitrosTimeStamp::value(msg2.second.GetMessage());
      return ts1.nanoseconds() < ts2.nanoseconds();
    });

  const int64_t latest_ts = NitrosTimeStamp::value(max_element->second.GetMessage()).nanoseconds();

  RCLCPP_DEBUG(node.get_logger(), "Using image msg timestamp [%ld]", latest_ts);

  const double time_delta_ms = (latest_ts - last_track_ts) / 1e6;
  // Skipping the calculation for the first frame
  if (time_delta_ms > node.image_jitter_threshold_ms_ && last_track_ts != -1) {
    RCLCPP_WARN(
      node.get_logger(),
      "Delta between current and previous frame [%f ms] is above threshold [%f ms]",
      time_delta_ms, node.image_jitter_threshold_ms_);
  }

  Stopwatch stopwatch_track;
  // First we add all imu measurements received since the last update.
  for (const auto & imu_msg : imu_msgs) {
    const auto imu_measurement = TocuVSLAMImuMeasurement(imu_msg);
    const rclcpp::Time timestamp(imu_msg->header.stamp);
    int64_t imu_ts = static_cast<int64_t>(timestamp.nanoseconds());
    RCLCPP_DEBUG(node.get_logger(), "Using imu msg timestamp [%ld]", imu_ts);
    const CUVSLAM_Status status = CUVSLAM_RegisterImuMeasurement(
      cuvslam_handle, latest_ts, &imu_measurement);
    if (status != CUVSLAM_SUCCESS) {
      RCLCPP_WARN(node.get_logger(), "CUVSLAM has failed to register an IMU measurement");
    }
  }

  // Convert images to cuvslam's format.
  std::vector<CUVSLAM_Image> cuvslam_images;
  cuvslam_images.reserve(idx_and_image_msgs.size());

  for (const auto & [idx, image_msg] : idx_and_image_msgs) {
    cuvslam_images.push_back(TocuVSLAMImage(idx, image_msg, latest_ts));
  }

  StopwatchScope ssw_track(stopwatch_track);
  CUVSLAM_PoseEstimate vo_pose_estimate;
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "CUVSLAM_Track", nvidia::isaac_ros::nitros::CLR_MAGENTA);
  const CUVSLAM_Status vo_status =
    CUVSLAM_TrackGpuMem(
    cuvslam_handle, cuvslam_images.data(), cuvslam_images.size(), nullptr,
    &vo_pose_estimate);
  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
  ssw_track.Stop();

  if (vo_status == CUVSLAM_TRACKING_LOST) {
    pose_cache.Reset();
    RCLCPP_WARN(node.get_logger(), "Visual tracking is lost");
    nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
  } else if (vo_status != CUVSLAM_SUCCESS) {
    RCLCPP_WARN(node.get_logger(), "Unknown Tracker Error %d", vo_status);
    nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
    return;
  }

  const rclcpp::Time timestamp_output = node.override_publishing_stamp_ ?
    node.get_clock()->now() : rclcpp::Time(latest_ts);

  if (vo_status == CUVSLAM_SUCCESS) {
    if (ground_constraint_handle &&
      !PoseToGround(ground_constraint_handle, vo_pose_estimate.pose))
    {
      RCLCPP_WARN(node.get_logger(), "Unknown ground constraint Error %d", vo_status);
      nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
      return;
    }

    // Get VO pose and transform to ROS conventions.
    // odom and map are using same conventions.
    const tf2::Transform cv_odom_pose_cv_base_link = FromcuVSLAMPose(vo_pose_estimate.pose);
    const tf2::Transform odom_pose_base_link = ChangeBasis(
      canonical_pose_cuvslam, cv_odom_pose_cv_base_link);

    // Get SLAM pose and transform to ROS conventions. The SLAM pose uses loop closures for robust
    // tracking.
    // Note: It can result in sudden jumps of the final pose.
    // Publish Smooth pose if enable_rectified_pose_ = false
    tf2::Transform cv_map_pose_cv_base_link = cv_odom_pose_cv_base_link;
    if (node.enable_localization_n_mapping_) {
      CUVSLAM_Pose slam_pose;
      const CUVSLAM_Status slam_pose_status = CUVSLAM_GetSlamPose(cuvslam_handle, &slam_pose);
      if (slam_pose_status != CUVSLAM_SUCCESS) {
        RCLCPP_WARN(node.get_logger(), "Get Slam Pose Error %d", slam_pose_status);
        nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
        return;
      }
      cv_map_pose_cv_base_link = FromcuVSLAMPose(slam_pose);
    }

    const tf2::Transform map_pose_base_link =
      ChangeBasis(canonical_pose_cuvslam, cv_map_pose_cv_base_link);
    const tf2::Transform map_pose_odom = map_pose_base_link * odom_pose_base_link.inverse();

    // Publish transforms to the TF tree.
    if (node.publish_map_to_odom_tf_) {
      if (!node.invert_map_to_odom_tf_) {
        PublishFrameTransform(
          timestamp_output, map_pose_odom, node.map_frame_,
          node.odom_frame_);
      } else {
        PublishFrameTransform(
          timestamp_output,
          map_pose_odom.inverse(), node.odom_frame_, node.map_frame_);
      }
    }

    if (node.publish_odom_to_base_tf_) {
      if (!node.invert_odom_to_base_tf_) {
        PublishFrameTransform(
          timestamp_output, odom_pose_base_link, node.odom_frame_, node.base_frame_);
      } else {
        PublishFrameTransform(
          timestamp_output, odom_pose_base_link.inverse(), node.base_frame_, node.odom_frame_);
      }
    }

    // Calculate velocity using last position.
    pose_cache.Add(latest_ts, odom_pose_base_link);
    geometry_msgs::msg::Twist velocity;
    pose_cache.GetVelocity(
      velocity.linear.x, velocity.linear.y, velocity.linear.z,
      velocity.angular.x, velocity.angular.y, velocity.angular.z);
    velocity_cache.Add(
      velocity.linear.x, velocity.linear.y, velocity.linear.z,
      velocity.angular.x, velocity.angular.y, velocity.angular.z);

    // Prepare message parts needed for VO messages.
    PoseType vo_pose;
    tf2::toMsg(odom_pose_base_link, vo_pose);
    std_msgs::msg::Header header_odom;
    header_odom.stamp = timestamp_output;
    header_odom.frame_id = node.odom_frame_;

    // Prepare message parts needed for SLAM messages.
    PoseType slam_pose;
    tf2::toMsg(map_pose_base_link, slam_pose);
    std_msgs::msg::Header header_map;
    header_map.stamp = timestamp_output;
    header_map.frame_id = node.map_frame_;

    if (HasSubscribers(node.tracking_vo_pose_pub_)) {
      // Tracking_vo_pose_pub_
      auto pose_only = std::make_unique<PoseStampedType>();
      pose_only->header = header_odom;
      pose_only->pose = vo_pose;
      node.tracking_vo_pose_pub_->publish(std::move(pose_only));
    }
    if (HasSubscribers(node.tracking_vo_pose_covariance_pub_)) {
      // Tracking_vo_covariance_pub_
      auto pose_n_cov = std::make_unique<PoseWithCovarianceStampedType>();
      pose_n_cov->header = header_odom;
      pose_n_cov->pose.pose = vo_pose;
      auto covariance_transform = FromcuVSLAMCovariance(vo_pose_estimate.covariance);
      for (size_t i = 0; i < 6 * 6; i++) {
        pose_n_cov->pose.covariance[i] = covariance_transform(i);
      }
      node.tracking_vo_pose_covariance_pub_->publish(std::move(pose_n_cov));
    }
    if (HasSubscribers(node.tracking_odometry_pub_)) {
      OdometryType odom;
      odom.header = header_odom;
      odom.child_frame_id = node.base_frame_;

      odom.pose.pose = vo_pose;
      bool res = pose_cache.GetCovariance(odom.pose.covariance);
      if (!res) {
        // set default Identity matrix
        for (int i = 0; i < 6; i++) {
          for (int j = 0; j < 6; j++) {
            odom.pose.covariance[i * 6 + j] = i == j ? 1 : 0;
          }
        }
      }

      odom.twist.twist.linear = velocity.linear;
      odom.twist.twist.angular = velocity.angular;

      res = velocity_cache.GetCovariance(odom.twist.covariance);
      if (!res) {
        // set default Identity matrix
        for (int i = 0; i < 6; i++) {
          for (int j = 0; j < 6; j++) {
            odom.twist.covariance[i * 6 + j] = i == j ? 1 : 0;
          }
        }
      }
      node.tracking_odometry_pub_->publish(odom);
    }
    if (HasSubscribers(node.vis_vo_velocity_pub_)) {
      PublishOdometryVelocity(
        timestamp_output,
        node.base_frame_,
        node.vis_vo_velocity_pub_);
    }
    if (HasSubscribers(node.vis_slam_odometry_pub_)) {
      OdometryType odom;
      odom.header = header_map;
      odom.child_frame_id = node.base_frame_;

      // only populating slam_pose for viz
      odom.pose.pose = slam_pose;
      node.vis_slam_odometry_pub_->publish(odom);
    }
    if (HasSubscribers(node.tracking_vo_path_pub_)) {
      // Tracking_vo_path_pub_
      PoseStampedType pose_stamped;
      pose_stamped.header = header_odom;
      pose_stamped.pose = vo_pose;
      vo_path.add(pose_stamped);

      auto path_msg = std::make_unique<PathType>();
      path_msg->header = header_odom;
      path_msg->poses = vo_path.getData();
      node.tracking_vo_path_pub_->publish(std::move(path_msg));
    }
    if (HasSubscribers(node.tracking_slam_path_pub_)) {
      // Tracking_slam_path_pub_
      PoseStampedType pose_stamped;
      pose_stamped.header = header_map;
      pose_stamped.pose = slam_pose;
      slam_path.add(pose_stamped);

      auto path_msg = std::make_unique<PathType>();
      path_msg->header = header_map;
      path_msg->poses = slam_path.getData();
      node.tracking_slam_path_pub_->publish(std::move(path_msg));
    }

    // Draw gravity vector
    if (HasSubscribers(node.vis_gravity_pub_) && node.enable_imu_fusion_) {
      PublishGravity(
        timestamp_output,
        node.base_frame_,
        node.vis_gravity_pub_);
    }
  }

  // Calculate the tracking execution time statistics.
  ssw.Stop();
  const double track_execution_time = stopwatch_track.Seconds();
  track_execution_times.add(track_execution_time);
  double track_execution_time_max = 0;
  double track_execution_time_mean = 0;
  const auto & values = track_execution_times.getData();
  for (double t : values) {
    track_execution_time_max = std::max(track_execution_time_max, t);
    track_execution_time_mean += t;
  }
  if (values.size()) {
    track_execution_time_mean /= values.size();
  }
  // Publish status.
  std_msgs::msg::Header header;
  header.stamp = timestamp_output;
  header.frame_id = node.map_frame_;
  if (HasSubscribers(node.visual_slam_status_pub_)) {
    VisualSlamStatusType visual_slam_status_msg;
    visual_slam_status_msg.header = header;
    visual_slam_status_msg.vo_state = vo_status == CUVSLAM_SUCCESS ? 1 : 2;
    visual_slam_status_msg.node_callback_execution_time = stopwatch.Seconds();
    visual_slam_status_msg.track_execution_time = track_execution_time;
    visual_slam_status_msg.track_execution_time_max = track_execution_time_max;
    visual_slam_status_msg.track_execution_time_mean = track_execution_time_mean;
    node.visual_slam_status_pub_->publish(visual_slam_status_msg);
  }
  // Publish diagnostics.
  if (HasSubscribers(node.diagnostics_pub_)) {
    DiagnosticArrayType diagnostics;
    diagnostics.header = header;
    DiagnosticStatusType & status = diagnostics.status.emplace_back();
    if (vo_status == CUVSLAM_SUCCESS) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    }
    status.name = "Visual Slam Diagnostics";
    status.message = "Tracking state and execution time measurements";
    status.hardware_id = "visual_slam";
    status.values.emplace_back();
    status.values.back().key = "vo_status";
    status.values.back().value = std::to_string(vo_status);
    status.values.emplace_back();
    status.values.back().key = "track_execution_time";
    status.values.back().value = std::to_string(track_execution_time);
    status.values.emplace_back();
    status.values.back().key = "track_execution_time_max";
    status.values.back().value = std::to_string(track_execution_time_max);
    status.values.emplace_back();
    status.values.back().key = "track_execution_time_mean";
    status.values.back().value = std::to_string(track_execution_time_mean);
    node.diagnostics_pub_->publish(diagnostics);
  }

  last_track_ts = latest_ts;
  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}

// Asynchronous response for CUVSLAM_SaveToSlamDb()
void VisualSlamNode::VisualSlamImpl::SaveToSlamDbResponse(
  void * cnt,
  CUVSLAM_Status status)
{
  SaveToSlamDbContext * context = static_cast<SaveToSlamDbContext *>(cnt);

  auto result = std::make_shared<ActionSaveMap::Result>();
  result->success = false;

  if (status == CUVSLAM_SUCCESS) {
    // publish_feedback
    if (context->goal_handle) {
      result->success = true;
      context->goal_handle->succeed(result);
    }
  } else {
    // abort
    if (context->goal_handle) {
      context->goal_handle->abort(result);
    }
  }

  delete context;
}

// Asynchronous response for CUVSLAM_LocalizeInExistDb()
void VisualSlamNode::VisualSlamImpl::LocalizeInExistDbResponse(
  void * cnt, CUVSLAM_Status status, const struct CUVSLAM_Pose * pose_in_db)
{
  LocalizeInExistDbContext * context = static_cast<LocalizeInExistDbContext *>(cnt);

  auto result = std::make_shared<ActionLoadMapAndLocalize::Result>();
  result->success = false;

  VisualSlamNode::VisualSlamImpl * impl = context->impl;
  if (status == CUVSLAM_SUCCESS && impl) {
    if (!pose_in_db) {
      // abort
      if (context->goal_handle) {
        context->goal_handle->abort(result);
      }
      RCLCPP_ERROR(
        impl->node.get_logger(), "Wrong arg in LocalizeInExistDbResponse())");
      delete context;
      return;
    }

    const tf2::Transform cv_map_pose_cv_base_link = FromcuVSLAMPose(*pose_in_db);
    const tf2::Transform map_pose_base_link =
      ChangeBasis(canonical_pose_cuvslam, cv_map_pose_cv_base_link);

    // Publish feedback
    if (context->goal_handle) {
      tf2::toMsg(map_pose_base_link, result->pose);
      result->success = true;
      context->goal_handle->succeed(result);
    }

    // To localizer_helper
    const auto pt = map_pose_base_link.getOrigin();
    RCLCPP_INFO(impl->node.get_logger(), "Localization result {%f, %f, %f}", pt[0], pt[1], pt[2]);
    impl->localizer_helper.SetResult(true, map_pose_base_link);
  } else {
    // Abort
    if (context->goal_handle) {
      context->goal_handle->abort(result);
    }
    // To localizer_helper
    if (impl) {
      impl->localizer_helper.SetResult(false, tf2::Transform());
      RCLCPP_WARN(impl->node.get_logger(), "Localization failed. Status=%d", status);
    }
  }
  delete context;
}

}  // namespace visual_slam
}  // namespace isaac_ros
}  // namespace nvidia
