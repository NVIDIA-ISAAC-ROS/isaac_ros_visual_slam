// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2021-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "isaac_ros_visual_slam/visual_slam_node.hpp"
#include "isaac_ros_visual_slam/impl/visual_slam_impl.hpp"
#include "isaac_ros_visual_slam/impl/elbrus_ros_convertion.hpp"
#include "isaac_ros_visual_slam/impl/posegraph_vis_helper.hpp"
#include "isaac_ros_visual_slam/impl/has_subscribers.hpp"
#include "isaac_ros_visual_slam/impl/stopwatch.hpp"
#include "isaac_ros_visual_slam/impl/qos.hpp"

namespace isaac_ros
{
namespace visual_slam
{

VisualSlamNode::VisualSlamNode(rclcpp::NodeOptions options)
: Node("visual_slam", options),
  // Node parameters.
  denoise_input_images_(declare_parameter<bool>("denoise_input_images", false)),
  rectified_images_(declare_parameter<bool>("rectified_images", true)),
  enable_imu_(declare_parameter<bool>("enable_imu", false)),
  enable_observations_view_(declare_parameter<bool>("enable_observations_view", false)),
  enable_landmarks_view_(declare_parameter<bool>("enable_landmarks_view", false)),
  enable_debug_mode_(declare_parameter<bool>("enable_debug_mode", false)),
  debug_dump_path_(declare_parameter<std::string>("debug_dump_path", "/tmp/elbrus")),

  enable_localization_n_mapping_(declare_parameter<bool>("enable_localization_n_mapping", true)),
  enable_slam_visualization_(declare_parameter<bool>("enable_slam_visualization", false)),

  input_base_frame_(declare_parameter<std::string>("input_base_frame", "")),
  input_left_camera_frame_(declare_parameter<std::string>("input_left_camera_frame", "")),
  input_right_camera_frame_(declare_parameter<std::string>("input_right_camera_frame", "")),
  input_imu_frame_(declare_parameter<std::string>("input_imu_frame", "imu")),
  // This is the gravity vector in ROS coordinate frame.
  // Default value assumes that camera is horizontal to the floor.
  gravitational_force_(declare_parameter<std::vector<double>>(
      "gravitational_force",
      {0.0, 0, -9.8})),

  // frames hierarchy:
  // map - odom - base_link - ... - camera
  publish_odom_to_base_tf_(declare_parameter<bool>("publish_odom_to_base_tf", true)),
  publish_map_to_odom_tf_(declare_parameter<bool>("publish_map_to_odom_tf", true)),
  invert_odom_to_base_tf_(declare_parameter<bool>("invert_odom_to_base_tf", false)),
  invert_map_to_odom_tf_(declare_parameter<bool>("invert_map_to_odom_tf", false)),
  map_frame_(declare_parameter<std::string>("map_frame", "map")),
  odom_frame_(declare_parameter<std::string>("odom_frame", "odom")),
  base_frame_(declare_parameter<std::string>("base_frame", "base_link")),

  override_publishing_stamp_(declare_parameter<bool>("override_publishing_stamp", false)),

  path_max_size_(declare_parameter<int>("path_max_size", 1024)),

  msg_filter_queue_size_(declare_parameter<int>("msg_filter_queue_size", 100)),
  image_qos_(parseQosString(declare_parameter<std::string>("image_qos", "SENSOR_DATA"))),
  // Subscribers
  left_image_sub_(message_filters::Subscriber<sensor_msgs::msg::Image>(
      this, "stereo_camera/left/image", image_qos_)),
  left_camera_info_sub_(message_filters::Subscriber<sensor_msgs::msg::CameraInfo>(
      this, "stereo_camera/left/camera_info")),
  right_image_sub_(message_filters::Subscriber<sensor_msgs::msg::Image>(
      this, "stereo_camera/right/image", image_qos_)),
  right_camera_info_sub_(message_filters::Subscriber<sensor_msgs::msg::CameraInfo>(
      this, "stereo_camera/right/camera_info")),
  imu_sub_(create_subscription<sensor_msgs::msg::Imu>(
      "visual_slam/imu", rclcpp::QoS(100),
      std::bind(&VisualSlamNode::ReadImuData, this, std::placeholders::_1))),

  // Publishers: Visual SLAM
  visual_slam_status_pub_(
    create_publisher<isaac_ros_visual_slam_interfaces::msg::VisualSlamStatus>(
      "visual_slam/status", rclcpp::QoS(10))),
  tracking_vo_pose_pub_(
    create_publisher<geometry_msgs::msg::PoseStamped>(
      "visual_slam/tracking/vo_pose", rclcpp::QoS(100))),
  tracking_vo_pose_covariance_pub_(
    create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "visual_slam/tracking/vo_pose_covariance", rclcpp::QoS(100))),
  tracking_odometry_pub_(
    create_publisher<nav_msgs::msg::Odometry>(
      "visual_slam/tracking/odometry", rclcpp::QoS(100))),
  tracking_vo_path_pub_(
    create_publisher<nav_msgs::msg::Path>(
      "visual_slam/tracking/vo_path", rclcpp::QoS(100))),
  tracking_slam_path_pub_(
    create_publisher<nav_msgs::msg::Path>(
      "visual_slam/tracking/slam_path", rclcpp::QoS(100))),

  // Visualizators for odometry
  vis_observations_pub_(
    create_publisher<sensor_msgs::msg::PointCloud2>(
      "visual_slam/vis/observations_cloud", rclcpp::QoS(10))),
  vis_gravity_pub_(
    create_publisher<visualization_msgs::msg::Marker>(
      "visual_slam/vis/gravity", rclcpp::QoS(10))),
  vis_vo_velocity_pub_(
    create_publisher<visualization_msgs::msg::MarkerArray>(
      "visual_slam/vis/velocity", rclcpp::QoS(10))),

  // Visualizators for map and "loop closure"
  vis_landmarks_pub_(
    create_publisher<sensor_msgs::msg::PointCloud2>(
      "visual_slam/vis/landmarks_cloud", rclcpp::QoS(10))),
  vis_loop_closure_pub_(
    create_publisher<sensor_msgs::msg::PointCloud2>(
      "visual_slam/vis/loop_closure_cloud", rclcpp::QoS(10))),
  vis_posegraph_nodes_pub_(
    create_publisher<geometry_msgs::msg::PoseArray>(
      "visual_slam/vis/pose_graph_nodes", rclcpp::QoS(10))),
  vis_posegraph_edges_pub_(
    create_publisher<visualization_msgs::msg::Marker>(
      "visual_slam/vis/pose_graph_edges", rclcpp::QoS(10))),
  vis_posegraph_edges2_pub_(
    create_publisher<visualization_msgs::msg::Marker>(
      "visual_slam/vis/pose_graph_edges2", rclcpp::QoS(10))),

  // Visualizators for localization
  vis_localizer_pub_(
    create_publisher<visualization_msgs::msg::MarkerArray>(
      "visual_slam/vis/localizer", rclcpp::QoS(10))),
  vis_localizer_landmarks_pub_(
    create_publisher<sensor_msgs::msg::PointCloud2>(
      "visual_slam/vis/localizer_map_cloud", rclcpp::QoS(10))),
  vis_localizer_observations_pub_(
    create_publisher<sensor_msgs::msg::PointCloud2>(
      "visual_slam/vis/localizer_observations_cloud", rclcpp::QoS(10))),
  vis_localizer_loop_closure_pub_(
    create_publisher<sensor_msgs::msg::PointCloud2>(
      "visual_slam/vis/localizer_loop_closure_cloud", rclcpp::QoS(10))),

  // Slam Services
  reset_srv_(
    create_service<isaac_ros_visual_slam_interfaces::srv::Reset>(
      "visual_slam/reset", std::bind(&VisualSlamNode::CallbackReset,
      this, std::placeholders::_1, std::placeholders::_2))),
  get_all_poses_srv_(
    create_service<isaac_ros_visual_slam_interfaces::srv::GetAllPoses>(
      "visual_slam/get_all_poses", std::bind(&VisualSlamNode::CallbackGetAllPoses,
      this, std::placeholders::_1, std::placeholders::_2))),
  set_odometry_pose_srv_(create_service<isaac_ros_visual_slam_interfaces::srv::SetOdometryPose>(
      "visual_slam/set_odometry_pose", std::bind(&VisualSlamNode::CallbackSetOdometryPose,
      this, std::placeholders::_1, std::placeholders::_2))),

  // Initialize the impl
  impl_(std::make_unique<VisualSlamImpl>(*this))
{
  // Dump and check Elbrus API version
  int32_t elbrus_major, elbrus_minor;
  ELBRUS_GetVersion(&elbrus_major, &elbrus_minor);
  RCLCPP_INFO(get_logger(), "Elbrus version: %d.%d", elbrus_major, elbrus_minor);
  // VisualSlamNode is desined for this elbrus sdk version:
  int32_t exp_elbrus_major = 10, exp_elbrus_minor = 5;
  if (elbrus_major != exp_elbrus_major || elbrus_minor != exp_elbrus_minor) {
    RCLCPP_ERROR(
      get_logger(), "VisualSlamNode is designed to work with Elbrus SDK v%d.%d",
      exp_elbrus_major, exp_elbrus_minor);
    valid_elbrus_api_ = false;
  } else {
    valid_elbrus_api_ = true;
  }

  impl_->sync.reset(
    new VisualSlamImpl::Synchronizer(
      VisualSlamImpl::ExactTime(msg_filter_queue_size_),
      left_image_sub_,
      left_camera_info_sub_,
      right_image_sub_,
      right_camera_info_sub_
  ));
  impl_->sync->registerCallback(
    std::bind(
      &VisualSlamNode::TrackCameraPose, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

  load_map_and_localize_server_ =
    rclcpp_action::create_server<isaac_ros_visual_slam_interfaces::action::LoadMapAndLocalize>(
    this,
    "visual_slam/load_map_and_localize",
    std::bind(
      &VisualSlamNode::CallbackLoadMapAndLocalizeGoal, this, std::placeholders::_1,
      std::placeholders::_2),
    std::bind(
      &VisualSlamNode::CallbackLoadMapAndLocalizeCancel, this,
      std::placeholders::_1),
    std::bind(
      &VisualSlamNode::CallbackLoadMapAndLocalizeAccepted, this,
      std::placeholders::_1));

  save_map_server_ =
    rclcpp_action::create_server<isaac_ros_visual_slam_interfaces::action::SaveMap>(
    this,
    "visual_slam/save_map",
    std::bind(
      &VisualSlamNode::CallbackSaveMapGoal, this, std::placeholders::_1,
      std::placeholders::_2),
    std::bind(&VisualSlamNode::CallbackSaveMapCancel, this, std::placeholders::_1),
    std::bind(&VisualSlamNode::CallbackSaveMapAccepted, this, std::placeholders::_1));
}

VisualSlamNode::~VisualSlamNode()
{
  impl_.reset();
}


void VisualSlamNode::ReadImuData(const sensor_msgs::msg::Imu::ConstSharedPtr msg_imu)
{
  if (!valid_elbrus_api_) {
    exit(EXIT_FAILURE);
  }

  if (!impl_->IsInitialized() || !enable_imu_) {
    return;
  }
  const auto measurement = impl_->ToElbrusImuMeasurement(msg_imu);
  const rclcpp::Time timestamp(msg_imu->header.stamp.sec, msg_imu->header.stamp.nanosec);
  const int64_t acqtime_ns = static_cast<int64_t>(timestamp.nanoseconds());

  const ELBRUS_Status status = ELBRUS_RegisterImuMeasurement(
    impl_->elbrus_handle, acqtime_ns, &measurement);
  if (status != ELBRUS_SUCCESS) {
    RCLCPP_WARN(this->get_logger(), "ELBRUS has failed to register an IMU measurement");
  }
}

void VisualSlamNode::CallbackReset(
  const std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::Reset_Request> req,
  std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::Reset_Response> res)
{
  (void)req;

  RCLCPP_INFO(this->get_logger(), "elbrus: Reset");

  impl_->Exit();
  res->success = true;
}

// Asynchronous response for ELBRUS_SaveToSlamDb()
void VisualSlamNode::VisualSlamImpl::SaveToSlamDbResponse(
  void * cnt,
  ELBRUS_Status status)
{
  SaveToSlamDbContext * context = static_cast<SaveToSlamDbContext *>(cnt);

  auto result = std::make_shared<isaac_ros_visual_slam_interfaces::action::SaveMap::Result>();
  result->success = false;

  VisualSlamNode::VisualSlamImpl * impl =
    context->node ? context->node->impl_.get() : nullptr;
  if (status == ELBRUS_SUCCESS && impl) {
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
}

// Asynchronous response for ELBRUS_LocalizeInExistDb()
void VisualSlamNode::VisualSlamImpl::LocalizeInExistDbResponse(
  void * cnt,
  ELBRUS_Status status,
  const ELBRUS_Pose * pose_in_db)
{
  LocalizeInExistDbContext * context = static_cast<LocalizeInExistDbContext *>(cnt);

  auto result =
    std::make_shared<isaac_ros_visual_slam_interfaces::action::LoadMapAndLocalize::Result>();
  result->success = false;

  VisualSlamNode::VisualSlamImpl * impl =
    context->node ? context->node->impl_.get() : nullptr;
  if (status == ELBRUS_SUCCESS && impl) {
    if (!pose_in_db) {
      // abort
      if (context->goal_handle) {
        context->goal_handle->abort(result);
      }
      RCLCPP_ERROR(
        rclcpp::get_logger(
          impl->logger_name), "Wrong arg in LocalizeInExistDbResponse())");
      return;
    }
    const tf2::Transform ros_pos{ChangeBasis(
        impl->base_link_pose_elbrus, FromElbrusPose(
          *pose_in_db))};

    // publish_feedback
    if (context->goal_handle) {
      tf2::toMsg(ros_pos, result->pose);
      result->success = true;

      context->goal_handle->succeed(result);
    }

    // to localizer_helper
    const auto pt = ros_pos.getOrigin();
    RCLCPP_INFO(
      rclcpp::get_logger(
        impl->logger_name), "Localization result {%f, %f, %f}", pt[0], pt[1], pt[2]);
    impl->localizer_helper.SetResult(true, ros_pos);
  } else {
    // abort
    if (context->goal_handle) {
      context->goal_handle->abort(result);
    }
    // to localizer_helper
    if (impl) {
      impl->localizer_helper.SetResult(false, tf2::Transform());
      RCLCPP_WARN(rclcpp::get_logger(impl->logger_name), "Localization failed. Status=%d", status);
    }
  }
}

void VisualSlamNode::CallbackGetAllPoses(
  const std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::GetAllPoses_Request> req,
  std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::GetAllPoses_Response> res)
{
  res->success = false;

  RCLCPP_INFO(this->get_logger(), "elbrus: GetAllPoses");

  if (impl_->IsInitialized()) {
    // ELBRUS_GetAllPoses
    std::vector<ELBRUS_PoseStamped> elbrus_poses(req->max_count);
    uint32_t count = ELBRUS_GetAllPoses(
      impl_->elbrus_handle, req->max_count, elbrus_poses.data());
    if (count == 0) {
      RCLCPP_WARN(this->get_logger(), "ELBRUS_GetAllPoses Error");
    }
    res->poses.resize(count);

    for (uint32_t i = 0; i < count; i++) {
      const ELBRUS_Pose & elbrus_pose_elbrus = elbrus_poses[i].pose;
      tf2::Transform ros_pose_elbrus = FromElbrusPose(elbrus_pose_elbrus);
      tf2::Transform ros_pose_ros = ChangeBasis(impl_->base_link_pose_elbrus, ros_pose_elbrus);

      geometry_msgs::msg::Pose ros_pose_msg;
      tf2::toMsg(ros_pose_ros, ros_pose_msg);

      res->poses[i].pose = ros_pose_msg;
      res->poses[i].header.stamp = rclcpp::Time(elbrus_poses[i].timestamp_ns, RCL_ROS_TIME);
    }

    res->success = (count != 0);
  }
}

void VisualSlamNode::CallbackSetOdometryPose(
  const std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::SetOdometryPose_Request> req,
  std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::SetOdometryPose_Response> res)
{
  res->success = false;

  RCLCPP_INFO(this->get_logger(), "elbrus: CallbackSetOdometryPose");

  if (impl_->IsInitialized()) {
    tf2::Transform requested_pose;
    tf2::fromMsg(req->pose, requested_pose);

    ELBRUS_Pose vo_pose;
    const ELBRUS_Status status = ELBRUS_GetOdometryPose(impl_->elbrus_handle, &vo_pose);
    if (status != ELBRUS_SUCCESS) {
      RCLCPP_WARN(this->get_logger(), "ELBRUS_GetOdometryPose() Error %d", status);
      return;
    }
    const tf2::Transform odom_pose_ebrus = FromElbrusPose(vo_pose);
    const tf2::Transform odom_pose_ros = ChangeBasis(
      impl_->base_link_pose_elbrus, odom_pose_ebrus);

    // start_odom_pose * odom_pose = requested_pose * left_pose_base_link.inverse();
    impl_->start_odom_pose = requested_pose * impl_->left_pose_base_link.inverse() *
      odom_pose_ros.inverse();
    res->success = true;
  } else {
    RCLCPP_ERROR(this->get_logger(), "ELBRUS tracker is not initialized");
  }
}

// SaveMap action
rclcpp_action::GoalResponse VisualSlamNode::CallbackSaveMapGoal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const isaac_ros_visual_slam_interfaces::action::SaveMap::Goal> goal)
{
  (void)uuid;
  (void)goal;
  if (impl_->IsInitialized()) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  return rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::CancelResponse VisualSlamNode::CallbackSaveMapCancel(
  const std::shared_ptr<GoalHandleSaveMap> goal_handle)
{
  (void)goal_handle;
  return rclcpp_action::CancelResponse::REJECT;
}

void VisualSlamNode::CallbackSaveMapAccepted(
  const std::shared_ptr<GoalHandleSaveMap> goal_handle)
{
  auto result = std::make_shared<isaac_ros_visual_slam_interfaces::action::SaveMap::Result>();
  result->success = false;

  if (!impl_->IsInitialized()) {
    goal_handle->abort(result);
    return;
  }
  const auto & goal = goal_handle->get_goal();

  // Goal
  std::string map_url = goal->map_url;
  RCLCPP_INFO(this->get_logger(), "elbrus: SaveMap %s", map_url.c_str());

  // Context
  impl_->save_to_slam_db_contexts.push_back(VisualSlamImpl::SaveToSlamDbContext());
  VisualSlamImpl::SaveToSlamDbContext * context = &impl_->save_to_slam_db_contexts.back();
  context->node = this;
  context->goal_handle = goal_handle;

  // Save to Slam DB
  const ELBRUS_Status status = ELBRUS_SaveToSlamDb(
    impl_->elbrus_handle,
    map_url.c_str(),
    &VisualSlamNode::VisualSlamImpl::SaveToSlamDbResponse, context);
  if (status != ELBRUS_SUCCESS) {
    RCLCPP_WARN(this->get_logger(), "ELBRUS_SaveToSlamDb Error %d", status);
    goal_handle->abort(result);
    return;
  }
}

// LoadMapAndLocalize action
rclcpp_action::GoalResponse VisualSlamNode::CallbackLoadMapAndLocalizeGoal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const isaac_ros_visual_slam_interfaces::action::LoadMapAndLocalize::Goal>
  goal)
{
  (void)uuid;
  (void)goal;
  if (impl_->IsInitialized()) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }
  return rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::CancelResponse VisualSlamNode::CallbackLoadMapAndLocalizeCancel(
  const std::shared_ptr<GoalHandleLoadMapAndLocalize> goal_handle)
{
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void VisualSlamNode::CallbackLoadMapAndLocalizeAccepted(
  const std::shared_ptr<GoalHandleLoadMapAndLocalize> goal_handle)
{
  RCLCPP_WARN(this->get_logger(), "CallbackLoadMapAndLocalizeAccepted()");

  auto result =
    std::make_shared<isaac_ros_visual_slam_interfaces::action::LoadMapAndLocalize::Result>();
  result->success = false;

  if (!impl_->IsInitialized()) {
    goal_handle->abort(result);
    return;
  }
  const auto & goal = goal_handle->get_goal();

  const auto & pt = goal->localize_near_point;
  // Goal
  std::string load_map_and_localize_url = goal->map_url;
  tf2::Vector3 load_map_and_localize_point = tf2::Vector3(pt.x, pt.y, pt.z);

  RCLCPP_INFO(
    this->get_logger(), "elbrus: LoadMapAndLocalize %s [%f, %f, %f]",
    goal->map_url.c_str(), pt.x, pt.y, pt.z);

  // Init guess_pose
  tf2::Transform guess_pose;
  guess_pose.setIdentity();
  guess_pose.setOrigin(load_map_and_localize_point);

  // Convert to elbrus
  guess_pose = ChangeBasis(impl_->elbrus_pose_base_link, guess_pose);
  ELBRUS_Pose guess_pose_elbrus = ToElbrusPose(guess_pose);

  // Context
  impl_->localize_in_exist_db_contexts.push_back(VisualSlamImpl::LocalizeInExistDbContext());
  VisualSlamImpl::LocalizeInExistDbContext * context =
    &impl_->localize_in_exist_db_contexts.back();
  context->node = this;
  context->goal_handle = goal_handle;

  // Localize In Exist Db
  const ELBRUS_Status status = ELBRUS_LocalizeInExistDb(
    impl_->elbrus_handle,
    load_map_and_localize_url.c_str(),
    &guess_pose_elbrus,
    0,                    // reserved for future implementation
    nullptr,
    &VisualSlamNode::VisualSlamImpl::LocalizeInExistDbResponse, context);
  if (status != ELBRUS_SUCCESS) {
    RCLCPP_WARN(this->get_logger(), "ELBRUS_LocalizeInExistDb Error %d", status);
    goal_handle->abort(result);
    return;
  }
}


void VisualSlamNode::TrackCameraPose(
  const sensor_msgs::msg::Image::ConstSharedPtr & msg_left_img,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg_left_ci,
  const sensor_msgs::msg::Image::ConstSharedPtr & msg_right_img,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg_right_ci)
{
  Stopwatch stopwatch;
  StopwatchScope ssw(stopwatch);
  if (!valid_elbrus_api_) {
    exit(EXIT_FAILURE);
  }

  // Initialize on first call
  if (!(impl_->IsInitialized())) {
    impl_->Init(*this, msg_left_ci, msg_right_ci);
  }
  // Convert frame to mono image
  const cv::Mat mono_left_img = cv_bridge::toCvShare(msg_left_img, "mono8")->image;
  const cv::Mat mono_right_img = cv_bridge::toCvShare(msg_right_img, "mono8")->image;

  // Track a new pair of frames
  Stopwatch stopwatch_track;
  if (impl_->IsInitialized()) {
    std::array<ELBRUS_Image, kNumCameras> elbrus_images = {};
    rclcpp::Time timestamp(msg_left_img->header.stamp.sec, msg_left_img->header.stamp.nanosec);

    const int64_t acqtime_ns = static_cast<int64_t>(timestamp.nanoseconds());

    elbrus_images[LEFT_CAMERA_IDX] =
      impl_->ToElbrusImage(LEFT_CAMERA_IDX, mono_left_img, acqtime_ns);
    elbrus_images[RIGHT_CAMERA_IDX] =
      impl_->ToElbrusImage(RIGHT_CAMERA_IDX, mono_right_img, acqtime_ns);

    StopwatchScope ssw_track(stopwatch_track);
    ELBRUS_PoseEstimate pose_estimate;
    const ELBRUS_Status status =
      ELBRUS_Track(impl_->elbrus_handle, elbrus_images.data(), nullptr, &pose_estimate);
    ssw_track.Stop();

    if (status == ELBRUS_TRACKING_LOST) {
      impl_->pose_cache.Reset();
      RCLCPP_WARN(this->get_logger(), "Tracker is lost");
      return;
    }
    if (status != ELBRUS_SUCCESS) {
      RCLCPP_WARN(this->get_logger(), "Unknown Tracker Error %d", status);
      return;
    }

    // Pure VO.
    tf2::Transform start_elbrus_pose_smooth_ebrus = FromElbrusPose(pose_estimate.pose);

    // Rectified pose in Elbrus coordinate system. It uses loop closure to get the robust tracking.
    // Note: It can result in sudden jumps of the final pose.
    // Publish Smooth pose if enable_rectified_pose_ = false
    tf2::Transform start_elbrus_pose_rectified_ebrus = start_elbrus_pose_smooth_ebrus;

    if (this->enable_localization_n_mapping_) {
      ELBRUS_PoseSlam pose_slam;
      const ELBRUS_Status pose_slam_status = ELBRUS_GetSlamPose(
        impl_->elbrus_handle, &pose_slam);
      if (pose_slam_status != ELBRUS_SUCCESS) {
        RCLCPP_WARN(this->get_logger(), "Get Rectified Pose Error %d", pose_slam_status);
        return;
      }
      start_elbrus_pose_rectified_ebrus = FromElbrusPose(pose_slam.pose);
    }

    // Change of basis vectors for smooth pose
    const tf2::Transform odom_pose_smooth_left{ChangeBasis(
        impl_->base_link_pose_elbrus,
        start_elbrus_pose_smooth_ebrus)};
    // Change of basis vectors for rectified pose
    tf2::Transform odom_pose_rectified_left{ChangeBasis(
        impl_->base_link_pose_elbrus, start_elbrus_pose_rectified_ebrus)};

    // Use start odom pose
    odom_pose_rectified_left = ChangeBasis(
      impl_->start_odom_pose, odom_pose_rectified_left);

    // Frames hierarchy:
    // map - odom - base_link - ... - camera
    const tf2::Transform map_pose_odom = odom_pose_rectified_left *
      odom_pose_smooth_left.inverse();

    const tf2::Transform odom_pose_base_frame = ChangeBasis(
      impl_->start_odom_pose, odom_pose_smooth_left);

    rclcpp::Time timestamp_output = msg_left_img->header.stamp;
    if (override_publishing_stamp_) {
      timestamp_output = impl_->node_clock->now();
    }
    if (publish_map_to_odom_tf_) {
      if (!invert_map_to_odom_tf_) {
        impl_->PublishFrameTransform(timestamp_output, map_pose_odom, map_frame_, odom_frame_);
      } else {
        impl_->PublishFrameTransform(
          timestamp_output,
          map_pose_odom.inverse(), odom_frame_, map_frame_);
      }
    }
    if (publish_odom_to_base_tf_) {
      if (!invert_odom_to_base_tf_) {
        impl_->PublishFrameTransform(
          timestamp_output, odom_pose_base_frame, odom_frame_,
          base_frame_);
      } else {
        impl_->PublishFrameTransform(
          timestamp_output, odom_pose_base_frame.inverse(),
          base_frame_, odom_frame_);
      }
    }
    impl_->pose_cache.Add(timestamp.nanoseconds(), odom_pose_base_frame);

    geometry_msgs::msg::Twist velocity;
    // Extract timestamp from left image and create headers for
    // slam and vo poses
    std_msgs::msg::Header header_map = msg_left_img->header;
    header_map.frame_id = map_frame_;
    std_msgs::msg::Header header_odom = msg_left_img->header;
    header_odom.frame_id = odom_frame_;

    impl_->pose_cache.GetVelocity(
      velocity.linear.x, velocity.linear.y, velocity.linear.z,
      velocity.angular.x, velocity.angular.y, velocity.angular.z);
    impl_->velocity_cache.Add(
      velocity.linear.x, velocity.linear.y, velocity.linear.z,
      velocity.angular.x, velocity.angular.y, velocity.angular.z);

    {
      // Pose Tracking

      geometry_msgs::msg::Pose vo_pose;
      tf2::toMsg(odom_pose_base_frame, vo_pose);

      geometry_msgs::msg::Pose slam_pose;
      tf2::toMsg(odom_pose_rectified_left, slam_pose);

      if (HasSubscribers(*this, tracking_vo_pose_pub_)) {
        // Tracking_vo_pose_pub_
        auto pose_only = std::make_unique<geometry_msgs::msg::PoseStamped>();
        pose_only->header = header_odom;
        pose_only->pose = vo_pose;
        tracking_vo_pose_pub_->publish(std::move(pose_only));
      }
      if (HasSubscribers(*this, tracking_vo_pose_covariance_pub_)) {
        // Tracking_vo_covariance_pub_
        auto pose_n_cov = std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();
        pose_n_cov->header = header_odom;
        pose_n_cov->pose.pose = vo_pose;
        for (size_t i = 0; i < 6 * 6; i++) {
          pose_n_cov->pose.covariance[i] = static_cast<double>(pose_estimate.covariance[i]);
        }
        tracking_vo_pose_covariance_pub_->publish(std::move(pose_n_cov));
      }
      if (HasSubscribers(*this, tracking_odometry_pub_)) {
        nav_msgs::msg::Odometry odom;
        odom.header = header_odom;
        odom.child_frame_id = base_frame_;

        odom.pose.pose = vo_pose;
        bool res = impl_->pose_cache.GetCovariance(odom.pose.covariance);
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


        res = impl_->velocity_cache.GetCovariance(odom.twist.covariance);
        if (!res) {
          // set default Identity matrix
          for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
              odom.twist.covariance[i * 6 + j] = i == j ? 1 : 0;
            }
          }
        }
        tracking_odometry_pub_->publish(odom);
      }
      if (HasSubscribers(*this, vis_vo_velocity_pub_)) {
        impl_->PublishOdometryVelocity(
          timestamp_output,
          base_frame_,
          vis_vo_velocity_pub_);
      }
      if (HasSubscribers(*this, tracking_vo_path_pub_)) {
        // Tracking_vo_path_pub_
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = header_odom;
        pose_stamped.pose = vo_pose;
        impl_->vo_path.add(pose_stamped);

        auto path_msg = std::make_unique<nav_msgs::msg::Path>();
        path_msg->header = header_odom;
        path_msg->poses = impl_->vo_path.getData();
        tracking_vo_path_pub_->publish(std::move(path_msg));
      }
      if (HasSubscribers(*this, tracking_slam_path_pub_)) {
        // Tracking_slam_path_pub_
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = header_map;
        pose_stamped.pose = slam_pose;
        impl_->slam_path.add(pose_stamped);

        auto path_msg = std::make_unique<nav_msgs::msg::Path>();
        path_msg->header = header_map;
        path_msg->poses = impl_->slam_path.getData();
        tracking_slam_path_pub_->publish(std::move(path_msg));
      }
    }
    // Draw gravity vector
    if (HasSubscribers(*this, vis_gravity_pub_) && this->enable_imu_) {
      impl_->PublishGravity(
        timestamp_output,
        base_frame_,
        vis_gravity_pub_);
    }
    // Visual_odometry_status_pub_
    ssw.Stop();
    if (HasSubscribers(*this, visual_slam_status_pub_)) {
      const double track_execution_time = stopwatch_track.Seconds();
      impl_->track_execution_times.add(track_execution_time);

      double track_execution_time_max = 0;
      double track_execution_time_mean = 0;
      const auto values = impl_->track_execution_times.getData();
      for (double t : values) {
        track_execution_time_max = std::max(track_execution_time_max, t);
        track_execution_time_mean += t;
      }
      if (values.size()) {
        track_execution_time_mean /= values.size();
      }

      isaac_ros_visual_slam_interfaces::msg::VisualSlamStatus visual_slam_status_msg;
      visual_slam_status_msg.header = header_map;
      visual_slam_status_msg.vo_state = pose_estimate.vo_state;
      visual_slam_status_msg.integrator_state = pose_estimate.integrator_state;
      visual_slam_status_msg.node_callback_execution_time = stopwatch.Seconds();
      visual_slam_status_msg.track_execution_time = track_execution_time;
      visual_slam_status_msg.track_execution_time_max = track_execution_time_max;
      visual_slam_status_msg.track_execution_time_mean = track_execution_time_mean;
      visual_slam_status_pub_->publish(visual_slam_status_msg);
    }
  }
}

}  // namespace visual_slam
}  // namespace isaac_ros

// Register as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(isaac_ros::visual_slam::VisualSlamNode)
