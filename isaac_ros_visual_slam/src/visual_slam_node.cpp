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

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "isaac_ros_nitros/types/type_utility.hpp"
#include "isaac_ros_visual_slam/visual_slam_node.hpp"
#include "isaac_ros_visual_slam/impl/types.hpp"
#include "isaac_ros_visual_slam/impl/visual_slam_impl.hpp"
#include "isaac_ros_visual_slam/impl/cuvslam_ros_convertion.hpp"
#include "isaac_ros_visual_slam/impl/posegraph_vis_helper.hpp"
#include "isaac_ros_visual_slam/impl/has_subscribers.hpp"
#include "isaac_ros_visual_slam/impl/stopwatch.hpp"
#include "isaac_ros_visual_slam/impl/qos.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace visual_slam
{

VisualSlamNode::VisualSlamNode(rclcpp::NodeOptions options)
: Node("visual_slam", options),
  // Node parameters.
  denoise_input_images_(declare_parameter<bool>("denoise_input_images", false)),
  rectified_images_(declare_parameter<bool>("rectified_images", true)),
  enable_imu_fusion_(declare_parameter<bool>("enable_imu_fusion", false)),
  imu_params_{
    declare_parameter<double>("gyro_noise_density", 0.000244),
    declare_parameter<double>("gyro_random_walk", 0.000019393),
    declare_parameter<double>("accel_noise_density", 0.001862),
    declare_parameter<double>("accel_random_walk", 0.003),
    declare_parameter<double>("calibration_frequency", 200.0)},
  img_jitter_threshold_ms_(declare_parameter<double>("img_jitter_threshold_ms", 33.33)),
  imu_jitter_threshold_ms_(declare_parameter<double>("imu_jitter_threshold_ms", 10.0)),
  enable_verbosity_(declare_parameter<bool>("enable_verbosity", false)),
  force_planar_mode_(declare_parameter<bool>("force_planar_mode", false)),
  enable_observations_view_(declare_parameter<bool>("enable_observations_view", false)),
  enable_landmarks_view_(declare_parameter<bool>("enable_landmarks_view", false)),
  enable_debug_mode_(declare_parameter<bool>("enable_debug_mode", false)),
  debug_dump_path_(declare_parameter<std::string>("debug_dump_path", "/tmp/cuvslam")),
  enable_localization_n_mapping_(declare_parameter<bool>("enable_localization_n_mapping", true)),
  enable_slam_visualization_(declare_parameter<bool>("enable_slam_visualization", false)),
  input_base_frame_(declare_parameter<std::string>("input_base_frame", "")),
  input_left_camera_frame_(declare_parameter<std::string>("input_left_camera_frame", "")),
  input_right_camera_frame_(declare_parameter<std::string>("input_right_camera_frame", "")),
  input_imu_frame_(declare_parameter<std::string>("input_imu_frame", "imu")),
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
  left_image_sub_(message_filters::Subscriber<ImageType>(
      this, "stereo_camera/left/image", image_qos_)),
  left_camera_info_sub_(message_filters::Subscriber<CameraInfoType>(
      this, "stereo_camera/left/camera_info")),
  right_image_sub_(message_filters::Subscriber<ImageType>(
      this, "stereo_camera/right/image", image_qos_)),
  right_camera_info_sub_(message_filters::Subscriber<CameraInfoType>(
      this, "stereo_camera/right/camera_info")),
  imu_sub_(create_subscription<ImuType>(
      "visual_slam/imu", rclcpp::QoS(100),
      std::bind(&VisualSlamNode::ReadImuData, this, std::placeholders::_1))),

  // Publishers: Visual SLAM
  visual_slam_status_pub_(
    create_publisher<VisualSlamStatusType>("visual_slam/status", rclcpp::QoS(10))),
  tracking_vo_pose_pub_(
    create_publisher<PoseStampedType>("visual_slam/tracking/vo_pose", rclcpp::QoS(100))),
  tracking_vo_pose_covariance_pub_(
    create_publisher<PoseWithCovarianceStampedType>(
      "visual_slam/tracking/vo_pose_covariance", rclcpp::QoS(100))),
  tracking_odometry_pub_(
    create_publisher<OdometryType>("visual_slam/tracking/odometry", rclcpp::QoS(100))),
  tracking_vo_path_pub_(
    create_publisher<PathType>("visual_slam/tracking/vo_path", rclcpp::QoS(100))),
  tracking_slam_path_pub_(
    create_publisher<PathType>("visual_slam/tracking/slam_path", rclcpp::QoS(100))),

  // Visualizators for odometry
  vis_observations_pub_(
    create_publisher<PointCloud2Type>("visual_slam/vis/observations_cloud", rclcpp::QoS(10))),
  vis_gravity_pub_(
    create_publisher<MarkerType>("visual_slam/vis/gravity", rclcpp::QoS(10))),
  vis_vo_velocity_pub_(
    create_publisher<MarkerArrayType>("visual_slam/vis/velocity", rclcpp::QoS(10))),
  vis_slam_odometry_pub_(
    create_publisher<OdometryType>("visual_slam/vis/slam_odometry", rclcpp::QoS(10))),

  // Visualizators for map and "loop closure"
  vis_landmarks_pub_(
    create_publisher<PointCloud2Type>("visual_slam/vis/landmarks_cloud", rclcpp::QoS(10))),
  vis_loop_closure_pub_(
    create_publisher<PointCloud2Type>("visual_slam/vis/loop_closure_cloud", rclcpp::QoS(10))),
  vis_posegraph_nodes_pub_(
    create_publisher<PoseArrayType>("visual_slam/vis/pose_graph_nodes", rclcpp::QoS(10))),
  vis_posegraph_edges_pub_(
    create_publisher<MarkerType>("visual_slam/vis/pose_graph_edges", rclcpp::QoS(10))),
  vis_posegraph_edges2_pub_(
    create_publisher<MarkerType>("visual_slam/vis/pose_graph_edges2", rclcpp::QoS(10))),

  // Visualizators for localization
  vis_localizer_pub_(
    create_publisher<MarkerArrayType>("visual_slam/vis/localizer", rclcpp::QoS(10))),
  vis_localizer_landmarks_pub_(
    create_publisher<PointCloud2Type>("visual_slam/vis/localizer_map_cloud", rclcpp::QoS(10))),
  vis_localizer_observations_pub_(
    create_publisher<PointCloud2Type>(
      "visual_slam/vis/localizer_observations_cloud", rclcpp::QoS(10))),
  vis_localizer_loop_closure_pub_(
    create_publisher<PointCloud2Type>(
      "visual_slam/vis/localizer_loop_closure_cloud", rclcpp::QoS(10))),

  // Slam Services
  reset_srv_(create_service<SrvReset>(
      "visual_slam/reset", std::bind(&VisualSlamNode::CallbackReset, this,
      std::placeholders::_1, std::placeholders::_2))),
  get_all_poses_srv_(
    create_service<SrvGetAllPoses>(
      "visual_slam/get_all_poses", std::bind(&VisualSlamNode::CallbackGetAllPoses, this,
      std::placeholders::_1, std::placeholders::_2))),
  set_odometry_pose_srv_(create_service<SrvSetOdometryPose>(
      "visual_slam/set_odometry_pose", std::bind(&VisualSlamNode::CallbackSetOdometryPose, this,
      std::placeholders::_1, std::placeholders::_2))),

  // Initialize the impl
  impl_(std::make_unique<VisualSlamImpl>(*this))
{
  // Dump and check cuVSLAM API version
  int32_t cuvslam_major, cuvslam_minor;
  const char * cuvslam_version;
  CUVSLAM_GetVersion(&cuvslam_major, &cuvslam_minor, &cuvslam_version);
  RCLCPP_INFO(get_logger(), "cuVSLAM version: %s", cuvslam_version);

  // VisualSlamNode is desined for this cuvslam sdk version:
  int32_t exp_cuvslam_major = 11, exp_cuvslam_minor = 4;
  if (cuvslam_major != exp_cuvslam_major || cuvslam_minor != exp_cuvslam_minor) {
    RCLCPP_ERROR(
      get_logger(), "VisualSlamNode is designed to work with cuVSLAM SDK v%d.%d",
      exp_cuvslam_major, exp_cuvslam_minor);
    valid_cuvslam_api_ = false;
  } else {
    valid_cuvslam_api_ = true;
  }

  Stopwatch stopwatch_gpu;
  StopwatchScope ssw_gpu(stopwatch_gpu);
  // Initializing GPU
  CUVSLAM_WarmUpGPU();
  RCLCPP_INFO(get_logger(), "Time taken by CUVSLAM_WarmUpGPU(): %f", ssw_gpu.Stop());

  impl_->sync.reset(
    new VisualSlamImpl::Synchronizer(
      VisualSlamImpl::ExactTime(msg_filter_queue_size_),
      left_image_sub_, left_camera_info_sub_, right_image_sub_, right_camera_info_sub_
  ));
  impl_->sync->registerCallback(
    std::bind(
      &VisualSlamNode::ReadImageData, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

  impl_->stream_sequencer.RegisterCallback(
    std::bind(&VisualSlamNode::ComputePose, this, std::placeholders::_1, std::placeholders::_2));

  load_map_and_localize_server_ = rclcpp_action::create_server<ActionLoadMapAndLocalize>(
    this,
    "visual_slam/load_map_and_localize",
    std::bind(
      &VisualSlamNode::CallbackLoadMapAndLocalizeGoal, this,
      std::placeholders::_1, std::placeholders::_2),
    std::bind(
      &VisualSlamNode::CallbackLoadMapAndLocalizeCancel, this, std::placeholders::_1),
    std::bind(
      &VisualSlamNode::CallbackLoadMapAndLocalizeAccepted, this, std::placeholders::_1));

  save_map_server_ = rclcpp_action::create_server<ActionSaveMap>(
    this,
    "visual_slam/save_map",
    std::bind(
      &VisualSlamNode::CallbackSaveMapGoal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&VisualSlamNode::CallbackSaveMapCancel, this, std::placeholders::_1),
    std::bind(&VisualSlamNode::CallbackSaveMapAccepted, this, std::placeholders::_1));
}

VisualSlamNode::~VisualSlamNode()
{
  impl_.reset();
}

void VisualSlamNode::ReadImageData(
  const ImageType::ConstSharedPtr & msg_left_img,
  const CameraInfoType::ConstSharedPtr & msg_left_ci,
  const ImageType::ConstSharedPtr & msg_right_img,
  const CameraInfoType::ConstSharedPtr & msg_right_ci)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "VisualSlamNode::ReadImageData", nvidia::isaac_ros::nitros::CLR_YELLOW);

  if (!valid_cuvslam_api_) {
    nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
    exit(EXIT_FAILURE);
  }

  const rclcpp::Time timestamp(msg_left_img->header.stamp.sec, msg_left_img->header.stamp.nanosec);
  const int64_t current_ts = static_cast<int64_t>(timestamp.nanoseconds());

  // Initialize on first call
  if (!(impl_->IsInitialized())) {
    impl_->Init(msg_left_ci, msg_right_ci);
    impl_->last_img_ts = current_ts;
  }

  impl_->stream_sequencer.CallbackStream2(current_ts, {msg_left_img, msg_right_img});

  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}


void VisualSlamNode::ReadImuData(const ImuType::ConstSharedPtr msg_imu)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "VisualSlamNode::ReadImuData", nvidia::isaac_ros::nitros::CLR_RED);
  if (!valid_cuvslam_api_) {
    exit(EXIT_FAILURE);
  }
  if (!impl_->IsInitialized() || !enable_imu_fusion_) {
    return;
  }

  const rclcpp::Time timestamp(msg_imu->header.stamp.sec, msg_imu->header.stamp.nanosec);
  const int64_t current_ts = static_cast<int64_t>(timestamp.nanoseconds());

  impl_->stream_sequencer.CallbackStream1(current_ts, msg_imu);
  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}

void VisualSlamNode::ComputePose(
  const std::vector<ImuType::ConstSharedPtr> imu_msgs,
  const std::pair<ImageType::ConstSharedPtr, ImageType::ConstSharedPtr> image_pair)
{
  for (auto imu_msg : imu_msgs) {
    impl_->RegisterImu(imu_msg);
  }
  impl_->TrackAndGetPose(image_pair.first, image_pair.second);
}

void VisualSlamNode::CallbackReset(
  const std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::Reset_Request> req,
  std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::Reset_Response> res)
{
  (void)req;

  RCLCPP_INFO(this->get_logger(), "cuvslam: Reset");

  impl_->Exit();
  res->success = true;
}

void VisualSlamNode::CallbackGetAllPoses(
  const std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::GetAllPoses_Request> req,
  std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::GetAllPoses_Response> res)
{
  res->success = false;

  RCLCPP_INFO(this->get_logger(), "cuvslam: GetAllPoses");

  if (impl_->IsInitialized()) {
    // CUVSLAM_GetAllPoses
    std::vector<CUVSLAM_PoseStamped> cuvslam_poses(req->max_count);
    uint32_t count = CUVSLAM_GetAllPoses(
      impl_->cuvslam_handle, req->max_count, cuvslam_poses.data());
    if (count == 0) {
      RCLCPP_WARN(this->get_logger(), "CUVSLAM_GetAllPoses Error");
    }
    res->poses.resize(count);

    for (uint32_t i = 0; i < count; i++) {
      const CUVSLAM_Pose & cuvslam_pose_cuvslam = cuvslam_poses[i].pose;
      tf2::Transform ros_pose_cuvslam = FromcuVSLAMPose(cuvslam_pose_cuvslam);
      tf2::Transform ros_pose_ros = ChangeBasis(impl_->canonical_pose_cuvslam, ros_pose_cuvslam);

      PoseType ros_pose_msg;
      tf2::toMsg(ros_pose_ros, ros_pose_msg);

      res->poses[i].pose = ros_pose_msg;
      res->poses[i].header.stamp = rclcpp::Time(cuvslam_poses[i].timestamp_ns, RCL_ROS_TIME);
    }

    res->success = (count != 0);
  }
}

void VisualSlamNode::CallbackSetOdometryPose(
  const std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::SetOdometryPose_Request> req,
  std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::SetOdometryPose_Response> res)
{
  res->success = false;

  RCLCPP_INFO(this->get_logger(), "cuvslam: CallbackSetOdometryPose");

  if (impl_->IsInitialized()) {
    tf2::Transform req_map_pose_base_link;
    tf2::fromMsg(req->pose, req_map_pose_base_link);

    CUVSLAM_Pose vo_pose;
    const CUVSLAM_Status status = CUVSLAM_GetOdometryPose(impl_->cuvslam_handle, &vo_pose);
    if (status != CUVSLAM_SUCCESS) {
      RCLCPP_WARN(this->get_logger(), "CUVSLAM_GetOdometryPose() Error %d", status);
      return;
    }

    const tf2::Transform ros_vo_pose =
      ChangeBasis(impl_->canonical_pose_cuvslam, FromcuVSLAMPose(vo_pose));

    const tf2::Transform odom_pose_base_link = impl_->initial_odom_pose_left * ros_vo_pose *
      impl_->base_link_pose_left.inverse();

    const tf2::Transform map_pose_odom = req_map_pose_base_link * odom_pose_base_link.inverse();

    impl_->PublishFrameTransform(this->get_clock()->now(), map_pose_odom, "map", odom_frame_, true);
    res->success = true;
  } else {
    RCLCPP_ERROR(this->get_logger(), "CUVSLAM tracker is not initialized");
  }
}

// SaveMap action
rclcpp_action::GoalResponse VisualSlamNode::CallbackSaveMapGoal(
  const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ActionSaveMap::Goal> goal)
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
  auto result = std::make_shared<ActionSaveMap::Result>();
  result->success = false;

  if (!impl_->IsInitialized()) {
    goal_handle->abort(result);
    return;
  }
  const auto & goal = goal_handle->get_goal();

  // Goal
  std::string map_url = goal->map_url;
  RCLCPP_INFO(this->get_logger(), "cuvslam: SaveMap %s", map_url.c_str());

  // Context
  VisualSlamImpl::SaveToSlamDbContext * context = new VisualSlamImpl::SaveToSlamDbContext();
  context->goal_handle = goal_handle;

  // Save to Slam DB
  const CUVSLAM_Status status = CUVSLAM_SaveToSlamDb(
    impl_->cuvslam_handle, map_url.c_str(),
    &VisualSlamNode::VisualSlamImpl::SaveToSlamDbResponse, context);
  if (status != CUVSLAM_SUCCESS) {
    RCLCPP_WARN(this->get_logger(), "CUVSLAM_SaveToSlamDb Error %d", status);
    goal_handle->abort(result);
    return;
  }
}

// LoadMapAndLocalize action
rclcpp_action::GoalResponse VisualSlamNode::CallbackLoadMapAndLocalizeGoal(
  const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ActionLoadMapAndLocalize::Goal> goal)
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
    std::make_shared<ActionLoadMapAndLocalize::Result>();
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
    this->get_logger(), "cuvslam: LoadMapAndLocalize %s [%f, %f, %f]",
    goal->map_url.c_str(), pt.x, pt.y, pt.z);

  // Init guess_pose
  tf2::Transform guess_pose;
  guess_pose.setIdentity();
  guess_pose.setOrigin(load_map_and_localize_point);

  // Convert to cuvslam
  guess_pose = ChangeBasis(impl_->cuvslam_pose_canonical, guess_pose);
  CUVSLAM_Pose guess_pose_cuvslam = TocuVSLAMPose(guess_pose);

  // Context
  VisualSlamImpl::LocalizeInExistDbContext * context =
    new VisualSlamImpl::LocalizeInExistDbContext();
  context->impl = impl_.get();
  context->goal_handle = goal_handle;

  // Localize In Exist Db
  const CUVSLAM_Status status = CUVSLAM_LocalizeInExistDb(
    impl_->cuvslam_handle, load_map_and_localize_url.c_str(), &guess_pose_cuvslam,
    0,                    // reserved for future implementation
    nullptr,
    &VisualSlamNode::VisualSlamImpl::LocalizeInExistDbResponse, context);
  if (status != CUVSLAM_SUCCESS) {
    RCLCPP_WARN(this->get_logger(), "CUVSLAM_LocalizeInExistDb Error %d", status);
    goal_handle->abort(result);
    return;
  }
}

}  // namespace visual_slam
}  // namespace isaac_ros
}  // namespace nvidia

// Register as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nvidia::isaac_ros::visual_slam::VisualSlamNode)
