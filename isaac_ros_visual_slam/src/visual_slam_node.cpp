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
#include <utility>
#include <vector>
#include <thread>

#include "isaac_ros_common/qos.hpp"
#include "isaac_ros_nitros/types/type_utility.hpp"
#include "isaac_ros_visual_slam/visual_slam_node.hpp"
#include "isaac_ros_visual_slam/impl/types.hpp"
#include "isaac_ros_visual_slam/impl/visual_slam_impl.hpp"
#include "isaac_ros_visual_slam/impl/cuvslam_ros_conversion.hpp"
#include "isaac_ros_visual_slam/impl/posegraph_vis_helper.hpp"
#include "isaac_ros_visual_slam/impl/has_subscribers.hpp"
#include "isaac_ros_visual_slam/impl/stopwatch.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


namespace nvidia
{
namespace isaac_ros
{
namespace visual_slam
{

VisualSlamNode::VisualSlamNode(rclcpp::NodeOptions options)
: Node("visual_slam", options),
  // Functional Parameters:
  num_cameras_(declare_parameter<int>("num_cameras", 2)),
  multicam_mode_(declare_parameter<int>("multicam_mode", 1)),
  min_num_images_(declare_parameter<int>("min_num_images", num_cameras_)),
  sync_matching_threshold_ms_(declare_parameter<double>("sync_matching_threshold_ms", 5.0)),
  border_mask_top_(declare_parameter<int>("img_mask_top", 0)),
  border_mask_bottom_(declare_parameter<int>("img_mask_bottom", 0)),
  border_mask_left_(declare_parameter<int>("img_mask_left", 0)),
  border_mask_right_(declare_parameter<int>("img_mask_right", 0)),
  enable_image_denoising_(declare_parameter<bool>("enable_image_denoising", false)),
  rectified_images_(declare_parameter<bool>("rectified_images", true)),
  enable_ground_constraint_in_odometry_(
    declare_parameter<bool>("enable_ground_constraint_in_odometry", false)),
  enable_ground_constraint_in_slam_(
    declare_parameter<bool>("enable_ground_constraint_in_slam", false)),
  enable_localization_n_mapping_(declare_parameter<bool>("enable_localization_n_mapping", true)),
  enable_imu_fusion_(declare_parameter<bool>("enable_imu_fusion", false)),
  enable_debug_imu_mode_(declare_parameter<bool>("debug_imu_mode", false)),
  imu_params_{
    declare_parameter<double>("gyro_noise_density", 0.000244),
    declare_parameter<double>("gyro_random_walk", 0.000019393),
    declare_parameter<double>("accel_noise_density", 0.001862),
    declare_parameter<double>("accel_random_walk", 0.003),
    declare_parameter<double>("calibration_frequency", 200.0)},
  image_jitter_threshold_ms_(declare_parameter<double>("image_jitter_threshold_ms", 34.0)),
  imu_jitter_threshold_ms_(declare_parameter<double>("imu_jitter_threshold_ms", 10.0)),
  save_map_folder_path_(declare_parameter<std::string>("save_map_folder_path", "")),
  load_map_folder_path_(declare_parameter<std::string>("load_map_folder_path", "")),
  localize_on_startup_(declare_parameter<bool>("localize_on_startup", false)),
  localizer_horizontal_radius_(declare_parameter<float>("localizer_horizontal_radius", 1.5)),
  localizer_vertical_radius_(declare_parameter<float>("localizer_vertical_radius", 0.5)),
  localizer_horizontal_step_(declare_parameter<float>("localizer_horizontal_step", 0.5)),
  localizer_vertical_step_(declare_parameter<float>("localizer_vertical_step", 0.25)),
  localizer_angular_step_(declare_parameter<float>("localizer_angular_step", 0.1745)),
  slam_max_map_size_(declare_parameter<int>("slam_max_map_size", 300)),
  slam_throttling_time_ms_(declare_parameter<int>("slam_throttling_time_ms", 500)),
  // Frame parameters:
  map_frame_(declare_parameter<std::string>("map_frame", "map")),
  odom_frame_(declare_parameter<std::string>("odom_frame", "odom")),
  base_frame_(declare_parameter<std::string>("base_frame", "base_link")),
  camera_optical_frames_(declare_parameter<std::vector<std::string>>(
      "camera_optical_frames",
      std::vector<std::string>{})),
  imu_frame_(declare_parameter<std::string>("imu_frame", "")),
  // Message Parameters:
  image_buffer_size_(declare_parameter<int>("image_buffer_size", 10)),
  imu_buffer_size_(declare_parameter<int>("imu_buffer_size", 50)),
  image_qos_(::isaac_ros::common::AddQosParameter(*this, "SENSOR_DATA", "image_qos")),
  imu_qos_(::isaac_ros::common::AddQosParameter(*this, "SENSOR_DATA", "imu_qos")),
  // Output Parameters:
  override_publishing_stamp_(declare_parameter<bool>("override_publishing_stamp", false)),
  publish_map_to_odom_tf_(declare_parameter<bool>("publish_map_to_odom_tf", true)),
  publish_odom_to_base_tf_(declare_parameter<bool>("publish_odom_to_base_tf", true)),
  invert_map_to_odom_tf_(declare_parameter<bool>("invert_map_to_odom_tf", false)),
  invert_odom_to_base_tf_(declare_parameter<bool>("invert_odom_to_base_tf", false)),
  // Debug/Visualization Parameters:
  enable_slam_visualization_(declare_parameter<bool>("enable_slam_visualization", false)),
  enable_observations_view_(declare_parameter<bool>("enable_observations_view", false)),
  enable_landmarks_view_(declare_parameter<bool>("enable_landmarks_view", false)),
  path_max_size_(declare_parameter<int>("path_max_size", 1024)),
  verbosity_(declare_parameter<int>("verbosity", 0)),
  enable_debug_mode_(declare_parameter<bool>("enable_debug_mode", false)),
  debug_dump_path_(declare_parameter<std::string>("debug_dump_path", "/tmp/cuvslam")),
  enable_request_hint_(declare_parameter<bool>("enable_request_hint", true)),
  localize_in_map_callback_group_(this->create_callback_group(rclcpp::CallbackGroupType::
    MutuallyExclusive)),
  // Subscribers:
  image_subs_(std::make_unique<std::vector<std::shared_ptr<NitrosImageViewSubscriber>>>(
      [this]() {
        std::vector<std::shared_ptr<NitrosImageViewSubscriber>> subs;
        subs.reserve(num_cameras_);
        for (uint i = 0; i < num_cameras_; ++i) {
          subs.push_back(
            std::make_shared<NitrosImageViewSubscriber>(
              this,
              "visual_slam/image_" + std::to_string(i),
              nvidia::isaac_ros::nitros::nitros_image_rgb8_t::supported_type_name,
              std::bind(&VisualSlamNode::CallbackImage, this, i, std::placeholders::_1),
              nvidia::isaac_ros::nitros::NitrosDiagnosticsConfig(), image_qos_));
        }
        return subs;
      }())),
  camera_info_subs_(
    [this]() {
      std::vector<rclcpp::Subscription<CameraInfoType>::SharedPtr> subs;
      subs.reserve(num_cameras_);
      for (uint i = 0; i < num_cameras_; ++i) {
        subs.push_back(
          create_subscription<CameraInfoType>(
            "visual_slam/camera_info_" + std::to_string(i), image_qos_,
            [this, i](const CameraInfoType::ConstSharedPtr & msg) {
              return CallbackCameraInfo(i, msg);
            }));
      }
      return subs;
    }()),
  imu_sub_(
    [this]() -> rclcpp::Subscription<ImuType>::SharedPtr {
      if (!enable_imu_fusion_) {return nullptr;}
      return create_subscription<ImuType>(
        "visual_slam/imu", imu_qos_,
        std::bind(&VisualSlamNode::CallbackImu, this, std::placeholders::_1));
    }()),
  initial_pose_sub_(
    [this]() -> rclcpp::Subscription<PoseWithCovarianceStampedType>::SharedPtr {
      rclcpp::SubscriptionOptions options;
      options.callback_group = localize_in_map_callback_group_;
      return create_subscription<PoseWithCovarianceStampedType>(
        "visual_slam/initial_pose", rclcpp::QoS(rclcpp::ServicesQoS()),
        std::bind(&VisualSlamNode::CallbackInitialPose, this, std::placeholders::_1),
        options
      );
    }()),
  // Publishers: Visual SLAM
  visual_slam_status_pub_(
    create_publisher<VisualSlamStatusType>(
      "visual_slam/status", ::isaac_ros::common::ParseQosString("DEFAULT"))),
  tracking_vo_pose_pub_(
    create_publisher<PoseStampedType>(
      "visual_slam/tracking/vo_pose", ::isaac_ros::common::ParseQosString("DEFAULT"))),
  tracking_vo_pose_covariance_pub_(
    create_publisher<PoseWithCovarianceStampedType>(
      "visual_slam/tracking/vo_pose_covariance", ::isaac_ros::common::ParseQosString("DEFAULT"))),
  tracking_odometry_pub_(
    create_publisher<OdometryType>(
      "visual_slam/tracking/odometry", ::isaac_ros::common::ParseQosString("DEFAULT"))),
  tracking_vo_path_pub_(
    create_publisher<PathType>(
      "visual_slam/tracking/vo_path", ::isaac_ros::common::ParseQosString("DEFAULT"))),
  tracking_slam_path_pub_(
    create_publisher<PathType>(
      "visual_slam/tracking/slam_path", ::isaac_ros::common::ParseQosString("DEFAULT"))),
  diagnostics_pub_(
    create_publisher<DiagnosticArrayType>(
      "/diagnostics", ::isaac_ros::common::ParseQosString("DEFAULT"))),

  // Publishers: sends a message to request another pose hint.
  trigger_hint_pub_(
    [this]() -> rclcpp::Publisher<PoseWithCovarianceStampedType>::SharedPtr {
      if (!enable_request_hint_) {return nullptr;}
      return create_publisher<PoseWithCovarianceStampedType>(
        "visual_slam/trigger_hint", ::isaac_ros::common::ParseQosString("DEFAULT"));
    }()),

  // Visualizators for odometry
  vis_observations_pub_(
    create_publisher<PointCloud2Type>(
      "visual_slam/vis/observations_cloud", ::isaac_ros::common::ParseQosString("DEFAULT"))),
  vis_gravity_pub_(
    create_publisher<MarkerType>(
      "visual_slam/vis/gravity", ::isaac_ros::common::ParseQosString("DEFAULT"))),
  vis_vo_velocity_pub_(
    create_publisher<MarkerArrayType>(
      "visual_slam/vis/velocity", ::isaac_ros::common::ParseQosString("DEFAULT"))),
  vis_slam_odometry_pub_(
    create_publisher<OdometryType>(
      "visual_slam/vis/slam_odometry", ::isaac_ros::common::ParseQosString("DEFAULT"))),

  // Visualizators for map and "loop closure"
  vis_landmarks_pub_(
    create_publisher<PointCloud2Type>(
      "visual_slam/vis/landmarks_cloud", ::isaac_ros::common::ParseQosString("DEFAULT"))),
  vis_loop_closure_pub_(
    create_publisher<PointCloud2Type>(
      "visual_slam/vis/loop_closure_cloud", ::isaac_ros::common::ParseQosString("DEFAULT"))),
  vis_posegraph_nodes_pub_(
    create_publisher<PoseArrayType>(
      "visual_slam/vis/pose_graph_nodes", ::isaac_ros::common::ParseQosString("DEFAULT"))),
  vis_posegraph_edges_pub_(
    create_publisher<MarkerType>(
      "visual_slam/vis/pose_graph_edges", ::isaac_ros::common::ParseQosString("DEFAULT"))),
  vis_posegraph_edges2_pub_(
    create_publisher<MarkerType>(
      "visual_slam/vis/pose_graph_edges2", ::isaac_ros::common::ParseQosString("DEFAULT"))),

  // Visualizators for localization
  vis_localizer_pub_(
    create_publisher<MarkerArrayType>(
      "visual_slam/vis/localizer", ::isaac_ros::common::ParseQosString("DEFAULT"))),
  vis_localizer_landmarks_pub_(
    create_publisher<PointCloud2Type>(
      "visual_slam/vis/localizer_map_cloud", ::isaac_ros::common::ParseQosString("DEFAULT"))),
  vis_localizer_observations_pub_(
    create_publisher<PointCloud2Type>(
      "visual_slam/vis/localizer_observations_cloud",
      ::isaac_ros::common::ParseQosString("DEFAULT"))),
  vis_localizer_loop_closure_pub_(
    create_publisher<PointCloud2Type>(
      "visual_slam/vis/localizer_loop_closure_cloud",
      ::isaac_ros::common::ParseQosString("DEFAULT"))),

  // Slam Services
  reset_srv_(
    create_service<SrvReset>(
      "visual_slam/reset", std::bind(
        &VisualSlamNode::CallbackReset, this,
        std::placeholders::_1, std::placeholders::_2))),
  get_all_poses_srv_(
    create_service<SrvGetAllPoses>(
      "visual_slam/get_all_poses", std::bind(
        &VisualSlamNode::CallbackGetAllPoses, this,
        std::placeholders::_1, std::placeholders::_2))),
  set_slam_pose_srv_(
    create_service<SrvSetSlamPose>(
      "visual_slam/set_slam_pose", std::bind(
        &VisualSlamNode::CallbackSetSlamPose, this,
        std::placeholders::_1, std::placeholders::_2))),
  save_map_srv_(
    create_service<SrvFilePath>(
      "visual_slam/save_map", std::bind(
        &VisualSlamNode::CallbackSaveMap, this,
        std::placeholders::_1, std::placeholders::_2))),
  load_map_srv_(
    create_service<SrvFilePath>(
      "visual_slam/load_map", std::bind(
        &VisualSlamNode::CallbackLoadMap, this,
        std::placeholders::_1, std::placeholders::_2))),
  localize_in_map_srv_(
    create_service<SrvLocalizeInMap>(
      "visual_slam/localize_in_map", std::bind(
        &VisualSlamNode::CallbackLocalizeInMap, this,
        std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default,
      localize_in_map_callback_group_)),
  // Initialize the impl
  impl_(std::make_unique<VisualSlamImpl>(*this))
{
  // Dump and check cuVSLAM API version
  int32_t cuvslam_major, cuvslam_minor;
  const char * cuvslam_version;
  CUVSLAM_GetVersion(&cuvslam_major, &cuvslam_minor, &cuvslam_version);
  RCLCPP_INFO(get_logger(), "cuVSLAM version: %s", cuvslam_version);

  // VisualSlamNode is desined for this cuvslam sdk version:
  int32_t exp_cuvslam_major = 12, exp_cuvslam_minor = 6;
  if (cuvslam_major != exp_cuvslam_major || cuvslam_minor != exp_cuvslam_minor) {
    RCLCPP_FATAL(
      get_logger(), "VisualSlamNode is designed to work with cuVSLAM SDK v%d.%d",
      exp_cuvslam_major, exp_cuvslam_minor);
    exit(EXIT_FAILURE);
  }

  Stopwatch stopwatch_gpu;
  StopwatchScope ssw_gpu(stopwatch_gpu);
  // Initializing GPU
  CUVSLAM_WarmUpGPU();
  RCLCPP_INFO(get_logger(), "Time taken by CUVSLAM_WarmUpGPU(): %f", ssw_gpu.Stop());
}

VisualSlamNode::~VisualSlamNode()
{
  if (!save_map_folder_path_.empty()) {
    if (impl_->IsInitialized()) {
      impl_->SaveMap(save_map_folder_path_);
    }
  }
  impl_.reset();

  // Having the destructor destroying NITROS types may fail if the process-wide GXF/NITROS context
  // has been released elsewhere. This is out of control of this node. To avoid crashes, we refrain
  // from deleting them and instead relay on the OS for memory cleanup. Note that this will lead to
  // memory leaks if several classes are instantiated in the same process.
  image_subs_.release();
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

  if (!impl_->IsInitialized()) {
    RCLCPP_ERROR(this->get_logger(), "CUVSLAM tracker is not initialized");
    res->success = false;
    return;
  }

  // CUVSLAM_GetAllSlamPoses
  std::vector<CUVSLAM_PoseStamped> cuvslam_poses(req->max_count);
  const uint32_t count = CUVSLAM_GetAllSlamPoses(
    impl_->cuvslam_handle, req->max_count, cuvslam_poses.data());
  if (count == 0) {
    RCLCPP_WARN(this->get_logger(), "CUVSLAM_GetAllSlamPoses Error");
  }
  res->poses.resize(count);

  for (uint32_t i = 0; i < count; i++) {
    tf2::Transform cv_map_pose_cv_base = FromcuVSLAMPose(cuvslam_poses[i].pose);
    tf2::Transform map_pose_base =
      ChangeBasis(canonical_pose_cuvslam, cv_map_pose_cv_base);
    PoseType ros_pose_msg;
    tf2::toMsg(map_pose_base, ros_pose_msg);

    res->poses[i].pose = ros_pose_msg;
    res->poses[i].header.stamp = rclcpp::Time(cuvslam_poses[i].timestamp_ns, RCL_ROS_TIME);
  }

  res->success = (count != 0);
}

void VisualSlamNode::CallbackSetSlamPose(
  const std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::SetSlamPose_Request> req,
  std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::SetSlamPose_Response> res)
{
  res->success = false;

  RCLCPP_INFO(this->get_logger(), "cuvslam: CallbackSetSlamPose");

  if (impl_->IsInitialized()) {
    tf2::Transform req_map_pose_base_link;
    tf2::fromMsg(req->pose, req_map_pose_base_link);

    // Convert to cuvslam
    req_map_pose_base_link = ChangeBasis(cuvslam_pose_canonical, req_map_pose_base_link);
    CUVSLAM_Pose req_map_pose_cuvslam = TocuVSLAMPose(req_map_pose_base_link);

    const CUVSLAM_Status status = CUVSLAM_SetSlamPose(impl_->cuvslam_handle, &req_map_pose_cuvslam);
    if (status != CUVSLAM_SUCCESS) {
      RCLCPP_WARN(this->get_logger(), "CUVSLAM_SetSlamPose() Error %d", status);
      return;
    }
    res->success = true;
  } else {
    RCLCPP_ERROR(this->get_logger(), "CUVSLAM tracker is not initialized");
  }
}

void VisualSlamNode::CallbackSaveMap(
  const std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::FilePath::Request> req,
  std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::FilePath::Response> res)
{
  res->success = false;

  if (!impl_->IsInitialized()) {
    RCLCPP_ERROR(this->get_logger(), "CUVSLAM tracker is not initialized");
    return;
  }

  const CUVSLAM_Status status = impl_->SaveMap(req->file_path);
  if (status != CUVSLAM_SUCCESS) {
    return;
  }

  res->success = true;
}

void VisualSlamNode::CallbackLoadMap(
  const std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::FilePath::Request> req,
  std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::FilePath::Response> res)
{
  res->success = false;
  load_map_folder_path_ = req->file_path;
  res->success = true;
}

void VisualSlamNode::CallbackLocalizeInMap(
  const std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap::Request> req,
  std::shared_ptr<isaac_ros_visual_slam_interfaces::srv::LocalizeInMap::Response> res)
{
  res->success = false;

  if (!impl_->IsInitialized()) {
    RCLCPP_ERROR(this->get_logger(), "CUVSLAM tracker is not initialized");
    return;
  }

  if (!req->map_folder_path.empty()) {
    load_map_folder_path_ = req->map_folder_path;
  }

  const auto maybe_pose = impl_->LocalizeInMap(
    load_map_folder_path_,
    req->pose_hint,
    map_frame_);
  if (!maybe_pose) {return;}

  res->success = true;
}

void VisualSlamNode::CallbackImu(const ImuType::ConstSharedPtr & msg) {impl_->CallbackImu(msg);}

void VisualSlamNode::CallbackImage(int index, const ImageType & msg)
{
  impl_->CallbackImage(index, msg);
}

void VisualSlamNode::CallbackCameraInfo(int index, const CameraInfoType::ConstSharedPtr & msg)
{
  impl_->CallbackCameraInfo(index, msg);
}

void VisualSlamNode::CallbackInitialPose(const PoseWithCovarianceStampedType::ConstSharedPtr & msg)
{
  const rclcpp::Time stamp(msg->header.stamp);
  if (impl_->LocalizeInMap(load_map_folder_path_, msg->pose.pose, msg->header.frame_id)) {
    return;
  }

  if (enable_request_hint_) {
    trigger_hint_pub_->publish(geometry_msgs::msg::PoseWithCovarianceStamped());
    RCLCPP_INFO(this->get_logger(), "Localization failed. Requesting another localization hint.");
  } else {
    RCLCPP_INFO(this->get_logger(), "Localization failed. Shutting down.");
    rclcpp::shutdown();
  }
}

}  // namespace visual_slam
}  // namespace isaac_ros
}  // namespace nvidia

// Register as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(nvidia::isaac_ros::visual_slam::VisualSlamNode)
