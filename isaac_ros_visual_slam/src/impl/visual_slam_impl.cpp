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

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include "eigen3/Eigen/Dense"
#include "isaac_ros_nitros/types/type_utility.hpp"
#include "isaac_ros_visual_slam/impl/cuvslam_ros_convertion.hpp"
#include "isaac_ros_visual_slam/impl/has_subscribers.hpp"
#include "isaac_ros_visual_slam/impl/stopwatch.hpp"
#include "isaac_ros_visual_slam/impl/types.hpp"
#include "isaac_ros_visual_slam/impl/visual_slam_impl.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace
{
constexpr char kFisheye4[] = "fisheye4";
constexpr char kBrown5k[] = "brown5k";
constexpr char kPinhole[] = "pinhole";
constexpr char kPolynomial[] = "polynomial";

// ROS' DISTORTION MODELS:
// 1.PLUMB_BOB
// 2.RATIONAL_POLYNOMIAL
// CUVSLAM' DISTORTION MODELS:
// 1.brown5k
// 2.fisheye4
// 3.polynomial
// ROS' PLUMB_BOB = cuVSLAM' brown5k
enum DistortionModel
{
  BROWN,
  FISHEYE,
  PINHOLE,
  RATIONAL_POLYNOMIAL
};

// Map the encoding desired string to the DistortionModel
const std::unordered_map<std::string, DistortionModel> g_str_to_distortion_model({
    {"plumb_bob", DistortionModel::BROWN},
    {"fisheye", DistortionModel::FISHEYE},
    {"pinhole", DistortionModel::PINHOLE},
    {"rational_polynomial", DistortionModel::RATIONAL_POLYNOMIAL}});

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

constexpr double kNanoSecInMilliSec = 1e6;
constexpr uint8_t kImageBufferSize = 50;
constexpr uint8_t kImuBufferSize = 50;
}  // namespace


namespace nvidia
{
namespace isaac_ros
{
namespace visual_slam
{

VisualSlamNode::VisualSlamImpl::VisualSlamImpl(VisualSlamNode & vslam_node)
: node(vslam_node),
  // Set cuvslam_pose_canonical for converting ROS coordinate to cuVSLAM coordinate
  // ROS    ->  cuVSLAM
  //  x     ->    -z
  //  y     ->    -x
  //  z     ->     y
  cuvslam_pose_canonical(tf2::Matrix3x3(
      0, -1, 0,
      0, 0, 1,
      -1, 0, 0
    )),
  canonical_pose_cuvslam(cuvslam_pose_canonical.inverse()),
  // Set cuvslam_pose_optical for converting Optical coordinate to cuVSLAM coordinate
  // Optical   ->  cuVSLAM
  //    x      ->     x
  //    y      ->    -y
  //    z      ->    -z
  cuvslam_pose_optical(tf2::Matrix3x3(
      1, 0, 0,
      0, -1, 0,
      0, 0, -1
    )),
  optical_pose_cuvslam(cuvslam_pose_optical.inverse()),
  tf_buffer(std::make_unique<tf2_ros::Buffer>(node.get_clock())),
  tf_listener(std::make_shared<tf2_ros::TransformListener>(*tf_buffer)),
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
  delta_ts(100),
  last_img_ts(0),
  stream_sequencer(kImuBufferSize, node.imu_jitter_threshold_ms_ * kNanoSecInMilliSec,
    kImageBufferSize, node.img_jitter_threshold_ms_ * kNanoSecInMilliSec)
{}

VisualSlamNode::VisualSlamImpl::~VisualSlamImpl()
{
  Exit();
}

// Flag to check the status of initialization.
bool VisualSlamNode::VisualSlamImpl::IsInitialized()
{
  return cuvslam_handle != nullptr;
}

void VisualSlamNode::VisualSlamImpl::Init(
  const CameraInfoType::ConstSharedPtr & msg_left_ci,
  const CameraInfoType::ConstSharedPtr & msg_right_ci)
{
  if (cuvslam_handle == nullptr) {
    const rclcpp::Time & stamp = msg_left_ci->header.stamp;

    std::string in_base_link = node.input_base_frame_;
    if (in_base_link.empty()) {
      in_base_link = node.base_frame_;
    }

    if (!node.input_left_camera_frame_.empty() && !in_base_link.empty()) {
      base_link_pose_left = GetFrameTransform(stamp, in_base_link, node.input_left_camera_frame_);
    }

    // Hierarchy of frames are map -> odom -> base_link -> camera -> left
    // Initially  map -> odom and odom -> base_link is identity
    initial_odom_pose_left = base_link_pose_left;
    initial_map_pose_left = base_link_pose_left;

    tf2::Transform left_pose_right;
    if (!node.input_right_camera_frame_.empty()) {
      RCLCPP_INFO(node.get_logger(), "left_pose_right reading from TF Tree");
      left_pose_right =
        ChangeBasis(
        cuvslam_pose_canonical,
        GetFrameTransform(stamp, node.input_left_camera_frame_, node.input_right_camera_frame_));
    } else {
      RCLCPP_INFO(node.get_logger(), "left_pose_right reading from CameraInfo");

      // Get camera intrinsics to set camera coordinate frames
      const double * right_p = msg_right_ci->p.data();
      // Set right camera translation relative to left camera from camera info
      // Tx = -fx' * B, where B is the baseline between the cameras.
      // B = -Tx/fx'
      const double fx = right_p[0];
      if (std::abs(fx) < 1e-6) {
        RCLCPP_ERROR(node.get_logger(), "FocalX is too small: %f", fx);
        return;
      }
      const double baseline = -right_p[3] / fx;
      if (std::abs(baseline) < 1e-6) {
        RCLCPP_ERROR(node.get_logger(), "Baseline is too small: %f", baseline);
        return;
      }
      RCLCPP_INFO(node.get_logger(), "Baseline is : %f", baseline);

      const Eigen::Vector3d right_camera_translation(baseline, right_p[7], right_p[11]);
      // Set right camera rotation to identity
      left_pose_right.setIdentity();
      left_pose_right.setOrigin(
        tf2::Vector3(
          right_camera_translation[0],
          right_camera_translation[1],
          right_camera_translation[2]));

      if (!node.rectified_images_) {
        //              T2
        //     A<--------------B
        //     |               |
        //  T1 |               |  T3
        //     V               V
        //     C<--------------D
        //             T4
        // A and B = Left and Right camera before stereo rectification respectively
        // C and D = Left and Right camera after stereo rectification respectively
        // T2 = [R2|t2] (left_unrect_pose_right_unrect)
        // T1 = [R1|t1] where t1 = {0, 0, 0} (left_rect_pose_left_unrect)
        // T3 = [R3|t3] where t3 = {0, 0, 0} (right_rect_pose_right_unrect)
        // T4 = [R4|t4] where R4 is [identity] and
        // t4 = {baseline, 0, 0} [After rectification] (left_rect_pose_right_rect)
        //    T4 = T1*T2*T3.inverse()
        // => T2 = T1.inverse()*T4*T3

        // Get camera rectification matrix
        const double * left_r = msg_left_ci->r.data();
        const double * right_r = msg_right_ci->r.data();
        const tf2::Matrix3x3 left_r_mat(
          left_r[0], left_r[1], left_r[2],
          left_r[3], left_r[4], left_r[5],
          left_r[6], left_r[7], left_r[8]);
        const tf2::Matrix3x3 right_r_mat(
          right_r[0], right_r[1], right_r[2],
          right_r[3], right_r[4], right_r[5],
          right_r[6], right_r[7], right_r[8]);

        tf2::Transform left_rect_pose_right_rect(left_pose_right);
        tf2::Transform left_rect_pose_left_unrect(left_r_mat);
        tf2::Transform right_rect_pose_right_unrect(right_r_mat);
        left_pose_right = ChangeBasis(
          cuvslam_pose_optical, (left_rect_pose_left_unrect.inverse() * left_rect_pose_right_rect *
          right_rect_pose_right_unrect));
      }
    }

    tf2::Transform left_camera_pose_imu = tf2::Transform::getIdentity();
    if (node.enable_imu_fusion_) {
      RCLCPP_INFO(node.get_logger(), "left_camera_pose_imu reading from TF Tree");
      std::string left_cam_frame = node.input_left_camera_frame_;

      // Left camera in the robot's center
      if (left_cam_frame.empty()) {
        left_cam_frame = in_base_link;
      }

      // Convert the left_camera_pose_imu from ROS to cuVSLAM frame
      left_camera_pose_imu = ChangeBasis(
        cuvslam_pose_canonical,
        GetFrameTransform(stamp, left_cam_frame, node.input_imu_frame_));
    }

    // cuVSLAM configuration needs Left pose IMU camera
    const CUVSLAM_Configuration configuration =
      GetConfiguration(TocuVSLAMPose(left_camera_pose_imu));

    CUVSLAM_CameraRig rig;
    rig.cameras = cuvslam_cameras.data();
    rig.num_cameras = kNumCameras;

    SetcuVSLAMCameraPose(
      cuvslam_cameras[LEFT_CAMERA_IDX], TocuVSLAMPose(tf2::Transform::getIdentity()));
    SetcuVSLAMCameraPose(
      cuvslam_cameras[RIGHT_CAMERA_IDX], TocuVSLAMPose(left_pose_right));

    cuvslam_cameras[LEFT_CAMERA_IDX].parameters = left_intrinsics.data();
    cuvslam_cameras[RIGHT_CAMERA_IDX].parameters = right_intrinsics.data();
    ReadCameraIntrinsics(msg_left_ci, cuvslam_cameras[LEFT_CAMERA_IDX], left_intrinsics);
    ReadCameraIntrinsics(msg_right_ci, cuvslam_cameras[RIGHT_CAMERA_IDX], right_intrinsics);

    CUVSLAM_TrackerHandle tracker;
    PrintConfiguration(node.get_logger(), configuration);
    Stopwatch stopwatch_tracker;
    StopwatchScope ssw_tracker(stopwatch_tracker);
    const CUVSLAM_Status status = CUVSLAM_CreateTracker(&tracker, &rig, &configuration);
    RCLCPP_INFO(
      node.get_logger(), "Time taken by CUVSLAM_CreateTracker(): %f",
      ssw_tracker.Stop());
    if (status != CUVSLAM_SUCCESS) {
      RCLCPP_ERROR(
        node.get_logger(),
        "Failed to initialize CUVSLAM tracker: %d", status);
      return;
    }

    pose_cache.Reset();

    // visualization helpers
    std::string camera_frame_for_publishing = node.base_frame_;
    if (!node.input_left_camera_frame_.empty()) {
      camera_frame_for_publishing = node.input_left_camera_frame_;
    }

    if (node.enable_slam_visualization_) {
      observations_vis_helper.Init(
        node.vis_observations_pub_, tracker,
        canonical_pose_cuvslam, camera_frame_for_publishing);
      landmarks_vis_helper.Init(
        node.vis_landmarks_pub_, tracker,
        initial_map_pose_left * canonical_pose_cuvslam, node.map_frame_);
      lc_landmarks_vis_helper.Init(
        node.vis_loop_closure_pub_, tracker,
        initial_map_pose_left * canonical_pose_cuvslam, node.map_frame_);
      pose_graph_helper.Init(
        node.vis_posegraph_nodes_pub_, node.vis_posegraph_edges_pub_,
        node.vis_posegraph_edges2_pub_, tracker,
        initial_map_pose_left * canonical_pose_cuvslam, node.map_frame_);
      localizer_helper.Init(
        node.vis_localizer_pub_, tracker,
        initial_map_pose_left * canonical_pose_cuvslam, node.map_frame_);
      localizer_landmarks_vis_helper.Init(
        node.vis_localizer_landmarks_pub_, tracker,
        initial_map_pose_left * canonical_pose_cuvslam, node.map_frame_);
      localizer_observations_vis_helper.Init(
        node.vis_localizer_observations_pub_, tracker,
        initial_map_pose_left * canonical_pose_cuvslam, node.map_frame_);
      localizer_lc_landmarks_vis_helper.Init(
        node.vis_localizer_loop_closure_pub_, tracker,
        initial_map_pose_left * canonical_pose_cuvslam, node.map_frame_);
    }
    cuvslam_handle = tracker;
  }
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
}

CUVSLAM_Configuration VisualSlamNode::VisualSlamImpl::GetConfiguration(
  const CUVSLAM_Pose & imu_pose_camera)
{
  CUVSLAM_Configuration configuration;
  CUVSLAM_InitDefaultConfiguration(&configuration);
  configuration.use_motion_model = 1;
  configuration.use_denoising = node.denoise_input_images_ ? 1 : 0;
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
  configuration.max_frame_delta_ms = node.img_jitter_threshold_ms_;
  configuration.verbosity = node.enable_verbosity_ ? 1 : 0;
  configuration.planar_constraints = node.force_planar_mode_ ? 1 : 0;

  CUVSLAM_ImuCalibration imu_calibration;
  imu_calibration.left_from_imu = imu_pose_camera;
  imu_calibration.gyroscope_noise_density = node.imu_params_.gyroscope_noise_density;
  imu_calibration.gyroscope_random_walk = node.imu_params_.gyroscope_random_walk;
  imu_calibration.accelerometer_noise_density = node.imu_params_.accelerometer_noise_density;
  imu_calibration.accelerometer_random_walk = node.imu_params_.accelerometer_random_walk;
  imu_calibration.frequency = node.imu_params_.calibration_frequency;
  configuration.imu_calibration = imu_calibration;

  return configuration;
}

// Helper function to set internal parameters for the cuvslam camera structure
void VisualSlamNode::VisualSlamImpl::SetcuVSLAMCameraPose(
  CUVSLAM_Camera & cam, const CUVSLAM_Pose & cuvslam_pose) {cam.pose = cuvslam_pose;}

void VisualSlamNode::VisualSlamImpl::ReadCameraIntrinsics(
  const CameraInfoType::ConstSharedPtr & msg_ci,
  CUVSLAM_Camera & cuvslam_camera, std::array<float, kMaxNumCameraParameters> & intrinsics)
{
  // Get pinhole
  cuvslam_camera.width = msg_ci->width;
  cuvslam_camera.height = msg_ci->height;
  cuvslam_camera.distortion_model = msg_ci->distortion_model.c_str();
  cuvslam_camera.num_parameters = 4;

  // the expected order is cx, cy, fx, fy
  const double * k = msg_ci->k.data();
  intrinsics[0] = k[2];
  intrinsics[1] = k[5];
  intrinsics[2] = k[0];
  intrinsics[3] = k[4];

  // Get lens distortion
  std::string distortion_model{msg_ci->distortion_model};
  const auto coefficients = msg_ci->d.data();
  if (distortion_model.empty() && Eigen::Vector3d(coefficients).isZero()) {
    // Continue with pinhole parameters
    RCLCPP_WARN(
      node.get_logger(), "Distortion Model is empty! Continuing with pinhole model.");
    distortion_model = "pinhole";
  }
  const auto distortion = g_str_to_distortion_model.find(distortion_model);
  if (distortion == std::end(g_str_to_distortion_model)) {
    RCLCPP_ERROR(
      node.get_logger(), "Unsupported distortion model[%s].", distortion_model.c_str());
    throw std::runtime_error("Unsupported distortion model.");
  } else {
    switch (distortion->second) {
      case (DistortionModel::BROWN): {
          if (Eigen::Vector3d(coefficients).isZero()) {
            // Continue with pinhole parameters
            cuvslam_camera.distortion_model = kPinhole;
            cuvslam_camera.num_parameters = 4;
          } else {
            cuvslam_camera.distortion_model = kBrown5k;

            const float k1 = coefficients[0];
            const float k2 = coefficients[1];
            const float t1 = coefficients[2];
            const float t2 = coefficients[3];
            const float k3 = coefficients[4];

            // 4-6: radial distortion coeffs (k1, k2, k3)
            intrinsics[4] = k1;
            intrinsics[5] = k2;
            intrinsics[6] = k3;

            // 7-8: tangential distortion coeffs (p1, p2)
            intrinsics[7] = t1;
            intrinsics[8] = t2;

            cuvslam_camera.num_parameters = 9;
          }
          break;
        }
      case (DistortionModel::FISHEYE): {
          cuvslam_camera.distortion_model = kFisheye4;
          // 4-7: fisheye distortion coeffs (k1, k2, k3, k4)
          intrinsics[4] = coefficients[0];
          intrinsics[5] = coefficients[1];
          intrinsics[6] = coefficients[2];
          intrinsics[7] = coefficients[3];

          cuvslam_camera.num_parameters = 8;
          break;
        }
      case (DistortionModel::PINHOLE): {
          cuvslam_camera.distortion_model = kPinhole;
          cuvslam_camera.num_parameters = 4;
          break;
        }
      case (DistortionModel::RATIONAL_POLYNOMIAL): {
          if (Eigen::Vector3d(coefficients).isZero()) {
            // Continue with pinhole parameters
            cuvslam_camera.distortion_model = kPinhole;
            cuvslam_camera.num_parameters = 4;
          } else {
            cuvslam_camera.distortion_model = kPolynomial;

            const float k1 = coefficients[0];
            const float k2 = coefficients[1];
            const float t1 = coefficients[2];
            const float t2 = coefficients[3];
            const float k3 = coefficients[4];
            const float k4 = coefficients[5];
            const float k5 = coefficients[6];
            const float k6 = coefficients[7];

            // 4-5 and 8-11: radial distortion coeffs (k1, k2, k3)
            intrinsics[4] = k1;
            intrinsics[5] = k2;
            intrinsics[8] = k3;
            intrinsics[9] = k4;
            intrinsics[10] = k5;
            intrinsics[11] = k6;

            // 6-7: tangential distortion coeffs (p1, p2)
            intrinsics[6] = t1;
            intrinsics[7] = t2;

            cuvslam_camera.num_parameters = 12;
          }
          break;
        }
      default: {
          RCLCPP_ERROR(node.get_logger(), "Unavailable input distortion model.");
          throw std::runtime_error("Unavailable input distortion model.");
        }
    }
  }
}

// Helper function to create CUVSLAM_Image from cv::Mat
CUVSLAM_Image VisualSlamNode::VisualSlamImpl::TocuVSLAMImage(
  int32_t camera_index, const cv::Mat & image, const int64_t & acqtime_ns)
{
  CUVSLAM_Image cuvslam_image;
  cuvslam_image.timestamp_ns = acqtime_ns;
  cuvslam_image.pixels = image.ptr();
  cuvslam_image.width = image.cols;
  cuvslam_image.height = image.rows;
  cuvslam_image.camera_index = camera_index;
  return cuvslam_image;
}

// Helper function to pass IMU data to cuVSLAM
CUVSLAM_ImuMeasurement VisualSlamNode::VisualSlamImpl::TocuVSLAMImuMeasurement(
  const ImuType::ConstSharedPtr & msg_imu)
{
  CUVSLAM_ImuMeasurement measurement;

  const auto & in_lin_acc = msg_imu->linear_acceleration;
  const auto & in_ang_vel = msg_imu->angular_velocity;

  const tf2::Vector3 ros_lin_acc(in_lin_acc.x, in_lin_acc.y, in_lin_acc.z);
  const tf2::Vector3 ros_ang_vel(in_ang_vel.x, in_ang_vel.y, in_ang_vel.z);

  // convert from ros frame to cuvslam
  const tf2::Vector3 cuvslam_lin_acc = cuvslam_pose_canonical * ros_lin_acc;
  const tf2::Vector3 cuvslam_ang_vel = cuvslam_pose_canonical * ros_ang_vel;

  // Set linear accelerations
  measurement.linear_accelerations[0] = cuvslam_lin_acc.x();
  measurement.linear_accelerations[1] = cuvslam_lin_acc.y();
  measurement.linear_accelerations[2] = cuvslam_lin_acc.z();
  // Set angular velocity
  measurement.angular_velocities[0] = cuvslam_ang_vel.x();
  measurement.angular_velocities[1] = cuvslam_ang_vel.y();
  measurement.angular_velocities[2] = cuvslam_ang_vel.z();

  return measurement;
}

// Helper function to publish source frame pose wrt target frame to the tf tree
void VisualSlamNode::VisualSlamImpl::PublishFrameTransform(
  rclcpp::Time stamp, const tf2::Transform & pose,
  const std::string & target, const std::string & source, bool is_static)
{
  geometry_msgs::msg::TransformStamped target_pose_source;
  target_pose_source.header.stamp = stamp;
  target_pose_source.header.frame_id = target;
  target_pose_source.child_frame_id = source;
  target_pose_source.transform = tf2::toMsg(pose);
  if (is_static) {
    tf_static_publisher->sendTransform(target_pose_source);
  } else {
    tf_publisher->sendTransform(target_pose_source);
  }
}

// Helper function to get child frame pose wrt parent frame from the tf tree
tf2::Transform VisualSlamNode::VisualSlamImpl::GetFrameTransform(
  rclcpp::Time stamp, const std::string & target, const std::string & source)
{
  geometry_msgs::msg::TransformStamped transform_stamped;
  tf2::Transform pose;

  // Time out duration for TF tree lookup before throwing an exception.
  const int32_t kTimeOutSeconds = 1;
  const uint32_t kTimeOutNanoSeconds = 0;
  try {
    if (!tf_buffer->canTransform(
        target, source, stamp, rclcpp::Duration(kTimeOutSeconds, kTimeOutNanoSeconds)))
    {
      RCLCPP_ERROR(
        node.get_logger(), "Transform is impossible. canTransform(%s->%s) returns false",
        target.c_str(), source.c_str());
    }
    transform_stamped = tf_buffer->lookupTransform(
      target, source, stamp);
    const tf2::Quaternion rotation(
      transform_stamped.transform.rotation.x,
      transform_stamped.transform.rotation.y,
      transform_stamped.transform.rotation.z,
      transform_stamped.transform.rotation.w);
    const tf2::Vector3 translation(
      transform_stamped.transform.translation.x,
      transform_stamped.transform.translation.y,
      transform_stamped.transform.translation.z);
    pose =
      tf2::Transform(rotation, translation);
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

double NanoToMilliSeconds(int64_t nano_sec) {return nano_sec / kNanoSecInMilliSec;}
bool VisualSlamNode::VisualSlamImpl::IsJitterAboveThreshold(double threshold)
{
  return (NanoToMilliSeconds(delta_ts.getData()[0]) - threshold) > 1.0f;
}

void VisualSlamNode::VisualSlamImpl::TrackAndGetPose(
  const ImageType::ConstSharedPtr & msg_left_img,
  const ImageType::ConstSharedPtr & msg_right_img)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "VisualSlamNode::VisualSlamImpl::TrackAndGetPose", nvidia::isaac_ros::nitros::CLR_MAGENTA);
  Stopwatch stopwatch;
  StopwatchScope ssw(stopwatch);
  rclcpp::Time timestamp(msg_left_img->header.stamp.sec, msg_left_img->header.stamp.nanosec);
  int64_t current_ts = static_cast<int64_t>(timestamp.nanoseconds());

  delta_ts.add(current_ts - last_img_ts);
  if (IsJitterAboveThreshold(node.img_jitter_threshold_ms_)) {
    RCLCPP_WARN(
      node.get_logger(), "Delta between current and previous frame [%f] is above threshold [%f]",
      NanoToMilliSeconds(delta_ts.getData()[0]), node.img_jitter_threshold_ms_);
  }

  // Convert frame to mono image
  const cv::Mat mono_left_img = cv_bridge::toCvShare(msg_left_img, "mono8")->image;
  const cv::Mat mono_right_img = cv_bridge::toCvShare(msg_right_img, "mono8")->image;

  // Track a new pair of frames
  Stopwatch stopwatch_track;
  if (IsInitialized()) {
    const int64_t acqtime_ns = current_ts;

    RCLCPP_DEBUG(node.get_logger(), "IMG msg timestamp [%ld]", acqtime_ns);

    cuvslam_images[LEFT_CAMERA_IDX] =
      TocuVSLAMImage(LEFT_CAMERA_IDX, mono_left_img, acqtime_ns);
    cuvslam_images[RIGHT_CAMERA_IDX] =
      TocuVSLAMImage(RIGHT_CAMERA_IDX, mono_right_img, acqtime_ns);

    StopwatchScope ssw_track(stopwatch_track);
    CUVSLAM_PoseEstimate pose_estimate;
    nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
      "CUVSLAM_Track", nvidia::isaac_ros::nitros::CLR_MAGENTA);
    const CUVSLAM_Status status =
      CUVSLAM_Track(cuvslam_handle, cuvslam_images.data(), nullptr, &pose_estimate);
    nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
    ssw_track.Stop();

    if (status == CUVSLAM_TRACKING_LOST) {
      pose_cache.Reset();
      RCLCPP_WARN(node.get_logger(), "Visual tracking is lost");
      nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
      return;
    }
    if (status != CUVSLAM_SUCCESS) {
      RCLCPP_WARN(node.get_logger(), "Unknown Tracker Error %d", status);
      nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
      return;
    }

    // Pure VO.
    tf2::Transform cuvslam_vo_pose = FromcuVSLAMPose(pose_estimate.pose);

    // Rectified pose in cuVSLAM coordinate system. It uses loop closure to get the robust tracking.
    // Note: It can result in sudden jumps of the final pose.
    // Publish Smooth pose if enable_rectified_pose_ = false
    tf2::Transform cuvslam_slam_pose = cuvslam_vo_pose;

    if (node.enable_localization_n_mapping_) {
      CUVSLAM_Pose pose_slam;
      const CUVSLAM_Status pose_slam_status = CUVSLAM_GetSlamPose(
        cuvslam_handle, &pose_slam);
      if (pose_slam_status != CUVSLAM_SUCCESS) {
        RCLCPP_WARN(node.get_logger(), "Get Rectified Pose Error %d", pose_slam_status);
        nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
        return;
      }
      cuvslam_slam_pose = FromcuVSLAMPose(pose_slam);
    }

    // Change of basis vectors for smooth pose
    const tf2::Transform ros_vo_pose{ChangeBasis(canonical_pose_cuvslam, cuvslam_vo_pose)};
    // Change of basis vectors for rectified pose
    tf2::Transform ros_slam_pose{ChangeBasis(canonical_pose_cuvslam, cuvslam_slam_pose)};

    const tf2::Transform odom_pose_base_link = initial_odom_pose_left * ros_vo_pose *
      base_link_pose_left.inverse();
    const tf2::Transform map_pose_base_link = initial_map_pose_left * ros_slam_pose *
      base_link_pose_left.inverse();

    // Frames hierarchy:
    // map - odom - base_link - ... - camera
    const tf2::Transform map_pose_odom = map_pose_base_link * odom_pose_base_link.inverse();


    rclcpp::Time timestamp_output = msg_left_img->header.stamp;
    if (node.override_publishing_stamp_) {
      timestamp_output = node.get_clock()->now();
    }
    if (node.publish_map_to_odom_tf_) {
      if (!node.invert_map_to_odom_tf_) {
        PublishFrameTransform(timestamp_output, map_pose_odom, node.map_frame_, node.odom_frame_);
      } else {
        PublishFrameTransform(
          timestamp_output,
          map_pose_odom.inverse(), node.odom_frame_, node.map_frame_);
      }
    }
    if (node.publish_odom_to_base_tf_) {
      if (!node.invert_odom_to_base_tf_) {
        PublishFrameTransform(
          timestamp_output, odom_pose_base_link, node.odom_frame_,
          node.base_frame_);
      } else {
        PublishFrameTransform(
          timestamp_output, odom_pose_base_link.inverse(),
          node.base_frame_, node.odom_frame_);
      }
    }
    pose_cache.Add(timestamp.nanoseconds(), odom_pose_base_link);

    geometry_msgs::msg::Twist velocity;
    // Extract timestamp from left image and create headers for
    // slam and vo poses
    std_msgs::msg::Header header_map = msg_left_img->header;
    header_map.frame_id = node.map_frame_;
    std_msgs::msg::Header header_odom = msg_left_img->header;
    header_odom.frame_id = node.odom_frame_;

    pose_cache.GetVelocity(
      velocity.linear.x, velocity.linear.y, velocity.linear.z,
      velocity.angular.x, velocity.angular.y, velocity.angular.z);
    velocity_cache.Add(
      velocity.linear.x, velocity.linear.y, velocity.linear.z,
      velocity.angular.x, velocity.angular.y, velocity.angular.z);


    // Pose Tracking

    PoseType vo_pose;
    tf2::toMsg(odom_pose_base_link, vo_pose);

    PoseType slam_pose;
    tf2::toMsg(map_pose_base_link, slam_pose);

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
      for (size_t i = 0; i < 6 * 6; i++) {
        pose_n_cov->pose.covariance[i] = static_cast<double>(pose_estimate.covariance[i]);
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
    // Visual_odometry_status_pub_
    ssw.Stop();
    if (HasSubscribers(node.visual_slam_status_pub_)) {
      const double track_execution_time = stopwatch_track.Seconds();
      track_execution_times.add(track_execution_time);

      double track_execution_time_max = 0;
      double track_execution_time_mean = 0;
      const auto values = track_execution_times.getData();
      for (double t : values) {
        track_execution_time_max = std::max(track_execution_time_max, t);
        track_execution_time_mean += t;
      }
      if (values.size()) {
        track_execution_time_mean /= values.size();
      }

      VisualSlamStatusType visual_slam_status_msg;
      visual_slam_status_msg.header = header_map;
      visual_slam_status_msg.vo_state = pose_estimate.vo_state;
      visual_slam_status_msg.node_callback_execution_time = stopwatch.Seconds();
      visual_slam_status_msg.track_execution_time = track_execution_time;
      visual_slam_status_msg.track_execution_time_max = track_execution_time_max;
      visual_slam_status_msg.track_execution_time_mean = track_execution_time_mean;
      node.visual_slam_status_pub_->publish(visual_slam_status_msg);
    }
  }
  last_img_ts = current_ts;
  nvidia::isaac_ros::nitros::nvtxRangePopWrapper();
}

void VisualSlamNode::VisualSlamImpl::RegisterImu(const ImuType::ConstSharedPtr & msg_imu)
{
  nvidia::isaac_ros::nitros::nvtxRangePushWrapper(
    "VisualSlamNode::VisualSlamImpl::RegisterImu", nvidia::isaac_ros::nitros::CLR_PURPLE);
  const auto measurement = TocuVSLAMImuMeasurement(msg_imu);
  const rclcpp::Time timestamp(msg_imu->header.stamp.sec, msg_imu->header.stamp.nanosec);
  const CUVSLAM_Status status = CUVSLAM_RegisterImuMeasurement(
    cuvslam_handle, static_cast<int64_t>(timestamp.nanoseconds()), &measurement);
  if (status != CUVSLAM_SUCCESS) {
    RCLCPP_WARN(node.get_logger(), "CUVSLAM has failed to register an IMU measurement");
  }
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
    const tf2::Transform ros_pos{ChangeBasis(
        impl->canonical_pose_cuvslam, FromcuVSLAMPose(*pose_in_db))};

    // publish_feedback
    if (context->goal_handle) {
      tf2::toMsg(ros_pos, result->pose);
      result->success = true;
      context->goal_handle->succeed(result);
    }

    // to localizer_helper
    const auto pt = ros_pos.getOrigin();
    RCLCPP_INFO(impl->node.get_logger(), "Localization result {%f, %f, %f}", pt[0], pt[1], pt[2]);
    impl->localizer_helper.SetResult(true, ros_pos);
  } else {
    // abort
    if (context->goal_handle) {
      context->goal_handle->abort(result);
    }
    // to localizer_helper
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
