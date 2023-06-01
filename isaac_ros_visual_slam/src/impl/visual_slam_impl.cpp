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
#include "isaac_ros_visual_slam/impl/cuvslam_ros_convertion.hpp"
#include "isaac_ros_visual_slam/impl/stopwatch.hpp"
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
}  // namespace


namespace isaac_ros
{
namespace visual_slam
{

VisualSlamNode::VisualSlamImpl::VisualSlamImpl(VisualSlamNode & node)
: logger_name(std::string(node.get_logger().get_name())),
  node_clock(node.get_clock()),
  // Set cuvslam_pose_base_link for converting ROS coordinate to cuVSLAM coordinate
  // ROS    ->  cuVSLAM
  //  x     ->    -z
  //  y     ->    -x
  //  z     ->     y
  cuvslam_pose_base_link(tf2::Matrix3x3(
      0, -1, 0,
      0, 0, 1,
      -1, 0, 0
    )),
  base_link_pose_cuvslam(cuvslam_pose_base_link.inverse()),
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
  tf_buffer(std::make_unique<tf2_ros::Buffer>(node_clock)),
  tf_listener(std::make_shared<tf2_ros::TransformListener>(*tf_buffer)),
  tf_publisher(std::make_unique<tf2_ros::TransformBroadcaster>(&node)),
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
  last_img_ts(0)
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
  VisualSlamNode & node,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg_left_ci,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg_right_ci)
{
  if (cuvslam_handle == nullptr) {
    const rclcpp::Time & stamp = msg_left_ci->header.stamp;

    std::string in_baselink = node.input_base_frame_;
    if (in_baselink.empty()) {
      in_baselink = node.base_frame_;
    }

    left_pose_base_link = tf2::Transform::getIdentity();
    if (!node.input_left_camera_frame_.empty() && !in_baselink.empty()) {
      left_pose_base_link = GetFrameTransform(stamp, node.input_left_camera_frame_, in_baselink);
    }

    // Set start_odom_pose to such pose that base_link is identity.
    // Start pose of visual slam = left_pose_base_link.
    start_odom_pose = left_pose_base_link.inverse();

    tf2::Transform left_pose_right;
    if (!node.input_right_camera_frame_.empty()) {
      RCLCPP_INFO(rclcpp::get_logger(logger_name), "left_pose_right reading from TF Tree");
      left_pose_right =
        ChangeBasis(
        cuvslam_pose_base_link,
        GetFrameTransform(stamp, node.input_left_camera_frame_, node.input_right_camera_frame_));
    } else {
      RCLCPP_INFO(rclcpp::get_logger(logger_name), "left_pose_right reading from CameraInfo");

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
      RCLCPP_INFO(rclcpp::get_logger(logger_name), "left_camera_pose_imu reading from TF Tree");
      std::string left_cam_frame = node.input_left_camera_frame_;

      // Left camera in the robot's center
      if (left_cam_frame.empty()) {
        left_cam_frame = in_baselink;
      }

      // Convert the left_camera_pose_imu from ROS to cuVSLAM frame
      left_camera_pose_imu = ChangeBasis(
        cuvslam_pose_base_link,
        GetFrameTransform(stamp, left_cam_frame, node.input_imu_frame_));
    }

    // cuVSLAM configuration needs Left pose IMU camera
    const CUVSLAM_Configuration configuration =
      GetConfiguration(node, TocuVSLAMPose(left_camera_pose_imu));

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
    PrintConfiguration(rclcpp::get_logger(logger_name), configuration);
    Stopwatch stopwatch_tracker;
    StopwatchScope ssw_tracker(stopwatch_tracker);
    const CUVSLAM_Status status = CUVSLAM_CreateTracker(&tracker, &rig, &configuration);
    RCLCPP_INFO(
      rclcpp::get_logger(logger_name), "Time taken by CUVSLAM_CreateTracker(): %f",
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
        node.vis_observations_pub_,
        tracker, base_link_pose_cuvslam, node, camera_frame_for_publishing);
      landmarks_vis_helper.Init(
        node.vis_landmarks_pub_,
        tracker, base_link_pose_cuvslam, node, node.map_frame_);
      lc_landmarks_vis_helper.Init(
        node.vis_loop_closure_pub_,
        tracker, base_link_pose_cuvslam, node, node.map_frame_);
      pose_graph_helper.Init(
        node.vis_posegraph_nodes_pub_, node.vis_posegraph_edges_pub_,
        node.vis_posegraph_edges2_pub_, tracker, base_link_pose_cuvslam, node, node.map_frame_);
      localizer_helper.Init(
        node.vis_localizer_pub_,
        tracker, base_link_pose_cuvslam, node, node.map_frame_);
      localizer_landmarks_vis_helper.Init(
        node.vis_localizer_landmarks_pub_,
        tracker, base_link_pose_cuvslam, node, node.map_frame_);
      localizer_observations_vis_helper.Init(
        node.vis_localizer_observations_pub_,
        tracker, base_link_pose_cuvslam, node, node.map_frame_);
      localizer_lc_landmarks_vis_helper.Init(
        node.vis_localizer_loop_closure_pub_,
        tracker, base_link_pose_cuvslam, node, node.map_frame_);
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
    RCLCPP_INFO(rclcpp::get_logger(logger_name), "cuVSLAM tracker was destroyed");
  }
}

CUVSLAM_Configuration VisualSlamNode::VisualSlamImpl::GetConfiguration(
  const VisualSlamNode & node,
  const CUVSLAM_Pose & imu_pose_camera)
{
  CUVSLAM_Configuration configuration;
  CUVSLAM_InitDefaultConfiguration(&configuration);
  configuration.use_motion_model = 1;
  configuration.use_denoising = node.denoise_input_images_ ? 1 : 0;
  configuration.horizontal_stereo_camera = node.rectified_images_ ? 1 : 0;
  configuration.enable_observations_export = node.enable_observations_view_ ? 1 : 0;
  if (node.enable_debug_mode_) {configuration.debug_dump_directory = node.debug_dump_path_.c_str();}
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
  CUVSLAM_Camera & cam,
  const CUVSLAM_Pose & cuvslam_pose) {cam.pose = cuvslam_pose;}

void VisualSlamNode::VisualSlamImpl::ReadCameraIntrinsics(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg_ci,
  CUVSLAM_Camera & cuvslam_camera,
  std::array<float, kMaxNumCameraParameters> & intrinsics)
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
      rclcpp::get_logger(
        logger_name), "Distortion Model is empty! Continuing with pinhole model.");
    distortion_model = "pinhole";
  }
  const auto distortion = g_str_to_distortion_model.find(distortion_model);
  if (distortion == std::end(g_str_to_distortion_model)) {
    RCLCPP_ERROR(
      rclcpp::get_logger(logger_name),
      "Unsupported distortion model[%s].", distortion_model.c_str());
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
          RCLCPP_ERROR(rclcpp::get_logger(logger_name), "Unavailable input distortion model.");
          throw std::runtime_error("Unavailable input distortion model.");
        }
    }
  }
}

// Helper function to create CUVSLAM_Image from cv::Mat
CUVSLAM_Image VisualSlamNode::VisualSlamImpl::TocuVSLAMImage(
  int32_t camera_index,
  const cv::Mat & image,
  const int64_t & acqtime_ns)
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
  const sensor_msgs::msg::Imu::ConstSharedPtr & msg_imu)
{
  CUVSLAM_ImuMeasurement measurement;

  const auto & in_lin_acc = msg_imu->linear_acceleration;
  const auto & in_ang_vel = msg_imu->angular_velocity;

  const tf2::Vector3 ros_lin_acc(in_lin_acc.x, in_lin_acc.y, in_lin_acc.z);
  const tf2::Vector3 ros_ang_vel(in_ang_vel.x, in_ang_vel.y, in_ang_vel.z);

  // convert from ros frame to cuvslam
  const tf2::Vector3 cuvslam_lin_acc = cuvslam_pose_base_link * ros_lin_acc;
  const tf2::Vector3 cuvslam_ang_vel = cuvslam_pose_base_link * ros_ang_vel;

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
  rclcpp::Time stamp,
  const tf2::Transform & pose,
  const std::string & target,
  const std::string & source)
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
  rclcpp::Time stamp,
  const std::string & target,
  const std::string & source)
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
        rclcpp::get_logger(
          logger_name), "Transform is impossible. canTransform(%s->%s) returns false",
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
      rclcpp::get_logger(logger_name), "Could not transform %s to %s: %s",
      source.c_str(), target.c_str(), ex.what());
    throw std::runtime_error("Could not find the requested transform!");
  }
  return pose;
}

void VisualSlamNode::VisualSlamImpl::PublishOdometryVelocity(
  rclcpp::Time stamp,
  const std::string & frame_id,
  const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher)
{
  geometry_msgs::msg::Twist twist;

  pose_cache.GetVelocity(
    twist.linear.x, twist.linear.y, twist.linear.z,
    twist.angular.x, twist.angular.y, twist.angular.z);

  visualization_msgs::msg::MarkerArray markers;
  markers.markers.resize(2);

  float width = 0.02f;
  {
    visualization_msgs::msg::Marker & m = markers.markers[0];
    m.header.frame_id = frame_id;
    m.header.stamp = stamp;
    m.ns = "linear velocity";
    m.id = 0;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.type = visualization_msgs::msg::Marker::ARROW;
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
    visualization_msgs::msg::Marker & m = markers.markers[1];
    m.header.frame_id = frame_id;
    m.header.stamp = stamp;
    m.ns = "angular velocity";
    m.id = 0;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.type = visualization_msgs::msg::Marker::ARROW;
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
  rclcpp::Time stamp,
  const std::string & frame_id,
  const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher)
{
  CUVSLAM_Gravity gravity_in_cuvslam;

  if (CUVSLAM_SUCCESS != CUVSLAM_GetLastGravity(cuvslam_handle, &gravity_in_cuvslam)) {
    // Don't publish anything if cuvslam hasn't calculated gravity yet. After the start
    // or loss of visual tracking, cuvslam needs some time to align optical and imu sensors.
    // Only after that is it able to calculate the gravity vector.
    return;
  }

  const tf2::Vector3 g_cuvslam(gravity_in_cuvslam.x, gravity_in_cuvslam.y, gravity_in_cuvslam.z);
  const tf2::Vector3 g_base_link = base_link_pose_cuvslam * g_cuvslam;

  visualization_msgs::msg::Marker m;
  m.header.frame_id = frame_id;
  m.header.stamp = stamp;
  m.ns = "gravity";
  m.id = 0;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.type = visualization_msgs::msg::Marker::ARROW;
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

}  // namespace visual_slam
}  // namespace isaac_ros
