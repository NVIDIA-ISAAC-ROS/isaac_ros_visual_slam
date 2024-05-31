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

#include "eigen3/Eigen/Dense"
#include "isaac_ros_visual_slam/impl/cuvslam_ros_conversion.hpp"
#include "isaac_ros_visual_slam/impl/types.hpp"
#include "sensor_msgs/distortion_models.hpp"
#include "rclcpp/logging.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace visual_slam
{

constexpr char kFisheye[] = "fisheye4";
constexpr char kBrown[] = "brown5k";
constexpr char kPinhole[] = "pinhole";
constexpr char kPolynomial[] = "polynomial";

// ROS' DISTORTION MODELS:
// 1.PLUMB_BOB
// 2.EQUIDISTANT
// 3.RATIONAL_POLYNOMIAL
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

template<DistortionModel T = DistortionModel::PINHOLE>
void FillDistortion(const CameraInfoType::ConstSharedPtr & msg, CUVSLAM_Camera & camera)
{
  camera.distortion_model = kPinhole;
  camera.num_parameters = 4;
  float * mutable_parameters = const_cast<float *>(camera.parameters);
  // 0-3: center point and focal length coeffs.
  mutable_parameters[0] = msg->k[2];  // cx
  mutable_parameters[1] = msg->k[5];  // cy
  mutable_parameters[2] = msg->k[0];  // fx
  mutable_parameters[3] = msg->k[4];  // fy
}

template<>
void FillDistortion<DistortionModel::BROWN>(
  const CameraInfoType::ConstSharedPtr & msg, CUVSLAM_Camera & camera)
{
  camera.distortion_model = kBrown;
  camera.num_parameters = 9;
  float * mutable_parameters = const_cast<float *>(camera.parameters);
  // 0-3: center point and focal length coeffs.
  mutable_parameters[0] = msg->k[2];  // cx
  mutable_parameters[1] = msg->k[5];  // cy
  mutable_parameters[2] = msg->k[0];  // fx
  mutable_parameters[3] = msg->k[4];  // fy
  // 4-6: radial distortion coeffs.
  mutable_parameters[4] = msg->d[0];  // k1
  mutable_parameters[5] = msg->d[1];  // k2
  mutable_parameters[6] = msg->d[4];  // k3
  // 7-8: tangential distortion coeffs.
  mutable_parameters[7] = msg->d[2];  // p1
  mutable_parameters[8] = msg->d[3];  // p2
}

template<>
void FillDistortion<DistortionModel::FISHEYE>(
  const CameraInfoType::ConstSharedPtr & msg, CUVSLAM_Camera & camera)
{
  camera.distortion_model = kFisheye;
  camera.num_parameters = 8;
  float * mutable_parameters = const_cast<float *>(camera.parameters);
  // 0-3: center point and focal length coeffs.
  mutable_parameters[0] = msg->k[2];  // cx
  mutable_parameters[1] = msg->k[5];  // cy
  mutable_parameters[2] = msg->k[0];  // fx
  mutable_parameters[3] = msg->k[4];  // fy
  // 4-8: fisheye distortion coeffs.
  mutable_parameters[4] = msg->d[0];  // k1
  mutable_parameters[5] = msg->d[1];  // k2
  mutable_parameters[6] = msg->d[2];  // k3
  mutable_parameters[7] = msg->d[3];  // k4
}

template<>
void FillDistortion<DistortionModel::RATIONAL_POLYNOMIAL>(
  const CameraInfoType::ConstSharedPtr & msg, CUVSLAM_Camera & camera)
{
  camera.distortion_model = kPolynomial;
  camera.num_parameters = 12;
  float * mutable_parameters = const_cast<float *>(camera.parameters);
  // 0-3: center point and focal length coeffs.
  mutable_parameters[0] = msg->k[2];  // cx
  mutable_parameters[1] = msg->k[5];  // cy
  mutable_parameters[2] = msg->k[0];  // fx
  mutable_parameters[3] = msg->k[4];  // fy
  // 4-5: radial distortion coeffs.
  mutable_parameters[4] = msg->d[0];  // k1
  mutable_parameters[5] = msg->d[1];  // k2
  // 6-7: tangential distortion coeffs.
  mutable_parameters[6] = msg->d[2];  // t1
  mutable_parameters[7] = msg->d[3];  // t2
  // 8-11: radial distortion coeffs (again).
  mutable_parameters[8] = msg->d[4];  // k3
  mutable_parameters[9] = msg->d[5];  // k4;
  mutable_parameters[10] = msg->d[6];  // k5;
  mutable_parameters[11] = msg->d[7];  // k6;
}

void FillCameraDistortion(const CameraInfoType::ConstSharedPtr & msg, CUVSLAM_Camera & camera)
{
  // Fallback to pinhole distortion if no distortion was provided.
  std::string distortion_model_name = msg->distortion_model;
  const auto distortion_params = msg->d.data();
  if (distortion_model_name.empty() &&
    Eigen::Vector3d(distortion_params).isZero())
  {
    distortion_model_name = "pinhole";
  }

  if (distortion_model_name == sensor_msgs::distortion_models::PLUMB_BOB) {
    return FillDistortion<DistortionModel::BROWN>(msg, camera);
  }
  if (distortion_model_name == sensor_msgs::distortion_models::EQUIDISTANT) {
    return FillDistortion<DistortionModel::FISHEYE>(msg, camera);
  }
  if (distortion_model_name == "pinhole") {
    return FillDistortion<DistortionModel::PINHOLE>(msg, camera);
  }
  if (distortion_model_name == sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL) {
    return FillDistortion<DistortionModel::RATIONAL_POLYNOMIAL>(msg, camera);
  }
  throw std::runtime_error("Unsupported distortion model.");
}

void FillIntrinsics(const CameraInfoType::ConstSharedPtr & msg, CUVSLAM_Camera & camera)
{
  camera.width = msg->width;
  camera.height = msg->height;
  FillCameraDistortion(msg, camera);
}

void FillExtrinsics(const tf2::Transform & base_pose_camera_optical, CUVSLAM_Camera & camera)
{
  // Converting base_pose_camera_optical into cuvslam frame.
  // Below equation represents base in cuvslam frame and also camera_optical in cuvslam frame
  const tf2::Transform cv_base_pose_cv_camera_optical =
    cuvslam_pose_canonical * base_pose_camera_optical * optical_pose_cuvslam;

  camera.pose = TocuVSLAMPose(cv_base_pose_cv_camera_optical);
}

// Helper function that converts transform into CUVSLAM_Pose
CUVSLAM_Pose TocuVSLAMPose(const tf2::Transform & tf_mat)
{
  CUVSLAM_Pose cuvslamPose;
  // tf2::Matrix3x3 is row major, but cuvslam is column major
  const tf2::Matrix3x3 rotation(tf_mat.getRotation());
  const int32_t kRotationMatCol = 3;
  const int32_t kRotationMatRow = 3;
  int cuvslam_idx = 0;
  for (int col_idx = 0; col_idx < kRotationMatCol; ++col_idx) {
    const tf2::Vector3 & rot_col = rotation.getColumn(col_idx);
    for (int row_idx = 0; row_idx < kRotationMatRow; ++row_idx) {
      cuvslamPose.r[cuvslam_idx] = rot_col[row_idx];
      cuvslam_idx++;
    }
  }

  const tf2::Vector3 & translation = tf_mat.getOrigin();
  cuvslamPose.t[0] = translation.x();
  cuvslamPose.t[1] = translation.y();
  cuvslamPose.t[2] = translation.z();
  return cuvslamPose;
}

// Helper function to convert cuvslam pose estimate to tf2::Transform
tf2::Transform FromcuVSLAMPose(
  const CUVSLAM_Pose & cuvslam_pose)
{
  const auto & r = cuvslam_pose.r;
  // tf2::Matrix3x3 is row major and cuVSLAM rotation mat is column major.
  const tf2::Matrix3x3 rotation(r[0], r[3], r[6], r[1], r[4], r[7], r[2], r[5], r[8]);
  const tf2::Vector3 translation(cuvslam_pose.t[0], cuvslam_pose.t[1], cuvslam_pose.t[2]);

  return tf2::Transform(rotation, translation);
}

// Helper funtion to change basis from frame source to frame target
// target_pose_source = Transformation(Rotation only; translation is zero) matrix between target
// and source.
// source_pose_source = Transformation(Rotation and translation) inside source frame.
// It is any arbritary transformation in source frame.
tf2::Transform ChangeBasis(
  const tf2::Transform & target_pose_source, const tf2::Transform & source_pose_source)
{
  return target_pose_source * source_pose_source * target_pose_source.inverse();
}

// Helper function to create CUVSLAM_ImageEncoding from string using sensor_msgs image encoding.
CUVSLAM_ImageEncoding TocuVSLAMImageEncoding(const std::string & image_encoding)
{
  if (image_encoding == sensor_msgs::image_encodings::MONO8) {
    return CUVSLAM_ImageEncoding::MONO8;
  }
  if (image_encoding == sensor_msgs::image_encodings::RGB8) {
    return CUVSLAM_ImageEncoding::RGB8;
  }
  throw std::invalid_argument("Received unknown image encoding: " + image_encoding);
}

// Helper function to create CUVSLAM_Image from NitrosImageView.
CUVSLAM_Image TocuVSLAMImage(
  int32_t camera_index, const ImageType & image_view, const int64_t & acqtime_ns)
{
  CUVSLAM_Image cuvslam_image;
  cuvslam_image.timestamp_ns = acqtime_ns;
  cuvslam_image.pixels = image_view.GetGpuData();
  cuvslam_image.width = image_view.GetWidth();
  cuvslam_image.height = image_view.GetHeight();
  cuvslam_image.camera_index = camera_index;
  cuvslam_image.pitch = image_view.GetStride();
  cuvslam_image.image_encoding = TocuVSLAMImageEncoding(image_view.GetEncoding());
  return cuvslam_image;
}

// Helper function to pass IMU data to cuVSLAM
CUVSLAM_ImuMeasurement TocuVSLAMImuMeasurement(const ImuType::ConstSharedPtr & msg_imu)
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

Eigen::Matrix<float, 6, 6> FromcuVSLAMCovariance(float covariance[36])
{
  tf2::Quaternion quat = canonical_pose_cuvslam.getRotation();
  Eigen::Quaternion<float> q(quat.w(), quat.x(), quat.y(), quat.z());
  Eigen::Matrix<float, 3, 3> canonical_pose_cuvslam_mat = q.matrix();

  Eigen::Matrix<float, 6, 6> block_canonical_pose_cuvslam = Eigen::Matrix<float, 6, 6>::Zero();
  block_canonical_pose_cuvslam.block<3, 3>(0, 0) = canonical_pose_cuvslam_mat;
  block_canonical_pose_cuvslam.block<3, 3>(3, 3) = canonical_pose_cuvslam_mat;
  Eigen::Matrix<float, 6, 6> covariance_mat =
    Eigen::Map<Eigen::Matrix<float, 6, 6, Eigen::StorageOptions::AutoAlign>>(covariance);
  Eigen::Matrix<float, 6, 6> ros_covariance_mat = Eigen::Matrix<float, 6, 6>::Zero();

  // The covariance matrix from cuVSLAM arranges elements as follows:
  // (rotation about X axis, rotation about Y axis, rotation about Z axis, x, y, z)
  // However, in ROS, the order is:
  // (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
  ros_covariance_mat.block<3, 3>(0, 0) = covariance_mat.block<3, 3>(3, 3);
  ros_covariance_mat.block<3, 3>(0, 3) = covariance_mat.block<3, 3>(3, 0);
  ros_covariance_mat.block<3, 3>(3, 0) = covariance_mat.block<3, 3>(0, 3);
  ros_covariance_mat.block<3, 3>(3, 3) = covariance_mat.block<3, 3>(0, 0);
  Eigen::Matrix<float, 6, 6> covariance_mat_change_basis =
    block_canonical_pose_cuvslam * ros_covariance_mat * block_canonical_pose_cuvslam.transpose();
  return covariance_mat_change_basis;
}

}  // namespace visual_slam
}  // namespace isaac_ros
}  // namespace nvidia
