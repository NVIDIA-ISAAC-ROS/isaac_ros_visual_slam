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

#ifndef ISAAC_ROS_VISUAL_SLAM__IMPL__CUVSLAM_ROS_CONVERSION_HPP_
#define ISAAC_ROS_VISUAL_SLAM__IMPL__CUVSLAM_ROS_CONVERSION_HPP_

#include <string>

#include "cuvslam.h" // NOLINT - include .h without directory
#include "cv_bridge/cv_bridge.h"
#include "isaac_ros_visual_slam/impl/types.hpp"
#include "tf2/LinearMath/Transform.h"
#include "eigen3/Eigen/Eigen"

namespace nvidia
{
namespace isaac_ros
{
namespace visual_slam
{

// Transformation converting from
// Canonical ROS Frame (x-forward, y-left, z-up) to
// cuVSLAM Frame       (x-right, y-up, z-backward)
// ROS    ->  cuVSLAM
//  x     ->    -z
//  y     ->    -x
//  z     ->     y
const tf2::Transform cuvslam_pose_canonical(tf2::Matrix3x3(
    0, -1, 0,
    0, 0, 1,
    -1, 0, 0
));

// Transformation converting from
// cuVSLAM Frame       (x-right, y-up, z-backward) to
// Canonical ROS Frame (x-forward, y-left, z-up)
const tf2::Transform canonical_pose_cuvslam(cuvslam_pose_canonical.inverse());

// Transformation converting from
// Optical Frame    (x-right, y-down, z-forward) to
// cuVSLAM Frame    (x-right, y-up, z-backward)
// Optical   ->  cuVSLAM
//    x      ->     x
//    y      ->    -y
//    z      ->    -z
const tf2::Transform cuvslam_pose_optical(tf2::Matrix3x3(
    1, 0, 0,
    0, -1, 0,
    0, 0, -1
));

// Transformation converting from
// cuVSLAM Frame    (x-right, y-up, z-backward) to
// Optical Frame    (x-right, y-down, z-forward)
const tf2::Transform optical_pose_cuvslam(cuvslam_pose_optical.inverse());

tf2::Transform ChangeBasis(
  const tf2::Transform & target_pose_source, const tf2::Transform & source_pose_source);

CUVSLAM_Pose TocuVSLAMPose(const tf2::Transform & tf_mat);
tf2::Transform FromcuVSLAMPose(const CUVSLAM_Pose & cuvslam_pose);

CUVSLAM_ImageEncoding TocuVSLAMImageEncoding(const std::string & image_encoding);

CUVSLAM_Image TocuVSLAMImage(
  int32_t camera_index, const ImageType & image_view, const int64_t & acqtime_ns);

CUVSLAM_ImuMeasurement TocuVSLAMImuMeasurement(const ImuType::ConstSharedPtr & msg_imu);

void FillIntrinsics(const CameraInfoType::ConstSharedPtr & msg, CUVSLAM_Camera & camera);

void FillExtrinsics(const tf2::Transform & base_pose_camera_optical, CUVSLAM_Camera & camera);

Eigen::Matrix<float, 6, 6> FromcuVSLAMCovariance(float covariance[36]);

}  // namespace visual_slam
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_VISUAL_SLAM__IMPL__CUVSLAM_ROS_CONVERSION_HPP_
