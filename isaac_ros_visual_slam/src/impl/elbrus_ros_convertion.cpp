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

#include "isaac_ros_visual_slam/impl/elbrus_ros_convertion.hpp"

namespace isaac_ros
{
namespace visual_slam
{

// Helper function that converts transform into ELBRUS_Pose
ELBRUS_Pose ToElbrusPose(const tf2::Transform & tf_mat)
{
  ELBRUS_Pose elbrusPose;
  // tf2::Matrix3x3 is row major, but elbrus is column major
  const tf2::Matrix3x3 rotation(tf_mat.getRotation());
  const int32_t kRotationMatCol = 3;
  const int32_t kRotationMatRow = 3;
  int elbrus_idx = 0;
  for (int col_idx = 0; col_idx < kRotationMatCol; ++col_idx) {
    const tf2::Vector3 & rot_col = rotation.getColumn(col_idx);
    for (int row_idx = 0; row_idx < kRotationMatRow; ++row_idx) {
      elbrusPose.r[elbrus_idx] = rot_col[row_idx];
      elbrus_idx++;
    }
  }

  const tf2::Vector3 & translation = tf_mat.getOrigin();
  elbrusPose.t[0] = translation.x();
  elbrusPose.t[1] = translation.y();
  elbrusPose.t[2] = translation.z();
  return elbrusPose;
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

// Helper function to convert elbrus pose estimate to tf2::Transform
tf2::Transform FromElbrusPose(
  const ELBRUS_Pose & elbrus_pose)
{
  const auto & r = elbrus_pose.r;
  // tf2::Matrix3x3 is row major and Elbrus rotation mat is column major.
  const tf2::Matrix3x3 rotation(r[0], r[3], r[6], r[1], r[4], r[7], r[2], r[5], r[8]);
  const tf2::Vector3 translation(elbrus_pose.t[0], elbrus_pose.t[1], elbrus_pose.t[2]);

  return tf2::Transform(rotation, translation);
}

}  // namespace visual_slam
}  // namespace isaac_ros
