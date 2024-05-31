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

#include <math.h>

#include <cstring>
#include <vector>

#include "isaac_ros_visual_slam/impl/pose_cache.hpp"

namespace
{
using Scalar = double;
void getTranslationAndEulerAngles(
  const tf2::Transform & t,
  Scalar & x, Scalar & y, Scalar & z,
  Scalar & roll, Scalar & pitch, Scalar & yaw)
{
  const tf2::Vector3 & translate = t.getOrigin();
  x = translate[0];
  y = translate[1];
  z = translate[2];


  /*
  Quaternion to RollPitchYaw transition;
   @TECHREPORT{blanco2010se3,
      author = {Blanco, Jos{\'{e}}-Luis},
      month = sep,
      title = {A tutorial on SE(3) transformation parameterizations and on-manifold optimization},
      year = {2010},
      institution = {University of Malaga}
  */

  tf2::Quaternion q = t.getRotation();
  q.normalize();

  double determinant = q.w() * q.y() - q.x() * q.z();

  if (std::abs(determinant) < 0.5 - 1e-2) {
    // We calculate jacobians on this functions below;
    roll = atan2(
      2.0 * (q.w() * q.x() + q.y() * q.z()),
      (1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y())));
    pitch = asin(2.0 * (q.w() * q.y() - q.x() * q.z()));
    yaw =
      atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), (1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z())));
  } else {
    roll = 0;
    if (determinant < 0) {
      pitch = -M_PI / 2;
      yaw = 2.0 * atan2(q.x(), q.w());
    } else {
      pitch = M_PI / 2;
      yaw = -2.0 * atan2(q.x(), q.w());
    }
  }
}

void QuaternionCovToRollPitchYawCov(
  const std::array<double, 7 * 7> & quat_cov,
  const tf2::Quaternion & mean_quat,
  std::array<double, 6 * 6> & cov)
{
  // Jacobian
  double J[6][7];

  // Set identity;
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 7; j++) {
      J[i][j] = i == j ? 1 : 0;
    }
  }

  double determinant = mean_quat.w() * mean_quat.y() - mean_quat.x() * mean_quat.z();

  if (std::abs(determinant) < 0.5 - 1e-2) {
    double t1 = 1 - 2 * (mean_quat.x() * mean_quat.x() + mean_quat.y() * mean_quat.y());
    double t2 = 2 * (mean_quat.w() * mean_quat.x() + mean_quat.y() * mean_quat.z());
    double t3 = 1 / (t1 * t1 + t2 * t2);

    double t4 = 1 / sqrt(1 - 4 * pow(determinant, 2));

    double t5 = 1.0 - 2.0 * (mean_quat.y() * mean_quat.y() + mean_quat.z() * mean_quat.z());
    double t6 = 2.0 * (mean_quat.w() * mean_quat.z() + mean_quat.x() * mean_quat.y());
    double t7 = 1 / (t5 * t5 + t6 * t6);

    J[3][3] = (2 * mean_quat.w() * t1 + 4 * mean_quat.x() * t2) * t3;      // d roll / d q.x
    J[3][4] = (2 * mean_quat.z() * t1 + 4 * mean_quat.y() * t2) * t3;      // d roll / d q.y
    J[3][5] = 2 * mean_quat.y() * t1 * t3;                                 // d roll / d q.z
    J[3][6] = 2 * mean_quat.x() * t1 * t3;                                 // d roll / d q.w

    J[4][3] = -2 * mean_quat.z() * t4;                                     // d pitch / d q.x
    J[4][4] = 2 * mean_quat.w() * t4;                                      // d pitch / d q.y
    J[4][5] = -2 * mean_quat.x() * t4;                                     // d pitch / d q.z
    J[4][6] = 2 * mean_quat.y() * t4;                                      // d pitch / d q.w

    J[5][3] = 2 * mean_quat.y() * t5 * t7;                                 // d yaw / d q.x
    J[5][4] = (2 * mean_quat.x() * t5 + 4 * mean_quat.y() * t6) * t7;      // d yaw / d q.y
    J[5][5] = (2 * mean_quat.w() * t5 + 4 * mean_quat.z() * t6) * t7;      // d yaw / d q.z
    J[5][6] = 2 * mean_quat.z() * t5 * t7;                                 // d yaw / d q.w
  } else {
    double t1 = mean_quat.w() * mean_quat.w();
    double t2 = mean_quat.x() * mean_quat.x();
    double t4 = 1 / (t1 + t2);

    J[3][3] = 0;
    J[3][4] = 0;
    J[3][5] = 0;
    J[3][6] = 0;

    J[4][3] = 0;
    J[4][4] = 0;
    J[4][5] = 0;
    J[4][6] = 0;

    J[5][4] = 0;
    J[5][5] = 0;

    if (determinant < 0) {
      // roll  = 0
      // pitch = -M_PI/2.0
      // yaw   = 2.0 * atan2(q.x(), q.w())

      J[5][3] = 2 * mean_quat.w() * t4;         // d yaw / d q.x
      J[5][6] = -2 * mean_quat.x() * t4;        // d yaw / d q.w
    } else {
      // roll  = 0;
      // pitch = M_PI/2.0;
      // yaw   = -2.0 * atan2(q.x(), q.w());
      J[5][3] = -2 * mean_quat.w() * t4;        // d yaw / d q.x
      J[5][6] = 2 * mean_quat.x() * t4;         // d yaw / d q.w
    }
  }

  // Calculate J * covariance * J.transpose()
  {
    double cov_J_transposed[7][6];     //  covariance * J.transpose()
    for (int i = 0; i < 7; i++) {
      for (int j = 0; j < 6; j++) {
        double t = 0;
        for (int k = 0; k < 7; k++) {
          t += quat_cov[i * 7 + k] * J[j][k] /*J transposed*/;
        }
        cov_J_transposed[i][j] = t;
      }
    }

    for (int i = 0; i < 6; i++) {
      for (int j = 0; j < 6; j++) {
        double t = 0;
        for (int k = 0; k < 7; k++) {
          t += J[i][k] * cov_J_transposed[k][j];
        }
        cov[i * 6 + j] = t;
      }
    }
  }
}
}  //  namespace

namespace nvidia
{
namespace isaac_ros
{
namespace visual_slam
{
void PoseCache::Reset()
{
  poses_.clear();
}
void PoseCache::Add(int64_t timestamp, const tf2::Transform & pose)
{
  poses_.push_back({timestamp, pose});

  while (poses_.size() > num_poses_to_keep_) {
    poses_.pop_front();
  }
}

bool PoseCache::GetVelocity(
  double & x, double & y, double & z, double & roll, double & pitch,
  double & yaw) const
{
  if (poses_.size() < 2) {
    x = 0;
    y = 0;
    z = 0;
    roll = 0;
    pitch = 0;
    yaw = 0;
    return false;
  }
  // diff
  auto & it0 = poses_.front();
  auto & it1 = poses_.back();

  const tf2::Transform dp = it0.second.inverse() * it1.second;
  const double dt = (it1.first - it0.first) * 1e-9;
  const double dt_inv = 1. / dt;

  getTranslationAndEulerAngles(dp, x, y, z, roll, pitch, yaw);

  x *= dt_inv;
  y *= dt_inv;
  z *= dt_inv;
  roll *= dt_inv;
  pitch *= dt_inv;
  yaw *= dt_inv;
  return true;
}

bool PoseCache::GetCovariance(std::array<double, 6 * 6> & cov) const
{
  if (poses_.size() < 10) {
    return false;
  }

  std::array<double, 7 * 7> pose_quat_covariance;
  pose_quat_covariance.fill(0.0);

  tf2::Quaternion mean_quat;
  {
    std::array<double, 7> mean;
    mean.fill(0.0);

    tf2::Vector3 prev_axis = {0, 0, 0};
    for (const auto & pair : poses_) {
      const tf2::Transform & pose = pair.second;
      const tf2::Vector3 & translate = pose.getOrigin();
      tf2::Quaternion q = pose.getRotation();
      q.normalize();

      if (prev_axis.dot(q.getAxis()) < 0) {
        q *= -1;
      }
      prev_axis = q.getAxis();

      double vect[7] = {
        translate[0], translate[1], translate[2], q.x(), q.y(), q.z(), q.w()
      };

      for (int i = 0; i < 7; i++) {
        mean[i] += vect[i];
        for (int j = 0; j < 7; j++) {
          pose_quat_covariance[i * 7 + j] += vect[i] * vect[j];
        }
      }
    }

    for (int i = 0; i < 7; i++) {
      mean[i] /= poses_.size();
    }

    for (int i = 0; i < 7; i++) {
      for (int j = 0; j < 7; j++) {
        pose_quat_covariance[i * 7 + j] = pose_quat_covariance[i * 7 + j] / poses_.size() -
          mean[i] * mean[j];
      }
    }

    mean_quat = tf2::Quaternion(mean[3], mean[4], mean[5], mean[6]);
    mean_quat.normalize();
  }

  QuaternionCovToRollPitchYawCov(pose_quat_covariance, mean_quat, cov);
  return true;
}

void VelocityCache::Reset()
{
  velocities_.clear();
}

void VelocityCache::Add(
  const double & x, const double & y, const double & z, const double & roll,
  const double & pitch, const double & yaw)
{
  velocities_.push_back(
        {
          x, y, z,
          roll, pitch, yaw
        });

  while (velocities_.size() > num_velocities_to_keep_) {
    velocities_.pop_front();
  }
}

bool VelocityCache::GetCovariance(std::array<double, 6 * 6> & cov) const
{
  if (velocities_.size() < 10) {
    return false;
  }

  std::array<double, 7> mean;
  mean.fill(0.0);

  for (const auto & vect : velocities_) {
    for (int i = 0; i < 6; i++) {
      mean[i] += vect[i];
      for (int j = 0; j < 6; j++) {
        cov[i * 6 + j] += vect[i] * vect[j];
      }
    }
  }

  for (int i = 0; i < 6; i++) {
    mean[i] /= velocities_.size();
  }

  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      cov[i * 6 + j] = cov[i * 6 + j] / velocities_.size() - mean[i] * mean[j];
    }
  }

  return true;
}

}  //  namespace visual_slam
}  //  namespace isaac_ros
}  //  namespace nvidia
