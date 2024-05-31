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

#ifndef ISAAC_ROS_VISUAL_SLAM__IMPL__LOCALIZER_VIS_HELPER_HPP_
#define ISAAC_ROS_VISUAL_SLAM__IMPL__LOCALIZER_VIS_HELPER_HPP_

#include <string>

#include "isaac_ros_visual_slam/impl/types.hpp"
#include "viz_helper.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace visual_slam
{

// visualize Localizer state
struct LocalizerVisHelper : public VisHelper
{
public:
  LocalizerVisHelper(
    uint32_t max_items_count = 1024,
    uint32_t period_ms = 500
  );
  ~LocalizerVisHelper() override;

  void Init(
    rclcpp::Publisher<MarkerArrayType>::SharedPtr publisher_localizer_probes,
    CUVSLAM_TrackerHandle cuvslam_handle,
    const tf2::Transform & canonical_pose_cuvslam,
    const std::string & frame_id);

  void Reset() override;

  void Run();

  void SetResult(bool succesed, const tf2::Transform & pose);

protected:
  uint32_t max_items_count_ = 1024;
  uint32_t period_ms_ = 500;

  uint64_t last_timestamp_ns_ = 0;
  uint64_t last_num_probes_ = 0;
  bool reset_required_ = false;

  bool localization_finished_ = false;
  bool have_result_pose_ = false;
  tf2::Transform result_pose_;

  rclcpp::Publisher<MarkerArrayType>::SharedPtr publisher_localizer_probes_;
};

}  // namespace visual_slam
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_VISUAL_SLAM__IMPL__LOCALIZER_VIS_HELPER_HPP_
