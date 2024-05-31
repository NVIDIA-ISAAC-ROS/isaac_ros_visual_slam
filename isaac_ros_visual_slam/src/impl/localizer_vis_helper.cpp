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
#include <string>
#include <vector>

#include "isaac_ros_visual_slam/impl/cuvslam_ros_conversion.hpp"
#include "isaac_ros_visual_slam/impl/has_subscribers.hpp"
#include "isaac_ros_visual_slam/impl/localizer_vis_helper.hpp"
#include "isaac_ros_visual_slam/impl/types.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace visual_slam
{

// LocalizerVisHelper

LocalizerVisHelper::LocalizerVisHelper(
  uint32_t max_items_count,
  uint32_t period_ms)
: max_items_count_(max_items_count), period_ms_(period_ms)
{
}

LocalizerVisHelper::~LocalizerVisHelper()
{
  Exit();
}

void LocalizerVisHelper::Init(
  rclcpp::Publisher<MarkerArrayType>::SharedPtr publisher_localizer_probes,
  CUVSLAM_TrackerHandle cuvslam_handle,
  const tf2::Transform & canonical_pose_cuvslam,
  const std::string & frame_id)
{
  publisher_localizer_probes_ = publisher_localizer_probes;

  VisHelper::Init(cuvslam_handle, canonical_pose_cuvslam, frame_id);
  thread_ = std::thread{&LocalizerVisHelper::Run, this};
}

void LocalizerVisHelper::Reset()
{
  CUVSLAM_DisableReadingDataLayer(cuvslam_handle_, LL_LOCALIZER_PROBES);
  publisher_localizer_probes_.reset();
  localization_finished_ = false;
  have_result_pose_ = false;
  last_timestamp_ns_ = 0;
  last_num_probes_ = 0;
  reset_required_ = true;
}
void LocalizerVisHelper::SetResult(bool succesed, const tf2::Transform & pose)
{
  localization_finished_ = true;
  have_result_pose_ = succesed;
  result_pose_ = pose;
  // force update
  last_timestamp_ns_ = 0;
}

void LocalizerVisHelper::Run()
{
  std::unique_lock<std::mutex> locker(mutex_);
  // cuvslam_handle_ will be null after Exit() was called
  while (cuvslam_handle_) {
    if (reset_required_) {
      // reset all data
      MarkerArrayType markers;
      markers.markers.resize(1);
      MarkerType & marker_probes = markers.markers[0];
      marker_probes.action = MarkerType::DELETEALL;
      publisher_localizer_probes_->publish(markers);
      reset_required_ = false;
    }
    cond_var_.wait_for(
      locker,
      std::chrono::milliseconds(period_ms_));
    if (!cuvslam_handle_) {break;}
    if (!rclcpp::ok()) {break;}
    if (!HasSubscribers(publisher_localizer_probes_)) {
      // no subcribers so disable reading
      CUVSLAM_DisableReadingDataLayer(cuvslam_handle_, LL_LOCALIZER_PROBES);
      continue;
    }
    // enable reading
    if (CUVSLAM_EnableReadingDataLayer(
        cuvslam_handle_, LL_LOCALIZER_PROBES, max_items_count_) != CUVSLAM_SUCCESS)
    {
      continue;
    }
    // read data
    CUVSLAM_LocalizerProbesRef localizer_probes;
    if (CUVSLAM_StartReadingLocalizerProbes(
        cuvslam_handle_, &localizer_probes) != CUVSLAM_SUCCESS)
    {
      continue;
    }
    if (last_timestamp_ns_ == localizer_probes.timestamp_ns &&
      last_num_probes_ == localizer_probes.num_probes)
    {
      // don't need to publish now
      CUVSLAM_FinishReadingLocalizerProbes(cuvslam_handle_);
      continue;
    }

    last_timestamp_ns_ = localizer_probes.timestamp_ns;
    last_num_probes_ = localizer_probes.num_probes;
    rclcpp::Time stamp(localizer_probes.timestamp_ns, RCL_SYSTEM_TIME);

    const float scale = localizer_probes.size;
    const tf2::Vector3 zero(0, 0, 0);
    const tf2::Vector3 one(scale, 0, 0);
    const tf2::Vector3 color_max(1, 1, 1);
    tf2::Vector3 color_in_progress(0.10, 0.15, 1);
    tf2::Vector3 color_successful(0.15, 0.5, 0.10);
    tf2::Vector3 color_failed(0.5, 0.1, 0.15);

    tf2::Vector3 color_min = color_in_progress;
    if (localization_finished_ && have_result_pose_) {
      color_min = color_successful;
    }
    if (localization_finished_ && !have_result_pose_) {
      color_min = color_failed;
    }

    if (true) {
      MarkerArrayType markers;
      markers.markers.resize(2);
      {
        MarkerType & marker_probes = markers.markers[0];
        marker_probes.header.frame_id = frame_id_;
        marker_probes.header.stamp = stamp;
        marker_probes.ns = "probes";
        marker_probes.id = 0;
        marker_probes.action = MarkerType::ADD;
        marker_probes.type = MarkerType::LINE_LIST;
        marker_probes.pose.position.x = 0;
        marker_probes.pose.position.y = 0;
        marker_probes.pose.position.z = 0;
        marker_probes.pose.orientation.x = 0.0;
        marker_probes.pose.orientation.y = 0.0;
        marker_probes.pose.orientation.z = 0.0;
        marker_probes.pose.orientation.w = 1.0;
        marker_probes.scale.x = 0.003;
        marker_probes.scale.y = 0.003;
        marker_probes.scale.z = 0.003;
        marker_probes.color.a = 0.25;
        marker_probes.color.r = 1.0;
        marker_probes.color.g = 1.0;
        marker_probes.color.b = 1.0;
        marker_probes.points.resize(localizer_probes.num_probes * 2);
        marker_probes.colors.resize(localizer_probes.num_probes * 2);
        if (localizer_probes.num_probes == 0) {
          marker_probes.action = MarkerType::DELETE;
        }
        for (uint32_t i = 0; i < localizer_probes.num_probes; i++) {
          const CUVSLAM_LocalizerProbe & probe = localizer_probes.probes[i];

          tf2::Transform guess_pose_cuvslam = FromcuVSLAMPose(probe.guess_pose);
          tf2::Vector3 origin = guess_pose_cuvslam * tf2::Vector3(
            0, 0, -localizer_probes.size * 0.05);
          guess_pose_cuvslam.setOrigin(origin);
          const tf2::Transform guess_pose__ros{ChangeBasis(
              canonical_pose_cuvslam_,
              guess_pose_cuvslam)};

          tf2::Vector3 p1 = guess_pose__ros * zero;
          tf2::Vector3 p2 = guess_pose__ros * (one * 0.4);

          geometry_msgs::msg::Point pp1;
          pp1.x = p1[0]; pp1.y = p1[1]; pp1.z = p1[2];
          geometry_msgs::msg::Point pp2;
          pp2.x = p2[0]; pp2.y = p2[1]; pp2.z = p2[2];

          marker_probes.points[i * 2 + 0] = pp1;
          marker_probes.points[i * 2 + 1] = pp2;

          float w = std::max(std::min(probe.weight, 1.f), 0.25f);
          tf2::Vector3 c = color_max * w + color_min * (1 - w);
          std_msgs::msg::ColorRGBA color;
          color.r = c[0]; color.g = c[1]; color.b = c[2];
          color.a = 1;

          marker_probes.colors[i * 2 + 0] = color;
          marker_probes.colors[i * 2 + 1] = color;
        }
      }
      {
        MarkerType & marker_probes = markers.markers[1];
        marker_probes.header.frame_id = frame_id_;
        marker_probes.header.stamp = stamp;
        marker_probes.ns = "result";
        marker_probes.id = 0;
        marker_probes.action = MarkerType::ADD;
        marker_probes.type = MarkerType::LINE_STRIP;
        marker_probes.pose.position.x = 0;
        marker_probes.pose.position.y = 0;
        marker_probes.pose.position.z = 0;
        marker_probes.pose.orientation.x = 0.0;
        marker_probes.pose.orientation.y = 0.0;
        marker_probes.pose.orientation.z = 0.0;
        marker_probes.pose.orientation.w = 1.0;
        marker_probes.scale.x = 0.1;
        marker_probes.scale.y = 0.1;
        marker_probes.scale.z = 0.1;
        marker_probes.color.a = 0.25;
        marker_probes.color.r = 1.0;
        marker_probes.color.g = 1.0;
        marker_probes.color.b = 1.0;
        std::vector<tf2::Vector3> line_strip;
        std::vector<tf2::Vector3> line_colors;
        if (have_result_pose_) {
          line_strip.push_back(result_pose_ * zero);
          line_strip.push_back(result_pose_ * tf2::Vector3(scale, 0, 0));
          line_colors.push_back(tf2::Vector3(1, 0, 0));
          line_colors.push_back(tf2::Vector3(1, 0, 0));

          line_strip.push_back(result_pose_ * zero);
          line_strip.push_back(result_pose_ * tf2::Vector3(0, scale, 0));
          line_colors.push_back(tf2::Vector3(0, 1, 0));
          line_colors.push_back(tf2::Vector3(0, 1, 0));

          line_strip.push_back(result_pose_ * zero);
          line_strip.push_back(result_pose_ * tf2::Vector3(0, 0, scale));
          line_colors.push_back(tf2::Vector3(0, 0, 1));
          line_colors.push_back(tf2::Vector3(0, 0, 1));
        } else {
          marker_probes.action = MarkerType::DELETE;
        }
        marker_probes.points.resize(line_strip.size());
        marker_probes.colors.resize(line_strip.size());

        for (size_t i = 0; i < line_strip.size(); i++) {
          auto & p = line_strip[i];
          geometry_msgs::msg::Point pp;
          pp.x = p[0]; pp.y = p[1]; pp.z = p[2];
          marker_probes.points[i] = pp;

          auto & c = line_colors[i];
          std_msgs::msg::ColorRGBA color;
          color.r = c[0]; color.g = c[1]; color.b = c[2];
          color.a = 1;
          marker_probes.colors[i] = color;
        }
      }
      publisher_localizer_probes_->publish(markers);
    } else {
    }

    CUVSLAM_FinishReadingLocalizerProbes(cuvslam_handle_);
  }
}

}  // namespace visual_slam
}  // namespace isaac_ros
}  // namespace nvidia
