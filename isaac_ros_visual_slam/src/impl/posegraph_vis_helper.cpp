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

#include <map>
#include <memory>
#include <string>
#include <utility>

#include "isaac_ros_visual_slam/impl/cuvslam_ros_conversion.hpp"
#include "isaac_ros_visual_slam/impl/has_subscribers.hpp"
#include "isaac_ros_visual_slam/impl/posegraph_vis_helper.hpp"
#include "isaac_ros_visual_slam/impl/types.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace visual_slam
{

// PoseGraphVisHelper
PoseGraphVisHelper::PoseGraphVisHelper(
  cuvslam::Slam::DataLayer layer,
  uint32_t max_items_count,
  uint32_t period_ms)
: layer_(layer), max_items_count_(max_items_count), period_ms_(period_ms)
{
}

PoseGraphVisHelper::~PoseGraphVisHelper()
{
  Exit();
}

void PoseGraphVisHelper::Init(
  const rclcpp::Publisher<PoseArrayType>::SharedPtr publisher_nodes,
  const rclcpp::Publisher<MarkerType>::SharedPtr publisher_edges,
  const rclcpp::Publisher<MarkerType>::SharedPtr publisher_edges2,
  std::shared_ptr<cuvslam::Slam> & cuvslam_slam,
  const tf2::Transform & canonical_pose_cuvslam,
  const std::string & frame_id,
  const rclcpp::Logger & logger)
{
  publisher_nodes_ = publisher_nodes;
  publisher_edges_ = publisher_edges;
  publisher_edges2_ = publisher_edges2;

  VisHelper::Init(cuvslam_slam, canonical_pose_cuvslam, frame_id, logger);
  thread_ = std::thread{&PoseGraphVisHelper::Run, this};
}

void PoseGraphVisHelper::Reset()
{
  if (cuvslam_slam_) {
    cuvslam_slam_->DisableReadingData(layer_);
  }
  publisher_nodes_.reset();
  publisher_edges_.reset();
  publisher_edges2_.reset();
}

void PoseGraphVisHelper::Run()
{
  std::unique_lock<std::mutex> locker(mutex_);
  // cuvslam_slam_ will be null after Exit() was called
  while (cuvslam_slam_) {
    try {
      cond_var_.wait_for(
      locker,
      std::chrono::milliseconds(period_ms_));
      if (!cuvslam_slam_) {break;}
      if (!rclcpp::ok()) {break;}

      bool has_subnumber_nodes = HasSubscribers(publisher_nodes_);
      bool has_subnumber_edges = HasSubscribers(publisher_edges_);
      bool has_subnumber_edges2 = HasSubscribers(publisher_edges2_);
      if (!has_subnumber_nodes && !has_subnumber_edges && !has_subnumber_edges2) {
      // no subscribers so disable reading
        cuvslam_slam_->DisableReadingData(layer_);
        continue;
      }

      // enable reading
      cuvslam_slam_->EnableReadingData(layer_, max_items_count_);
      // read data
      std::shared_ptr<const cuvslam::Slam::PoseGraph> pose_graph =
        cuvslam_slam_->ReadPoseGraph();
      if (last_timestamp_ns_ == pose_graph->timestamp_ns) {
        // Don't need to publish now
        continue;
      }
      last_timestamp_ns_ = pose_graph->timestamp_ns;
      rclcpp::Time stamp(last_timestamp_ns_, RCL_SYSTEM_TIME);

      if (has_subnumber_nodes) {
        // Publish nodes
        nodes_msg_t msg = std::make_unique<PoseArrayType>();

        msg->header.frame_id = frame_id_;
        msg->header.stamp = stamp;
        msg->poses.resize(pose_graph->nodes.size());

        for (uint32_t i = 0; i < pose_graph->nodes.size(); i++) {
          const cuvslam::Pose & cuvslam_pose = pose_graph->nodes[i].node_pose;

          // Change of basis vectors for pose
          const tf2::Transform ros_pos{ChangeBasis(
            canonical_pose_cuvslam_,
            FromcuVSLAMPose(cuvslam_pose))};

          PoseType dst;
          tf2::toMsg(ros_pos, dst);
          msg->poses[i] = dst;
        }
      // publishing
        publisher_nodes_->publish(std::move(msg));
      }

    // build nodes_index if required
      std::map<uint64_t, int> nodes_index;
      if (has_subnumber_edges || has_subnumber_edges2) {
        for (uint32_t i = 0; i < pose_graph->nodes.size(); i++) {
          const cuvslam::Slam::PoseGraphNode & node = pose_graph->nodes[i];
          nodes_index[node.id] = i;
        }
      }

      if (has_subnumber_edges) {
        MarkerType marker_edges;
        marker_edges.header.frame_id = frame_id_;
        marker_edges.header.stamp = stamp;
        marker_edges.ns = "edges";
        marker_edges.id = 0;
        marker_edges.action = MarkerType::ADD;
        marker_edges.type = MarkerType::LINE_LIST;
        marker_edges.pose.position.x = 0;
        marker_edges.pose.position.y = 0;
        marker_edges.pose.position.z = 0;
        marker_edges.pose.orientation.x = 0.0;
        marker_edges.pose.orientation.y = 0.0;
        marker_edges.pose.orientation.z = 0.0;
        marker_edges.pose.orientation.w = 1.0;
        marker_edges.scale.x = 0.0033;
        marker_edges.scale.y = 0.01;
        marker_edges.scale.z = 0.01;
        marker_edges.color.a = 0.25;
        marker_edges.color.r = 1.0;
        marker_edges.color.g = 1.0;
        marker_edges.color.b = 1.0;
        marker_edges.points.resize(pose_graph->edges.size() * 2);

        for (uint32_t i = 0; i < pose_graph->edges.size(); i++) {
          const cuvslam::Slam::PoseGraphEdge & edge = pose_graph->edges[i];
          auto it_from = nodes_index.find(edge.node_from);
          auto it_to = nodes_index.find(edge.node_to);
          if (it_from == nodes_index.end() || it_to == nodes_index.end() ) {
            continue;
          }

          const cuvslam::Pose & node_from = pose_graph->nodes[it_from->second].node_pose;
          const cuvslam::Pose & node_to = pose_graph->nodes[it_to->second].node_pose;
          const tf2::Transform ros_from{ChangeBasis(
            canonical_pose_cuvslam_,
            FromcuVSLAMPose(node_from))};
          const tf2::Transform ros_to{ChangeBasis(canonical_pose_cuvslam_,
                    FromcuVSLAMPose(node_to))};
          tf2::Vector3 zero(0, 0, 0);
          tf2::Vector3 p1 = ros_from * zero;
          tf2::Vector3 p2 = ros_to * zero;

          geometry_msgs::msg::Point pp1;
          pp1.x = p1[0];
          pp1.y = p1[1];
          pp1.z = p1[2];
          geometry_msgs::msg::Point pp2;
          pp2.x = p2[0];
          pp2.y = p2[1];
          pp2.z = p2[2];

          marker_edges.points[i * 2 + 0] = pp1;
          marker_edges.points[i * 2 + 1] = pp2;
        }
        // publishing
        publisher_edges_->publish(std::move(marker_edges));
      }

      if (has_subnumber_edges2) {
        MarkerType marker_edges_transform;
        marker_edges_transform.header.frame_id = frame_id_;
        marker_edges_transform.header.stamp = stamp;
        marker_edges_transform.ns = "edges";
        marker_edges_transform.id = 1;
        marker_edges_transform.action = MarkerType::ADD;
        marker_edges_transform.type = MarkerType::LINE_LIST;
        marker_edges_transform.pose.position.x = 0;
        marker_edges_transform.pose.position.y = 0;
        marker_edges_transform.pose.position.z = 0;
        marker_edges_transform.pose.orientation.x = 0.0;
        marker_edges_transform.pose.orientation.y = 0.0;
        marker_edges_transform.pose.orientation.z = 0.0;
        marker_edges_transform.pose.orientation.w = 1.0;
        marker_edges_transform.scale.x = 0.0033;
        marker_edges_transform.scale.y = 0.01;
        marker_edges_transform.scale.z = 0.01;
        marker_edges_transform.color.a = 0.25;
        marker_edges_transform.color.r = 1.0;
        marker_edges_transform.color.g = 0.3;
        marker_edges_transform.color.b = 0.2;
        marker_edges_transform.points.resize(pose_graph->edges.size() * 2);

        for (uint32_t i = 0; i < pose_graph->edges.size(); i++) {
          const cuvslam::Slam::PoseGraphEdge & edge = pose_graph->edges[i];
          auto it_from = nodes_index.find(edge.node_from);
          if (it_from == nodes_index.end()) {
            continue;
          }

          const cuvslam::Pose & node_from = pose_graph->nodes[it_from->second].node_pose;
          const tf2::Transform ros_from{ChangeBasis(
            canonical_pose_cuvslam_,
            FromcuVSLAMPose(node_from))};
          tf2::Vector3 zero(0, 0, 0);
          tf2::Vector3 p1 = ros_from * zero;

          geometry_msgs::msg::Point pp1;
          pp1.x = p1[0];
          pp1.y = p1[1];
          pp1.z = p1[2];

          const tf2::Transform ros_transform{ChangeBasis(
            canonical_pose_cuvslam_,
            FromcuVSLAMPose(edge.transform))};
          tf2::Vector3 p3 = ros_transform * zero;
          p3 = ros_from * p3;
          geometry_msgs::msg::Point pp3;
          pp3.x = p3[0];
          pp3.y = p3[1];
          pp3.z = p3[2];

          marker_edges_transform.points[i * 2 + 0] = pp1;
          marker_edges_transform.points[i * 2 + 1] = pp3;
        }
        // publishing
        publisher_edges2_->publish(std::move(marker_edges_transform));
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(logger_, "PoseGraphVisHelper has failed to run: %s", e.what());
    }
  }
}

}  // namespace visual_slam
}  // namespace isaac_ros
}  // namespace nvidia
