// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <gtest/gtest.h>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include "isaac_ros_visual_slam/impl/message_stream_synchronizer.hpp"

namespace nvidia::isaac_ros::visual_slam
{

geometry_msgs::msg::PointStamped CreateMessage(int time_ns)
{
  geometry_msgs::msg::PointStamped message;
  message.header.stamp = rclcpp::Time(time_ns);
  return message;
}

template<typename Message>
void ExpectTimestamps(
  const std::vector<std::pair<int, Message>> & messages,
  const std::vector<int> & indices,
  const std::vector<int> & timestamps
)
{
  ASSERT_EQ(timestamps.size(), indices.size());
  ASSERT_EQ(timestamps.size(), messages.size());

  for (size_t i = 0; i < timestamps.size(); ++i) {
    const int msg_idx = messages[i].first;
    const int msg_stamp = rclcpp::Time(messages[i].second.header.stamp).nanoseconds();
    EXPECT_EQ(indices[i], msg_idx) << "Message " << i << " has incorrect index.";
    EXPECT_EQ(timestamps[i], msg_stamp) << "Message " << i << "i has incorrect stamp.";
  }
}

TEST(MessageStreamSynchronizerTests, EverythingTest)
{
  constexpr int kNumTopics = 2;
  constexpr int kBufferSize = 10;
  constexpr double kTimestepDeltaThresholdNs = 4;
  MessageStreamSynchronizer<geometry_msgs::msg::PointStamped> sync(kNumTopics,
    kTimestepDeltaThresholdNs, kNumTopics, kBufferSize);

  // Use a lambda to store the messages from the callback.
  std::vector<std::pair<int, geometry_msgs::msg::PointStamped>> synced_msgs;
  sync.RegisterCallback(
    [&synced_msgs](int64_t /*timestamp_ns*/, const auto & callback_msgs) {
      synced_msgs = callback_msgs;
    });

  // Check that initially everything is empty.
  EXPECT_EQ(synced_msgs.size(), 0);

  // Now we simulate incoming messages. We assume the synced_msgs arrive every 5ns,

  // Warmup period, we fill every buffer.
  sync.AddMessage(0, 0, CreateMessage(0));
  sync.AddMessage(1, 0, CreateMessage(0));
  ExpectTimestamps(synced_msgs, {0, 1}, {0, 0});
  synced_msgs.clear();

  // No data drops, but timestamps have offsets: we expect a match.
  sync.AddMessage(0, 10, CreateMessage(10));
  sync.AddMessage(1, 12, CreateMessage(12));
  ExpectTimestamps(synced_msgs, {0, 1}, {10, 12});
  synced_msgs.clear();

  // Heavily delayed data, we dont expect a match yet.
  // Data from camera 0 is delayed.
  sync.AddMessage(1, 20, CreateMessage(20));
  ExpectTimestamps(synced_msgs, {}, {});

  // Now the delayed data from before arrives. We expect 2 matches.
  sync.AddMessage(1, 30, CreateMessage(30));
  // This is the delayed data from camera 0:
  sync.AddMessage(0, 20, CreateMessage(20));
  ExpectTimestamps(synced_msgs, {0, 1}, {20, 20});
  synced_msgs.clear();

  sync.AddMessage(0, 30, CreateMessage(30));
  ExpectTimestamps(synced_msgs, {0, 1}, {30, 30});
  synced_msgs.clear();

  // This time camera 1 has drops. We expect no matches.
  sync.AddMessage(0, 40, CreateMessage(40));
  ExpectTimestamps(synced_msgs, {}, {});
  synced_msgs.clear();

  // We have data from everything. We expect the algorithm to recover and have
  // a match.
  sync.AddMessage(0, 50, CreateMessage(50));
  sync.AddMessage(1, 50, CreateMessage(50));
  ExpectTimestamps(synced_msgs, {0, 1}, {50, 50});
  synced_msgs.clear();
}

}  // namespace nvidia::isaac_ros::visual_slam
