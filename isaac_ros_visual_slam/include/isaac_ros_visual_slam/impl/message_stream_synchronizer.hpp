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

#ifndef ISAAC_ROS_VISUAL_SLAM__IMPL__MESSAGE_STREAM_SYNCHRONIZER_HPP_
#define ISAAC_ROS_VISUAL_SLAM__IMPL__MESSAGE_STREAM_SYNCHRONIZER_HPP_

#include <algorithm>
#include <cstdlib>
#include <functional>
#include <utility>
#include <vector>

#include "isaac_ros_visual_slam/impl/message_buffer.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace visual_slam
{

// This class is designed synchronize n topics, where n can be set dynamically
// at runtime. This is different to the synchronizers from ROS' message_filters
// where n has to be set at compile time. All topics have to use the same
// message type. If some topics have message drops the synchronizer can also
// return the subset of matching messages.
template<typename Message>
class MessageStreamSynchronizer
{
public:
  using CallbackFunction =
    std::function<void (int64_t, const std::vector<std::pair<int, Message>> &)>;

  explicit MessageStreamSynchronizer(
    int num_topics,
    int timestamp_delta_threshold_ns,
    int min_num_messages, int buffer_size);

  // Functions to add new incomming messages. Should be called from the
  // subscriber's callback.
  void AddMessage(
    int idx, int64_t timestamp, const Message & message,
    bool trigger_callback = true);

  Message PeekNextMessage(int idx) const;

  // Update all buffers by popping all messages that can no longer be a match.
  // And if we have a match call trigger the callback function.
  void PopBuffersAndTriggerCallback();

  // Callback that is called whenever we have a new matching set of messages.
  void RegisterCallback(CallbackFunction callback);

private:
  // Get the timestamps of the earliest message of every message buffer.
  std::vector<int64_t> PeekMessageTimestamps() const;
  void TriggerCallbackWithMessages(const std::vector<int> & buffer_indices);

  // Threshold used to consider if timestamps of multiple messages are matching.
  const int timestamp_delta_threshold_ns_;
  // Minimum number of messages used to trigger the callback.
  size_t min_num_messages_;

  std::vector<MessageBuffer<Message>> message_buffers_;
  CallbackFunction callback_ = [](int64_t, const std::vector<std::pair<int, Message>> &) {};
};

namespace
{
std::vector<int> GetIndicesOfMessagesBeforeThreshold(
  const std::vector<int64_t> & values, int64_t threshold)
{
  if (values.empty()) {return {};}

  const int64_t min_value = *std::min_element(values.begin(), values.end());

  std::vector<int> indices;
  indices.reserve(values.size());

  for (size_t i = 0; i < values.size(); ++i) {
    if (values[i] <= min_value + threshold) {indices.push_back(i);}
  }
  return indices;
}

}  // namespace

template<typename Message>
MessageStreamSynchronizer<Message>::MessageStreamSynchronizer(
  int num_topics, int timestamp_delta_threshold_ns, int min_num_messages,
  int buffer_size)
: timestamp_delta_threshold_ns_(timestamp_delta_threshold_ns),
  min_num_messages_(min_num_messages),
  message_buffers_(num_topics, MessageBuffer<Message>(buffer_size)) {}

template<typename Message>
void MessageStreamSynchronizer<Message>::AddMessage(
  int idx, int64_t timestamp,
  const Message & message, bool trigger_callback)
{
  message_buffers_[idx].Push(timestamp, message);
  if (trigger_callback) {
    PopBuffersAndTriggerCallback();
  }
}

template<typename Message>
auto MessageStreamSynchronizer<Message>::PeekNextMessage(int idx) const
-> Message
{
  return message_buffers_[idx].Peek();
}

template<typename Message>
std::vector<int64_t>
MessageStreamSynchronizer<Message>::PeekMessageTimestamps() const
{
  std::vector<int64_t> timestamps;
  timestamps.reserve(message_buffers_.size());
  for (const auto & buffer : message_buffers_) {
    if (buffer.IsEmpty()) {
      return {};
    }
    timestamps.push_back(buffer.GetNextTimeStamp());
  }
  return timestamps;
}

template<typename Message>
void MessageStreamSynchronizer<Message>::TriggerCallbackWithMessages(
  const std::vector<int> & indices)
{
  int64_t max_timestamp = -1;
  std::vector<Message> messages;
  std::vector<std::pair<int, Message>> idx_and_images;
  idx_and_images.reserve(indices.size());

  for (auto & idx : indices) {
    const int64_t timestamp = message_buffers_[idx].GetNextTimeStamp();
    max_timestamp = std::max(timestamp, max_timestamp);
    idx_and_images.push_back({idx, message_buffers_[idx].Peek()});
  }

  callback_(max_timestamp, idx_and_images);
}

template<typename Message>
void MessageStreamSynchronizer<Message>::PopBuffersAndTriggerCallback()
{
  // Strategy: We look at the front message of every buffer and use all messages that are within the
  // timestamp threshold.
  while (true) {
    const std::vector<int64_t> timestamps = PeekMessageTimestamps();
    // If empty we don't yet have enough values.
    if (timestamps.empty()) {return;}

    const std::vector<int> front_match_indices =
      GetIndicesOfMessagesBeforeThreshold(timestamps, timestamp_delta_threshold_ns_);

    if (front_match_indices.size() >= min_num_messages_) {
      TriggerCallbackWithMessages(front_match_indices);
    }

    for (const int idx : front_match_indices) {
      message_buffers_[idx].Pop();
    }
  }
}

template<typename Message>
void MessageStreamSynchronizer<Message>::RegisterCallback(
  CallbackFunction callback)
{
  callback_ = std::move(callback);
}

}  // namespace visual_slam
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_VISUAL_SLAM__IMPL__MESSAGE_STREAM_SYNCHRONIZER_HPP_
