// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2023-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#ifndef ISAAC_ROS_VISUAL_SLAM__IMPL__MESSAGE_STREAM_SEQUENCER_HPP_
#define  ISAAC_ROS_VISUAL_SLAM__IMPL__MESSAGE_STREAM_SEQUENCER_HPP_

#include <cstdlib>
#include <functional>
#include <utility>
#include <vector>
#include "message_buffer.hpp"

namespace nvidia
{
namespace isaac_ros
{
namespace visual_slam
{

// This class is designed to generate an interleaved sequence of messages from two separate streams.
// Streams are named stream1 and stream2 and assumption is stream1 is faster than stream2.

template<class T1, class T2>
class MessageStreamSequencer
{
public:
  using Callback = std::function<void (const std::vector<T1> &, const T2 &)>;

  explicit MessageStreamSequencer(
    uint8_t size_stream1, int64_t eps_stream1, uint8_t size_stream2, int64_t eps_stream2);
  void CallbackStream1(const int64_t & timestamp, const T1 & msg);
  void CallbackStream2(const int64_t & timestamp, const T2 & msg);
  int64_t GetNextTimeStampStream1();
  int64_t GetNextTimeStampStream2();
  size_t GetSizeStream1();
  size_t GetSizeStream2();
  void RegisterCallback(Callback callback);

private:
  // Buffer
  MessageBuffer<T1> stream1_buff_;
  int64_t epsilon_stream1_;
  MessageBuffer<T2> stream2_buff_;
  int64_t epsilon_stream2_;
  Callback registered_callback_ = [](const std::vector<T1> &, const T2 &) {};
};

template<class T1, class T2>
MessageStreamSequencer<T1, T2>::MessageStreamSequencer(
  uint8_t size_stream1, int64_t eps_stream1,
  uint8_t size_stream2, int64_t eps_stream2)
: stream1_buff_{size_stream1}, epsilon_stream1_{eps_stream1},
  stream2_buff_{size_stream2}, epsilon_stream2_{eps_stream2}
{
}

template<class T1, class T2>
void MessageStreamSequencer<T1, T2>::CallbackStream1(const int64_t & timestamp, const T1 & msg)
{
  stream1_buff_.Push(timestamp, msg);
  std::pair<std::vector<T1>, T2> result;
  auto curr_msg_ts = stream1_buff_.GetCurrentTimeStamp();
  auto last_msg_ts = stream1_buff_.GetLastTimeStamp();
  auto next_msg_ts = stream1_buff_.GetNextTimeStamp();
  auto stream2_curr_ts = stream2_buff_.GetCurrentTimeStamp();
  auto stream2_next_ts = stream2_buff_.GetNextTimeStamp();
  auto stream2_last_ts = stream2_buff_.GetLastTimeStamp();

  // Ignoring late stream1 msg
  if (curr_msg_ts < stream2_last_ts) {
    stream1_buff_.Pop();
    return;
  }

  // Timeouts w.r.t self and other stream
  bool self_esp_timeout = std::abs(curr_msg_ts - last_msg_ts) > epsilon_stream1_;
  bool other_esp_timeout = std::abs(curr_msg_ts - stream2_curr_ts) > epsilon_stream2_;

  // Adding msgs till stream1 catches up with stream2
  if (curr_msg_ts < stream2_curr_ts && curr_msg_ts < stream2_next_ts &&
    (!other_esp_timeout && !self_esp_timeout))
  {
    return;
  } else {
    // Either timeout or stream1 has caught up and
    // all the msgs needed by stream2 for the current timestamp are present

    // Process all the stored msgs from stream2 till current time of stream1
    while ((curr_msg_ts >= stream2_next_ts || other_esp_timeout || self_esp_timeout) &&
      !stream2_buff_.IsEmpty())
    {
      // Process all the stored msgs from stream1 till current time of stream1
      while (!stream1_buff_.IsEmpty() && stream2_next_ts >= next_msg_ts) {
        result.first.emplace_back(stream1_buff_.Pop());
        next_msg_ts = stream1_buff_.GetNextTimeStamp();
      }
      result.second = stream2_buff_.Pop();
      registered_callback_(result.first, result.second);
      // Clearing stream1 msgs for the subsequent runs
      result.first.clear();
      stream2_next_ts = stream2_buff_.GetNextTimeStamp();
    }
  }
}

template<class T1, class T2>
void MessageStreamSequencer<T1, T2>::CallbackStream2(const int64_t & timestamp, const T2 & msg)
{
  stream2_buff_.Push(timestamp, msg);

  std::pair<std::vector<T1>, T2> result;
  auto curr_msg_ts = stream2_buff_.GetCurrentTimeStamp();
  auto last_msg_ts = stream2_buff_.GetLastTimeStamp();
  auto stream1_curr_ts = stream1_buff_.GetCurrentTimeStamp();
  auto stream1_next_ts = stream1_buff_.GetNextTimeStamp();

  // Ignoring late stream2 msg
  if (curr_msg_ts < stream1_buff_.GetLastTimeStamp()) {
    stream2_buff_.Pop();
    return;
  }

  // Timeouts w.r.t self and other stream
  bool self_timeout = std::abs(curr_msg_ts - last_msg_ts) > epsilon_stream2_;
  bool other_timeout = std::abs(curr_msg_ts - stream1_curr_ts) > epsilon_stream1_;

  // Adding msg and wait for other stream to catch-up
  if (curr_msg_ts > stream1_curr_ts && (!other_timeout && !self_timeout)) {
    return;
  } else {
    // Either timeout or stream1 has caught up and
    // all the msgs needed by stream2 for the current timestamp are present

    // Process all the stored msgs from stream2 till current time of stream1
    while (((curr_msg_ts >= stream1_next_ts && curr_msg_ts <= stream1_curr_ts) ||
      self_timeout || other_timeout) && !stream2_buff_.IsEmpty())
    {
      // Process all the stored msgs from stream1 till current time of stream1
      while (curr_msg_ts >= stream1_next_ts && !stream1_buff_.IsEmpty()) {
        result.first.emplace_back(stream1_buff_.Pop());
        stream1_next_ts = stream1_buff_.GetNextTimeStamp();
      }
      result.second = stream2_buff_.Pop();
      registered_callback_(result.first, result.second);
      // Clearing stream1 msgs for the subsequent runs
      result.first.clear();
    }
  }
}

template<class T1, class T2>
int64_t MessageStreamSequencer<T1, T2>::GetNextTimeStampStream1()
{
  return stream1_buff_.GetNextTimeStamp();
}

template<class T1, class T2>
int64_t MessageStreamSequencer<T1, T2>::GetNextTimeStampStream2()
{
  return stream2_buff_.GetNextTimeStamp();
}

template<class T1, class T2>
size_t MessageStreamSequencer<T1, T2>::GetSizeStream1() {return stream1_buff_.Size();}

template<class T1, class T2>
size_t MessageStreamSequencer<T1, T2>::GetSizeStream2() {return stream2_buff_.Size();}

template<class T1, class T2>
void MessageStreamSequencer<T1, T2>::RegisterCallback(Callback callback)
{
  registered_callback_ = callback;
}

}  // namespace visual_slam
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_VISUAL_SLAM__IMPL__MESSAGE_STREAM_SEQUENCER_HPP_
