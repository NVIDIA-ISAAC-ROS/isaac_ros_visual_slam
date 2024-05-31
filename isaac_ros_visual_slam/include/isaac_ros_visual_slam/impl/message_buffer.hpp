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

#ifndef ISAAC_ROS_VISUAL_SLAM__IMPL__MESSAGE_BUFFER_HPP_
#define ISAAC_ROS_VISUAL_SLAM__IMPL__MESSAGE_BUFFER_HPP_

#include <list>
#include <utility>
#include <vector>

namespace nvidia
{
namespace isaac_ros
{
namespace visual_slam
{

template<class T>
class MessageBuffer
{
public:
  explicit MessageBuffer(uint8_t maxsize);
  void Push(const int64_t & timestamp, const T & msg);
  T Pop();
  T Peek() const;
  size_t Size() const;
  bool IsEmpty() const;
  int64_t GetLastTimeStamp() const;
  int64_t GetCurrentTimeStamp() const;
  int64_t GetNextTimeStamp() const;
  void ClearAll();
  void ClearUpto(const int64_t & timestamp);
  std::vector<T> ClearAndGetUpto(const int64_t & timestamp);
  std::vector<T> GetAll() const;
  std::vector<T> GetUpto(const int64_t & start_time, const int64_t & end_time) const;

private:
  // Buffer size
  uint8_t max_size_;
  // Timestamp of the last processed / removed message
  int64_t last_msg_ts_;
  // Timestamp of the lastest message that is added to the buffer
  int64_t current_msg_ts_;
  // Timestamp of the next message to be processed
  int64_t next_msg_ts_;
  // Buffer to store a message along with a timestamp field.
  std::list<std::pair<int64_t, T>> buffer_;
};

template<typename T>
MessageBuffer<T>::MessageBuffer(uint8_t maxsize)
: max_size_{maxsize}, last_msg_ts_{-1}, current_msg_ts_{-1}, next_msg_ts_{-1} {}

template<typename T>
void MessageBuffer<T>::Push(const int64_t & timestamp, const T & msg)
{
  if (buffer_.size() >= max_size_) {buffer_.pop_front();}
  buffer_.emplace_back(std::make_pair(timestamp, msg));
  current_msg_ts_ = timestamp;
  next_msg_ts_ = buffer_.front().first;
}

template<typename T>
T MessageBuffer<T>::Pop()
{
  auto result = buffer_.front();
  buffer_.pop_front();
  last_msg_ts_ = result.first;
  next_msg_ts_ = buffer_.empty() ? -1 : buffer_.front().first;
  return result.second;
}

template<typename T>
T MessageBuffer<T>::Peek() const
{
  return buffer_.front().second;
}

template<typename T>
size_t MessageBuffer<T>::Size() const {return buffer_.size();}

template<typename T>
bool MessageBuffer<T>::IsEmpty() const {return buffer_.empty();}

template<typename T>
int64_t MessageBuffer<T>::GetLastTimeStamp() const {return last_msg_ts_;}

template<typename T>
int64_t MessageBuffer<T>::GetCurrentTimeStamp() const {return current_msg_ts_;}

template<typename T>
int64_t MessageBuffer<T>::GetNextTimeStamp() const {return next_msg_ts_;}

template<typename T>
void MessageBuffer<T>::ClearAll() {buffer_.clear();}

template<typename T>
void MessageBuffer<T>::ClearUpto(const int64_t & timestamp)
{
  buffer_.remove_if([timestamp](std::pair<int64_t, T> msg) {return msg.first <= timestamp;});
}

template<typename T>
std::vector<T> MessageBuffer<T>::ClearAndGetUpto(const int64_t & timestamp)
{
  const auto values = GetUpto(-1, timestamp);
  ClearUpto(timestamp);
  return values;
}

template<typename T>
std::vector<T> MessageBuffer<T>::GetUpto(const int64_t & start_time, const int64_t & end_time) const
{
  std::vector<T> result;
  for (auto msg : buffer_) {
    if (msg.first >= start_time && msg.first <= end_time) {result.emplace_back(msg.second);}
  }
  return result;
}

template<typename T>
std::vector<T> MessageBuffer<T>::GetAll() const
{
  std::vector<T> result;
  for (auto msg : buffer_) {
    result.emplace_back(msg.second);
  }
  return result;
}

}  // namespace visual_slam
}  // namespace isaac_ros
}  // namespace nvidia

#endif  // ISAAC_ROS_VISUAL_SLAM__IMPL__MESSAGE_BUFFER_HPP_
