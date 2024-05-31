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

#include <gtest/gtest.h>

#include "isaac_ros_visual_slam/impl/message_buffer.hpp"
#include "std_msgs/msg/header.hpp"


// This test fixture provides a message buffer of size = 5 with header msgs and timestamp vectors
// for easy access in the test cases
class MessageBufferTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    for (uint8_t i = 0; i < 5; i++) {
      std_msgs::msg::Header header_msg;
      header_msg.stamp.sec = timestamps[i];
      header_msg.stamp.nanosec = 0;
      msgs.emplace_back(header_msg);
    }
  }
  std::vector<int64_t> timestamps {1, 2, 3, 4, 5};
  std::vector<std_msgs::msg::Header> msgs;
  nvidia::isaac_ros::visual_slam::MessageBuffer<std_msgs::msg::Header> msg_buffer{5};
};

// This test is verifying the push and pop functionality.
// We are pushing 5 messages and popping out 5
TEST_F(MessageBufferTest, TestPushAndPop)
{
  // Initial size should be zero
  EXPECT_EQ(msg_buffer.Size(), static_cast<size_t>(0));
  for (uint8_t i = 0; i < 5; i++) {
    msg_buffer.Push(timestamps[i], msgs[i]);
  }
  ASSERT_FALSE(msg_buffer.IsEmpty());
  EXPECT_EQ(msg_buffer.Size(), static_cast<size_t>(5));

  auto msg = msg_buffer.Pop();
  ASSERT_FALSE(msg_buffer.IsEmpty());
  EXPECT_EQ(msg_buffer.Size(), static_cast<size_t>(4));
  msg_buffer.Pop();
  msg_buffer.Pop();
  msg_buffer.Pop();
  msg_buffer.Pop();
  ASSERT_TRUE(msg_buffer.IsEmpty());
  EXPECT_EQ(msg_buffer.Size(), static_cast<size_t>(0));
}

// This test is verifying the timestamps in the buffer when performing push and pop operations
// LastTimeStamp = Timestamp of last message popped
// CurrentTimeStamp = Timestamp of lastest message pushed
// NextTimeStamp = Timestamp of first / earliest message in the buffer
TEST_F(MessageBufferTest, TestTimestamps)
{
  // Cheking timestamps before push
  ASSERT_EQ(msg_buffer.GetLastTimeStamp(), -1);
  ASSERT_EQ(msg_buffer.GetCurrentTimeStamp(), -1);
  ASSERT_EQ(msg_buffer.GetNextTimeStamp(), -1);

  for (uint8_t i = 0; i < 5; i++) {
    msg_buffer.Push(timestamps[i], msgs[i]);
  }
  // Checking timestamps when only push is done
  EXPECT_EQ(msg_buffer.GetLastTimeStamp(), -1);
  EXPECT_EQ(msg_buffer.GetNextTimeStamp(), 1);
  EXPECT_EQ(msg_buffer.GetCurrentTimeStamp(), 5);

  auto msg = msg_buffer.Pop();

  // Checking timestamps after pop
  EXPECT_EQ(msg_buffer.GetLastTimeStamp(), 1);
  EXPECT_EQ(msg_buffer.GetNextTimeStamp(), 2);
  EXPECT_EQ(msg_buffer.GetCurrentTimeStamp(), 5);

  msg_buffer.Push(timestamps[4] + 1, msgs[4]);

  // Checking timestamps after pop
  EXPECT_EQ(msg_buffer.GetLastTimeStamp(), 1);
  EXPECT_EQ(msg_buffer.GetNextTimeStamp(), 2);
  EXPECT_EQ(msg_buffer.GetCurrentTimeStamp(), 6);

  for (uint8_t i = 0; i < 4; i++) {
    msg_buffer.Pop();
    EXPECT_EQ(msg_buffer.GetLastTimeStamp(), 2 + i);
    EXPECT_EQ(msg_buffer.GetNextTimeStamp(), 2 + (i + 1));
    EXPECT_EQ(msg_buffer.GetCurrentTimeStamp(), 6);
  }
  msg_buffer.Pop();
  ASSERT_TRUE(msg_buffer.IsEmpty());
  EXPECT_EQ(msg_buffer.GetLastTimeStamp(), msg_buffer.GetCurrentTimeStamp());
  EXPECT_EQ(msg_buffer.GetNextTimeStamp(), -1);
  EXPECT_EQ(msg_buffer.GetCurrentTimeStamp(), 6);
}

// This test is verifying the fixed width buffer is correct and size never exceeds 5
TEST_F(MessageBufferTest, TestFixedBufferSize)
{
  for (uint8_t i = 0; i < 7; i++) {
    std_msgs::msg::Header header_msg;
    header_msg.stamp.sec = i;
    header_msg.stamp.nanosec = 0;
    msg_buffer.Push(i, header_msg);
  }
  ASSERT_FALSE(msg_buffer.IsEmpty());
  EXPECT_EQ(msg_buffer.Size(), static_cast<size_t>(5));

  auto all_msgs = msg_buffer.GetAll();
  for (uint8_t i = 0; i < 5; i++) {
    EXPECT_EQ(all_msgs[i].stamp.sec, i + 2);
  }
}

// This test is verifying the range access and range delete functionality
// GetUpto includes a time range which is inclusive of [start_time, end_time]
// ClearUpto takes a timepoint uptill which all data should be cleared including the timepoint
// ClearAndGetUpto is the combination of ClearUpto and GetUpto
TEST_F(MessageBufferTest, TestRangeAccessAndDelete)
{
  for (uint8_t i = 0; i < 5; i++) {
    msg_buffer.Push(timestamps[i], msgs[i]);
  }

  auto all_msgs = msg_buffer.GetAll();
  ASSERT_EQ(all_msgs.size(), static_cast<size_t>(5));
  for (uint8_t i = 0; i < 5; i++) {
    EXPECT_EQ(all_msgs[i].stamp.sec, i + 1);
  }

  auto range_msgs1 = msg_buffer.GetUpto(2, 4);
  ASSERT_EQ(range_msgs1.size(), static_cast<size_t>(3));
  EXPECT_EQ(range_msgs1[0].stamp.sec, 2);
  EXPECT_EQ(range_msgs1[1].stamp.sec, 3);
  EXPECT_EQ(range_msgs1[2].stamp.sec, 4);

  auto range_msgs2 = msg_buffer.ClearAndGetUpto(3);
  ASSERT_EQ(range_msgs2.size(), static_cast<size_t>(3));
  EXPECT_EQ(range_msgs2[0].stamp.sec, 1);
  EXPECT_EQ(range_msgs2[1].stamp.sec, 2);
  EXPECT_EQ(range_msgs2[2].stamp.sec, 3);

  msg_buffer.ClearUpto(4);
  ASSERT_EQ(msg_buffer.Size(), static_cast<size_t>(1));

  msg_buffer.ClearAll();
  ASSERT_EQ(msg_buffer.Size(), static_cast<size_t>(0));
}
