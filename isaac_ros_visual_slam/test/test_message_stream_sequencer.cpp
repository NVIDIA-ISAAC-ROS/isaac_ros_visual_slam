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
#include <utility>
#include <vector>

#include "isaac_ros_visual_slam/impl/message_stream_sequencer.hpp"

using StreamIndex = uint64_t;
using TimestampType = int64_t;
using MsgType = int64_t;
// This defines a sequences of message on which stream it belongs to and its timestamp
using MessageSequence = std::vector<std::pair<StreamIndex, TimestampType>>;
using MessageStreamSequencer = nvidia::isaac_ros::visual_slam::MessageStreamSequencer<MsgType,
    MsgType>;

// This test fixture provides easy access to common methods and member variables that are used
// for validation and assertions
class MessageStreamSequencerTest : public ::testing::Test
{
public:
  // This callback is called by the MessageStreamSequencer. The signature puts and emphasis on
  // 1 or more messages for stream1 (faster stream) to 1 msg of stream2
  void Callback(std::vector<MsgType> stream1_msgs, MsgType stream2_msg)
  {
    for (auto msg : stream1_msgs) {
      interleaved_seq.emplace_back(std::make_pair(1, msg));
      ++out_size_stream1;
    }
    interleaved_seq.emplace_back(std::make_pair(2, stream2_msg));
    ++out_size_stream2;
  }

  // This method validates the output of MessageStreamSequencer. The timestamps of the interleaved
  // messages should always increase
  bool ValidateSeq()
  {
    int64_t curr_ts = 0;
    for (auto item : interleaved_seq) {
      if (item.second >= curr_ts) {curr_ts = item.second;} else {return false;}
    }
    return true;
  }

  // This method reads the sequence of messages and calls the respective callback of the message
  // based on which stream it belongs.
  void ProcessSeq(
    const MessageSequence & message_sequence, MessageStreamSequencer & stream_sequencer)
  {
    for (auto seq_item : message_sequence) {
      if (seq_item.first == 1) {
        stream_sequencer.CallbackStream1(seq_item.second, seq_item.second);
        ++in_size_stream1;
      } else {
        stream_sequencer.CallbackStream2(seq_item.second, seq_item.second);
        ++in_size_stream2;
      }
    }
  }

  MessageStreamSequencer GetMessageSequencer(
    uint8_t stream1_size, int stream1_frame_delta, int stream1_multiplier,
    uint8_t stream2_size, int stream2_frame_delta, int stream2_multiplier)
  {
    // Frame delta is difference between timestamps of current and previous frames
    // which is on average should be equals to 1/FPS

    int stream1_epsilon = stream1_multiplier * stream1_frame_delta;
    int stream2_epsilon = stream2_multiplier * stream2_frame_delta;

    MessageStreamSequencer stream_sequencer{stream1_size, stream1_epsilon, stream2_size,
      stream2_epsilon};
    stream_sequencer.RegisterCallback(
      std::bind(
        &MessageStreamSequencerTest::Callback, this, std::placeholders::_1, std::placeholders::_2));
    return stream_sequencer;
  }

protected:
  void SetUp() override {}

  MessageSequence interleaved_seq;
  size_t in_size_stream1{0}, in_size_stream2{0};
  size_t out_size_stream1{0}, out_size_stream2{0};
};


// Testing already interleaved sequence
// Only one message should be left in stream1 buffer after the sequence is processed
TEST_F(MessageStreamSequencerTest, TestSequence1)
{
  constexpr uint8_t kStream1Size = 30;
  constexpr int kStream1FrameDelta = 2;
  constexpr int kStream1Multipler = 1;
  constexpr uint8_t kStream2Size = 20;
  constexpr int kStream2FrameDelta = 5;
  constexpr int kStream2Multipler = 1;

  MessageStreamSequencer stream_sequencer = GetMessageSequencer(
    kStream1Size, kStream1FrameDelta, kStream1Multipler,
    kStream2Size, kStream2FrameDelta, kStream2Multipler);

  MessageSequence message_sequence{
    {2, 0}, {1, 1}, {1, 3}, {1, 5}, {2, 6}, {1, 7}, {1, 9}, {1, 11}, {2, 12}, {1, 13}, {1, 15},
    {1, 17}, {2, 18}, {1, 19}
  };

  ProcessSeq(message_sequence, stream_sequencer);

  ASSERT_TRUE(ValidateSeq());

  // Custom validation
  auto size_diff = message_sequence.size() - interleaved_seq.size();
  EXPECT_EQ(size_diff, static_cast<size_t>(1));
  EXPECT_EQ(in_size_stream1, out_size_stream1 + stream_sequencer.GetSizeStream1());
  EXPECT_EQ(in_size_stream2, out_size_stream2 + stream_sequencer.GetSizeStream2());

  if (in_size_stream1 - out_size_stream1) {
    EXPECT_LE(interleaved_seq.back().second, stream_sequencer.GetNextTimeStampStream1());
  }
  if (in_size_stream2 - out_size_stream2) {
    EXPECT_LE(interleaved_seq.back().second, stream_sequencer.GetNextTimeStampStream2());
  }
}

// Testing a sequence where one msg is out of order which will create a scenario
// where stream1 flush out will happen because of time out and last stream1_msg will be ignored
TEST_F(MessageStreamSequencerTest, TestSequence2)
{
  constexpr uint8_t kStream1Size = 30;
  constexpr int kStream1FrameDelta = 2;
  constexpr int kStream1Multipler = 1;
  constexpr uint8_t kStream2Size = 20;
  constexpr int kStream2FrameDelta = 5;
  constexpr int kStream2Multipler = 1;

  MessageStreamSequencer stream_sequencer = GetMessageSequencer(
    kStream1Size, kStream1FrameDelta, kStream1Multipler,
    kStream2Size, kStream2FrameDelta, kStream2Multipler);

  MessageSequence message_sequence{
    {2, 0}, {1, 1}, {1, 3}, {1, 5}, {2, 6}, {1, 7}, {1, 9}, {1, 11}, {2, 12}, {1, 13}, {1, 15},
    {1, 17}, {2, 18}, {2, 24}, {1, 19}
  };

  ProcessSeq(message_sequence, stream_sequencer);

  ASSERT_TRUE(ValidateSeq());

  // Custom validation
  EXPECT_GT(in_size_stream1, out_size_stream1 + stream_sequencer.GetSizeStream1());
  EXPECT_EQ(in_size_stream2, out_size_stream2 + stream_sequencer.GetSizeStream2());
  auto size_diff = message_sequence.size() - interleaved_seq.size();
  EXPECT_EQ(size_diff, static_cast<size_t>(1));
}

// Testing a sequence where stream1 will fall behind stream2 which will lead to messages delta
// going beyond epsilon because more consecutive messages from stream2
TEST_F(MessageStreamSequencerTest, TestSequence3)
{
  constexpr uint8_t kStream1Size = 30;
  constexpr int kStream1FrameDelta = 1;
  constexpr int kStream1Multipler = 1;
  constexpr uint8_t kStream2Size = 20;
  constexpr int kStream2FrameDelta = 1;
  constexpr int kStream2Multipler = 1;

  MessageStreamSequencer stream_sequencer = GetMessageSequencer(
    kStream1Size, kStream1FrameDelta, kStream1Multipler,
    kStream2Size, kStream2FrameDelta, kStream2Multipler);

  MessageSequence message_sequence{
    {2, 0}, {2, 5}, {1, 1}, {1, 3}, {1, 5}, {2, 10}, {2, 15}, {2, 20}, {1, 7}, {1, 9}, {2, 25},
    {1, 11}, {1, 13}, {1, 15}, {2, 30}, {2, 35}, {1, 17}, {1, 19}, {1, 21}, {2, 45}, {2, 55},
    {1, 25}, {2, 60}, {1, 31}, {1, 33}, {1, 39}, {2, 65}};

  ProcessSeq(message_sequence, stream_sequencer);

  ASSERT_TRUE(ValidateSeq());
}

// Same a sequnce 3 above with different epsilon values
TEST_F(MessageStreamSequencerTest, TestSequence4)
{
  constexpr uint8_t kStream1Size = 30;
  constexpr int kStream1FrameDelta = 2;
  constexpr int kStream1Multipler = 5;
  constexpr uint8_t kStream2Size = 20;
  constexpr int kStream2FrameDelta = 5;
  constexpr int kStream2Multipler = 5;

  MessageStreamSequencer stream_sequencer = GetMessageSequencer(
    kStream1Size, kStream1FrameDelta, kStream1Multipler,
    kStream2Size, kStream2FrameDelta, kStream2Multipler);

  MessageSequence message_sequence{
    {2, 0}, {2, 5}, {1, 1}, {1, 3}, {1, 5}, {2, 10}, {2, 15}, {2, 20}, {1, 7}, {1, 9}, {2, 25},
    {1, 11}, {1, 13}, {1, 15}, {2, 30}, {2, 35}, {1, 17}, {1, 19}, {1, 21}, {2, 45}, {2, 55},
    {1, 25}, {2, 60}, {1, 31}, {1, 33}, {1, 39}, {2, 65}};

  ProcessSeq(message_sequence, stream_sequencer);

  ASSERT_TRUE(ValidateSeq());
}

// Testing a sequence where epsilon is infinity. This will lead to 1 message each remaining on both
// the streams waiting for next message to come.
TEST_F(MessageStreamSequencerTest, TestSequence5)
{
  constexpr uint8_t kStream1Size = 30;
  constexpr int kStream1FrameDelta = 1;
  constexpr int kStream1Multipler = 1000000;
  constexpr uint8_t kStream2Size = 20;
  constexpr int kStream2FrameDelta = 1;
  constexpr int kStream2Multipler = 1000000;

  MessageStreamSequencer stream_sequencer = GetMessageSequencer(
    kStream1Size, kStream1FrameDelta, kStream1Multipler,
    kStream2Size, kStream2FrameDelta, kStream2Multipler);

  MessageSequence message_sequence{
    {2, 0}, {2, 5}, {1, 1}, {1, 3}, {1, 5}, {2, 10}, {2, 15}, {2, 20}, {1, 7}, {1, 9}, {2, 25},
    {1, 11}, {1, 13}, {1, 15}, {2, 30}, {2, 35}, {1, 17}, {1, 19}, {1, 21}, {2, 45}, {2, 55},
    {1, 25}, {2, 60}, {1, 31}, {1, 33}, {1, 39}, {2, 65}, {1, 41}, {1, 45}, {1, 51}, {1, 53},
    {2, 70}, {1, 55}, {1, 57}, {1, 59}, {1, 61}, {1, 63}, {1, 65}, {1, 73}, {2, 75}};

  ProcessSeq(message_sequence, stream_sequencer);

  ASSERT_TRUE(ValidateSeq());

  // Custom validation
  EXPECT_EQ(in_size_stream1, out_size_stream1 + stream_sequencer.GetSizeStream1());
  EXPECT_EQ(in_size_stream2, out_size_stream2 + stream_sequencer.GetSizeStream2());
  auto size_diff = message_sequence.size() - interleaved_seq.size();
  EXPECT_EQ(size_diff, static_cast<size_t>(2));
  if (in_size_stream1 - out_size_stream1) {
    EXPECT_LE(interleaved_seq.back().second, stream_sequencer.GetNextTimeStampStream1());
  }
  if (in_size_stream2 - out_size_stream2) {
    EXPECT_LE(interleaved_seq.back().second, stream_sequencer.GetNextTimeStampStream2());
  }
}

// Testing a sequence for timeout scenario. Timeout between same stream and across streams
// In this sequence the delta between stream2 messages increases which will trigger the timeout
// flushes of the stream2. And sporadic stream1 messages
TEST_F(MessageStreamSequencerTest, TestSequence6)
{
  constexpr uint8_t kStream1Size = 30;
  constexpr int kStream1FrameDelta = 2;
  constexpr int kStream1Multipler = 3;
  constexpr uint8_t kStream2Size = 30;
  constexpr int kStream2FrameDelta = 5;
  constexpr int kStream2Multipler = 3;

  MessageStreamSequencer stream_sequencer = GetMessageSequencer(
    kStream1Size, kStream1FrameDelta, kStream1Multipler,
    kStream2Size, kStream2FrameDelta, kStream2Multipler);

  MessageSequence message_sequence{
    {1, 0}, {2, 5}, {2, 10}, {1, 35}, {2, 40}, {1, 56}, {1, 57}, {2, 50}, {2, 70}, {2, 100},
    {1, 85}, {1, 135}, {2, 140}, {2, 145}, {2, 155}, {2, 165}, {2, 170}, {1, 155}
  };

  ProcessSeq(message_sequence, stream_sequencer);

  ASSERT_TRUE(ValidateSeq());

  // Custom validation
  auto size_diff = message_sequence.size() - interleaved_seq.size();
  EXPECT_EQ(size_diff, static_cast<size_t>(2));
}

// Testing a sequence just with stream2 msgs. All messages will get processed as no stream1 messages
// are present
TEST_F(MessageStreamSequencerTest, TestSequence7)
{
  constexpr uint8_t kStream1Size = 30;
  constexpr int kStream1FrameDelta = 1;
  constexpr int kStream1Multipler = 3;
  constexpr uint8_t kStream2Size = 30;
  constexpr int kStream2FrameDelta = 5;
  constexpr int kStream2Multipler = 3;

  MessageStreamSequencer stream_sequencer = GetMessageSequencer(
    kStream1Size, kStream1FrameDelta, kStream1Multipler,
    kStream2Size, kStream2FrameDelta, kStream2Multipler);

  MessageSequence message_sequence{
    {2, 0}, {2, 5}, {2, 10}, {2, 40}, {2, 50}, {2, 70}, {2, 100}, {2, 140}, {2, 145}, {2, 155},
    {2, 165}, {2, 170}
  };

  ProcessSeq(message_sequence, stream_sequencer);

  ASSERT_TRUE(ValidateSeq());

  // Custom validation
  EXPECT_EQ(in_size_stream1, out_size_stream1 + stream_sequencer.GetSizeStream1());
  EXPECT_EQ(in_size_stream2, out_size_stream2 + stream_sequencer.GetSizeStream2());
  auto size_diff = message_sequence.size() - interleaved_seq.size();
  EXPECT_EQ(size_diff, static_cast<size_t>(0));
}

// Testing a sequence just with stream1 msgs. No messages should be processed as no stream2 messages
// are present
TEST_F(MessageStreamSequencerTest, TestSequence8)
{
  constexpr uint8_t kStream1Size = 30;
  constexpr int kStream1FrameDelta = 2;
  constexpr int kStream1Multipler = 3;
  constexpr uint8_t kStream2Size = 30;
  constexpr int kStream2FrameDelta = 5;
  constexpr int kStream2Multipler = 3;

  MessageStreamSequencer stream_sequencer = GetMessageSequencer(
    kStream1Size, kStream1FrameDelta, kStream1Multipler,
    kStream2Size, kStream2FrameDelta, kStream2Multipler);

  MessageSequence message_sequence{
    {1, 0}, {1, 5}, {1, 10}, {1, 40}, {1, 50}, {1, 70}, {1, 100}, {1, 140}, {1, 145}, {1, 155},
    {1, 165}, {1, 170}
  };

  ProcessSeq(message_sequence, stream_sequencer);

  ASSERT_TRUE(ValidateSeq());

  // Custom validation
  EXPECT_EQ(in_size_stream1, out_size_stream1 + stream_sequencer.GetSizeStream1());
  EXPECT_EQ(in_size_stream2, out_size_stream2 + stream_sequencer.GetSizeStream2());
  auto size_diff = message_sequence.size() - interleaved_seq.size();
  EXPECT_EQ(size_diff, in_size_stream1);
}
