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

#include "isaac_ros_cuvslam/impl/limited_vector.hpp"

using LimitedVector = nvidia::isaac_ros::visual_slam::limited_vector<int>;

// path_max_size is exposed as a ROS 2 node parameter with no lower-bound check, so a
// zero-size limited_vector must be reachable without crashing.
TEST(LimitedVectorTest, AddWithZeroMaxSizeDoesNotCrash)
{
  LimitedVector vec(0);
  vec.add(42);
  EXPECT_TRUE(vec.getData().empty());
}

TEST(LimitedVectorTest, AddFillsUpToMaxSize)
{
  LimitedVector vec(3);
  vec.add(1);
  vec.add(2);
  EXPECT_EQ(vec.getData(), std::vector<int>({1, 2}));
}

TEST(LimitedVectorTest, AddSlidesWindowOnceFull)
{
  LimitedVector vec(3);
  vec.add(1);
  vec.add(2);
  vec.add(3);
  vec.add(4);
  EXPECT_EQ(vec.getData(), std::vector<int>({2, 3, 4}));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
