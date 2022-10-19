// SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
// Copyright (c) 2021-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// SPDX-License-Identifier: Apache-2.0

#ifndef ISAAC_ROS_VISUAL_SLAM__IMPL__STOPWATCH_HPP_
#define ISAAC_ROS_VISUAL_SLAM__IMPL__STOPWATCH_HPP_

#include <chrono>
#include <sstream>
#include <string>

namespace isaac_ros
{
namespace visual_slam
{

/*
Usage:
#include "stopwatch.hpp"
Stopwatch stopwatch;
for(;;) {
    isaac_ros::visual_slam::StopwatchScope ssw(stopwatch);
    // measured code ...
}

std::cout << "Statistic: " << stopwatch.Verbose() << std::endl;
*/

class Stopwatch
{
public:
  unsigned int counts = 0;
  std::chrono::nanoseconds duration;

public:
  Stopwatch()
  {
    duration = duration.zero();
  }
  std::string Verbose() const
  {
    double d = Seconds();
    double mean = counts ? d / counts : 0;
    std::stringstream ss;
    ss << counts << " calls " << d << "sec: " << mean << " sec per call";
    return ss.str();
  }
  double Seconds() const
  {
    constexpr double kNanoSecInSec = 1000000000.0;
    return duration.count() / kNanoSecInSec;
  }
  unsigned int Times() const
  {
    return counts;
  }
};

class StopwatchScope
{
  Stopwatch * stopwatch_ = nullptr;
  std::chrono::high_resolution_clock::time_point start_;

public:
  explicit StopwatchScope(Stopwatch & stopwatch)
  : stopwatch_(&stopwatch)
  {
    start_ = std::chrono::high_resolution_clock::now();
  }
  double Stop()
  {
    if (!stopwatch_) {
      return 0;
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::nanoseconds d =
      std::chrono::duration_cast<std::chrono::nanoseconds>(end - start_);
    stopwatch_->duration += d;
    stopwatch_->counts++;
    stopwatch_ = nullptr;
    constexpr double kNanoSecInSec = 1000000000.0;
    return d.count() / kNanoSecInSec;
  }
  ~StopwatchScope()
  {
    this->Stop();
  }
};

}  // namespace visual_slam
}  // namespace isaac_ros

#endif  // ISAAC_ROS_VISUAL_SLAM__IMPL__STOPWATCH_HPP_
