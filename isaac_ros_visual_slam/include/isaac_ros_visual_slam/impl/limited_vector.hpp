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

#ifndef ISAAC_ROS_VISUAL_SLAM__IMPL__LIMITED_VECTOR_HPP_
#define ISAAC_ROS_VISUAL_SLAM__IMPL__LIMITED_VECTOR_HPP_

#include <vector>

namespace isaac_ros
{
namespace visual_slam
{

// limited_vector
template<class T>
class limited_vector
{
  const uint max_size_;
  std::vector<T> data;

public:
  explicit limited_vector(uint max_size)
  : max_size_(max_size)
  {
  }
  void add(const T & value)
  {
    // tracking_vo_path_pub_
    if (data.size() >= max_size_) {
      std::rotate(data.begin(), data.begin() + 1, data.end());
      data.back() = value;
    } else {
      data.push_back(value);
    }
  }
  const std::vector<T> & getData() const
  {
    return data;
  }
};

}  // namespace visual_slam
}  // namespace isaac_ros

#endif  // ISAAC_ROS_VISUAL_SLAM__IMPL__LIMITED_VECTOR_HPP_
