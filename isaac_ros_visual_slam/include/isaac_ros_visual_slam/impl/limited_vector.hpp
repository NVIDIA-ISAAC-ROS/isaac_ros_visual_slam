/**
 * Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef ISAAC_ROS_VISUAL_SLAM__IMPL__LIMITED_VECTOR_HPP_
#define ISAAC_ROS_VISUAL_SLAM__IMPL__LIMITED_VECTOR_HPP_

#include <vector>

namespace isaac_ros
{
namespace visual_slam
{

// limited_vector
//
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
