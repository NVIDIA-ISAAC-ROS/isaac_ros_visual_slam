# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import launch
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """
    Launch file which brings up visual slam node configured for RealSense RGBD.

    This configuration uses aligned depth for pixel-wise correspondence with color images.
    Hardware sync is enabled to improve synchronization between color and depth streams.
    """
    realsense_camera_node = Node(
        name='camera',
        namespace='',
        package='realsense2_camera',
        executable='realsense2_camera_node',
        parameters=[{
            'enable_infra1': False,
            'enable_infra2': False,
            'enable_color': True,
            'enable_depth': True,
            'align_depth.enable': True,
            'enable_sync': True,
            'depth_module.emitter_enabled': 1,
            'depth_module.depth_profile': '640,360,60',
            'rgb_camera.color_profile': '640,360,60',
            'depth_module.enable_auto_exposure': True,
            'rgb_camera.enable_auto_exposure': True,
            'enable_gyro': False,
            'enable_accel': False,
            # Disable filters that add latency
            'decimation_filter.enable': False,
            'spatial_filter.enable': False,
            'temporal_filter.enable': False,
            'hole_filling_filter.enable': False,
        }],
    )

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            'tracking_mode': 2,  # RGBD mode
            'depth_scale_factor': 1000.0,
            'enable_image_denoising': False,
            'rectified_images': False,
            'image_jitter_threshold_ms': 20.00,  # Increased for aligned depth latency
            'sync_matching_threshold_ms': 10.0,  # Allow some sync tolerance
            'base_frame': 'camera_link',
            'enable_slam_visualization': True,
            'enable_landmarks_view': True,
            'enable_observations_view': True,
            'enable_ground_constraint_in_odometry': False,
            'enable_ground_constraint_in_slam': False,
            'enable_localization_n_mapping': True,
            'min_num_images': 1,
            'num_cameras': 1,
            'depth_camera_id': 0,
            'camera_optical_frames': [
                'camera_color_optical_frame'
            ],
        }],
        remappings=[
            ('visual_slam/image_0', '/camera/color/image_raw'),
            ('visual_slam/camera_info_0', '/camera/color/camera_info'),
            ('visual_slam/depth_0', '/camera/aligned_depth_to_color/image_raw'),
        ],
    )

    visual_slam_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[visual_slam_node],
        output='screen',
    )

    return launch.LaunchDescription([realsense_camera_node, visual_slam_launch_container])
