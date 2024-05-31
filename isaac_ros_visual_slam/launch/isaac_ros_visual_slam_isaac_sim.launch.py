# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Launch file which brings up visual slam node configured for Isaac Sim."""
    image_format_converter_node_left = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        name='image_format_node_left',
        parameters=[{
                'encoding_desired': 'mono8',
                'image_width': 1920,
                'image_height': 1200
        }],
        remappings=[
            ('image_raw', 'front_stereo_camera/left_rgb/image_raw'),
            ('image', 'front_stereo_camera/left_rgb/image_raw_mono')]
    )
    image_format_converter_node_right = ComposableNode(
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
        name='image_format_node_right',
        parameters=[{
                'encoding_desired': 'mono8',
                'image_width': 1920,
                'image_height': 1200
        }],
        remappings=[
            ('image_raw', 'front_stereo_camera/right_rgb/image_raw'),
            ('image', 'front_stereo_camera/right_rgb/image_raw_mono')]
    )
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        remappings=[('visual_slam/image_0', 'front_stereo_camera/left_rgb/image_raw_mono'),
                    ('visual_slam/camera_info_0',
                     'front_stereo_camera/left_rgb/camerainfo'),
                    ('visual_slam/image_1', 'front_stereo_camera/right_rgb/image_raw_mono'),
                    ('visual_slam/camera_info_1',
                     'front_stereo_camera/right_rgb/camerainfo')
                    ],
        parameters=[{
                    'use_sim_time': True,
                    'enable_image_denoising': True,
                    'rectified_images': True,
                    'enable_slam_visualization': True,
                    'enable_observations_view': True,
                    'enable_landmarks_view': True,
                    }]

    )

    visual_slam_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[visual_slam_node,
                                      image_format_converter_node_left,
                                      image_format_converter_node_right],
        output='screen'
    )

    return launch.LaunchDescription([visual_slam_launch_container])
