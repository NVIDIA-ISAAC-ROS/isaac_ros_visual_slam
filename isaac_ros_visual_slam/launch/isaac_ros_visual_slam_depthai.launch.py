# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.descriptions import ComposableNode
def generate_launch_description():
    """Launch file which brings up visual slam node configured for OAK."""

    depthai_prefix = get_package_share_directory('depthai_ros_driver')
    nvidia_slam_prefix = get_package_share_directory('isaac_ros_visual_slam')
    params_file = os.path.join(nvidia_slam_prefix, 'params', 'depthai.yaml')

    depthai = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(depthai_prefix, 'launch', 'camera.launch.py')),
            launch_arguments={"name": 'oak',
                              "params_file": params_file,
                               }.items())

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
                    'denoise_input_images': False,
                    'rectified_images': True,
                    'enable_debug_mode': False,
                    'debug_dump_path': '/tmp/cuvslam',
                    'enable_slam_visualization': True,
                    'enable_landmarks_view': True,
                    'enable_observations_view': True,
                    'map_frame': 'map',
                    'odom_frame': 'odom',
                    'base_frame': 'oak-d-base-frame',
                    'input_imu_frame': 'oak_imu_frame',
                    'enable_imu_fusion': True,
                    'gyro_noise_density': 0.9084722604427307 ,
                    'gyro_random_walk': 0.3189056080587463 ,
                    'accel_noise_density': 4.072394996046438 ,
                    'accel_random_walk': 1.9007919574432275 ,
                    'calibration_frequency': 100.0,
                    'img_jitter_threshold_ms': 22.00
                    }],
        remappings=[('stereo_camera/left/image', '/oak/left/image_rect'),
                    ('stereo_camera/left/camera_info', '/oak/left/camera_info'),
                    ('stereo_camera/right/image', '/oak/right/image_rect'),
                    ('stereo_camera/right/camera_info', '/oak/right/camera_info'),
                    ('visual_slam/imu', '/oak/imu/data')]
    )

    visual_slam_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            visual_slam_node
        ],
        output='screen'
    )

    return launch.LaunchDescription([visual_slam_launch_container, depthai])
