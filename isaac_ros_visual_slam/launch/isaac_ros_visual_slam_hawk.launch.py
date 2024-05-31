# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import (ComposableNodeContainer, LoadComposableNodes, Node, SetParameter,
                                SetRemap)
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Launch file which brings up visual slam node configured for Hawk."""
    launch_args = [
        DeclareLaunchArgument('module_id',
                              default_value='5',
                              description='Index specifying the stereo camera module to use.'),
        DeclareLaunchArgument('use_rectify',
                              default_value='False',
                              description='Whether to use rectify nodes')
    ]

    use_rectify = LaunchConfiguration('use_rectify')
    module_id = LaunchConfiguration('module_id')

    argus_imu_camera_static_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0.0947', '0.0061', '0.0', '0.0', '0.70710678', '0.0', '0.70710678', 'camera',
            'bmi088_frame'
        ],
    )

    correlated_timestamp_driver_node = ComposableNode(
        package='isaac_ros_correlated_timestamp_driver',
        plugin='nvidia::isaac_ros::correlated_timestamp_driver::CorrelatedTimestampDriverNode',
        name='correlated_timestamp_driver',
        parameters=[{
            'use_time_since_epoch': False,
            'nvpps_dev_file': '/dev/nvpps0'
        }])

    hawk_node = ComposableNode(
        name='hawk_front_node',
        package='isaac_ros_hawk',
        plugin='nvidia::isaac_ros::hawk::HawkNode',
        parameters=[{
            'module_id': module_id
        }],
    )

    bmi088_node = ComposableNode(
        package='isaac_ros_imu_bmi088',
        plugin='nvidia::isaac_ros::imu_bmi088::Bmi088Node',
        name='bmi088',
        parameters=[{
            'imu_frequency': 200,
            'accel_index': 0,
            'gyro_index': 1,
            'iio_buf_size': 64
        }],
        remappings=[('imu', 'camera/imu')],
    )

    left_rectify_node = ComposableNode(
        name='left_rectify_node',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        parameters=[{
            'output_width': 1920,
            'output_height': 1200
        }],
        remappings=[('image_raw', 'left/image_raw'), ('camera_info', 'left/camera_info'),
                    ('image_rect', 'left/image_rect'),
                    ('camera_info_rect', 'left/camera_info_rect')],
    )

    right_rectify_node = ComposableNode(
        name='right_rectify_node',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        parameters=[{
            'output_width': 1920,
            'output_height': 1200,
        }],
        remappings=[('image_raw', 'right/image_raw'), ('camera_info', 'right/camera_info'),
                    ('image_rect', 'right/image_rect'),
                    ('camera_info_rect', 'right/camera_info_rect')],
    )

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            'enable_image_denoising': False,
            'img_mask_bottom': 30,
            'img_mask_left': 150,
            'img_mask_right': 30,
            'enable_imu_fusion': True,
            'gyro_noise_density': 0.000244,
            'gyro_random_walk': 0.000019393,
            'accel_noise_density': 0.001862,
            'accel_random_walk': 0.003,
            'calibration_frequency': 200.0,
            'base_frame': 'camera',
            'imu_frame': 'bmi088_frame',
            'enable_ground_constraint_in_odometry': True,
            'enable_ground_constraint_in_slam': False,
            'enable_localization_n_mapping': False,
            'path_max_size': 1024,
            'enable_slam_visualization': True,
            'enable_landmarks_view': True,
            'enable_observations_view': True,
        }],
        remappings=[
            ('visual_slam/image_0', 'left/image_color'),
            ('visual_slam/camera_info_0', 'left/camera_info'),
            ('visual_slam/image_1', 'right/image_color'),
            ('visual_slam/camera_info_1', 'right/camera_info'),
            ('visual_slam/imu', 'camera/imu'),
        ],
    )

    visual_slam_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        package='rclcpp_components',
        executable='component_container_mt',
        namespace='',
        composable_node_descriptions=[
            hawk_node, correlated_timestamp_driver_node, bmi088_node, visual_slam_node
        ],
        output='screen',
    )

    load_rectify_nodes = LoadComposableNodes(
        target_container='visual_slam_launch_container',
        composable_node_descriptions=[left_rectify_node, right_rectify_node],
        condition=IfCondition(use_rectify),
    )

    group_action = GroupAction(launch_args + [
        SetParameter(name='rectified_images', value=False, condition=UnlessCondition(use_rectify)),
        # If we are using unrectified images:
        SetRemap(src=['visual_slam/camera_info_0'],
                 dst=['left/camera_info'],
                 condition=UnlessCondition(use_rectify)),
        SetRemap(src=['visual_slam/camera_info_1'],
                 dst=['right/camera_info'],
                 condition=UnlessCondition(use_rectify)),
        SetRemap(src=['visual_slam/image_0'],
                 dst=['left/image_raw'],
                 condition=UnlessCondition(use_rectify)),
        SetRemap(src=['visual_slam/image_1'],
                 dst=['right/image_raw'],
                 condition=UnlessCondition(use_rectify)),
        # If we are using rectified images:
        SetRemap(src=['visual_slam/camera_info_0'],
                 dst=['left/camera_info_rect'],
                 condition=IfCondition(use_rectify)),
        SetRemap(src=['visual_slam/camera_info_1'],
                 dst=['right/camera_info_rect'],
                 condition=IfCondition(use_rectify)),
        SetRemap(src=['visual_slam/image_0'],
                 dst=['left/image_rect'],
                 condition=IfCondition(use_rectify)),
        SetRemap(src=['visual_slam/image_1'],
                 dst=['right/image_rect'],
                 condition=IfCondition(use_rectify)),
        argus_imu_camera_static_transform,
        visual_slam_container,
        load_rectify_nodes,
    ])

    return launch.LaunchDescription([group_action])
