# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node, SetParameter, SetRemap
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Launch file which brings up visual slam node configured for Hawk."""
    # Conditionals for rectify setup
    use_rectify_arg = DeclareLaunchArgument(
        'use_rectify', default_value='False', description='Whether to use rectify nodes')
    use_rectify = IfCondition(LaunchConfiguration('use_rectify', default='False'))

    argus_imu_camera_static_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0947', '0.0061', '0.0', '0.0', '0.70710678',
                   '0.0', '0.70710678', 'camera', 'bmi088_frame']
    )

    correlated_timestamp_driver_node = ComposableNode(
        package='isaac_ros_correlated_timestamp_driver',
        plugin='nvidia::isaac_ros::correlated_timestamp_driver::CorrelatedTimestampDriverNode',
        name='correlated_timestamp_driver',
        parameters=[{'use_time_since_epoch': False,
                     'nvpps_dev_name': '/dev/nvpps0'}])

    hawk_front_node = ComposableNode(
        name='hawk_front_node',
        package='isaac_ros_hawk',
        plugin='nvidia::isaac_ros::hawk::HawkNode',
        parameters=[{'module_id': 5}]
    )

    bmi088_node = ComposableNode(
        package='isaac_ros_imu_bmi088',
        plugin='nvidia::isaac_ros::imu_bmi088::Bmi088Node',
        name='bmi088',
        parameters=[{'imu_frequency': 200,
                     'accel_index': 0,
                     'gyro_index': 1,
                     'iio_buf_size': 64}],
        remappings=[
            ('imu', 'camera/imu')
        ])

    left_rectify_node = ComposableNode(
        name='left_rectify_node',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        parameters=[{
            'output_width': 1920,
            'output_height': 1200
        }],
        remappings=[
            ('image_raw', 'left/image_raw'),
            ('camera_info', 'left/camerainfo'),
            ('image_rect', 'left/image_rect'),
            ('camera_info_rect', 'left/camera_info_rect')
        ]
    )

    right_rectify_node = ComposableNode(
        name='right_rectify_node',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        parameters=[{
            'output_width': 1920,
            'output_height': 1200,
        }],
        remappings=[
            ('image_raw', 'right/image_raw'),
            ('camera_info', 'right/camerainfo'),
            ('image_rect', 'right/image_rect'),
            ('camera_info_rect', 'right/camera_info_rect')
        ]
    )

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
                    'denoise_input_images': False,
                    'enable_debug_mode': False,
                    'debug_dump_path': '/tmp/cuvslam',
                    'enable_slam_visualization': True,
                    'enable_landmarks_view': True,
                    'enable_observations_view': True,
                    'map_frame': 'map',
                    'odom_frame': 'odom',
                    'base_frame': 'camera',
                    'input_imu_frame': 'bmi088_frame',
                    'enable_imu_fusion': True,
                    'gyro_noise_density': 0.000244,
                    'gyro_random_walk': 0.000019393,
                    'accel_noise_density': 0.001862,
                    'accel_random_walk': 0.003,
                    'calibration_frequency': 200.0,
                    'verbosity': True,
                    'min_cam_fps_threshold': 29
                    }],
        remappings=[('stereo_camera/left/image', 'left/image_raw'),
                    ('stereo_camera/left/camera_info', 'left/camerainfo'),
                    ('stereo_camera/right/image', 'right/image_raw'),
                    ('stereo_camera/right/camera_info', 'right/camerainfo'),
                    ('visual_slam/imu', 'camera/imu')]
    )

    visual_slam_with_rectify_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        package='rclcpp_components',
        executable='component_container',
        namespace='',
        composable_node_descriptions=[
            hawk_front_node,
            correlated_timestamp_driver_node,
            bmi088_node,
            left_rectify_node,
            right_rectify_node,
            visual_slam_node
        ],
        output='screen',
        condition=use_rectify
    )

    visual_slam_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        package='rclcpp_components',
        executable='component_container',
        namespace='',
        composable_node_descriptions=[
            hawk_front_node,
            correlated_timestamp_driver_node,
            bmi088_node,
            visual_slam_node
        ],
        output='screen',
        condition=LaunchConfigurationEquals('use_rectify', 'False')
    )

    group_action = GroupAction([
        use_rectify_arg,

        # Changes for using rectification node
        SetParameter(name='rectified_images', value=False,
                     condition=LaunchConfigurationEquals('use_rectify', 'False')),

        SetRemap(src=['stereo_camera/left/image'],
                 dst=['left/image_rect'],
                 condition=use_rectify),
        SetRemap(src=['stereo_camera/left/camera_info'],
                 dst=['left/camera_info_rect'],
                 condition=use_rectify),
        SetRemap(src=['stereo_camera/right/image'],
                 dst=['right/image_rect'],
                 condition=use_rectify),
        SetRemap(src=['stereo_camera/right/camera_info'],
                 dst=['right/camera_info_rect'],
                 condition=use_rectify),

        visual_slam_container,
        visual_slam_with_rectify_container,
        argus_imu_camera_static_transform
    ])

    return launch.LaunchDescription([group_action])
