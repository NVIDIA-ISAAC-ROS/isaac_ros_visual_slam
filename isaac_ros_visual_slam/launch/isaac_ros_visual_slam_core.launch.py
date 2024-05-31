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

from typing import Any, Dict

from isaac_ros_examples import IsaacROSLaunchFragment
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


class IsaacROSVisualSlamLaunchFragment(IsaacROSLaunchFragment):

    @staticmethod
    def get_composable_nodes(interface_specs: Dict[str, Any]) -> Dict[str, ComposableNode]:

        enable_image_denoising = LaunchConfiguration('enable_image_denoising')
        rectified_images = LaunchConfiguration('rectified_images')
        enable_slam_visualization = LaunchConfiguration('enable_slam_visualization')
        enable_landmarks_view = LaunchConfiguration('enable_landmarks_view')
        enable_observations_view = LaunchConfiguration('enable_observations_view')
        camera_optical_frames = LaunchConfiguration('camera_optical_frames')
        base_frame = LaunchConfiguration('base_frame')
        num_cameras = LaunchConfiguration('num_cameras')
        enable_imu_fusion = LaunchConfiguration('enable_imu_fusion')
        imu_frame = LaunchConfiguration('imu_frame')
        gyro_noise_density = LaunchConfiguration('gyro_noise_density')
        gyro_random_walk = LaunchConfiguration('gyro_random_walk')
        accel_noise_density = LaunchConfiguration('accel_noise_density')
        accel_random_walk = LaunchConfiguration('accel_random_walk')
        calibration_frequency = LaunchConfiguration('calibration_frequency')
        image_jitter_threshold_ms = LaunchConfiguration('image_jitter_threshold_ms')

        return {
            'image_format_converter_node_left': ComposableNode(
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
                name='image_format_node_left',
                parameters=[{
                        'encoding_desired': 'mono8',
                        'image_width': interface_specs['camera_resolution']['width'],
                        'image_height': interface_specs['camera_resolution']['height'],
                }],
                remappings=[
                    ('image_raw', 'left/image_rect'),
                    ('image', 'left/image_rect_mono')]
            ),
            'image_format_converter_node_right': ComposableNode(
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
                name='image_format_node_right',
                parameters=[{
                        'encoding_desired': 'mono8',
                        'image_width': interface_specs['camera_resolution']['width'],
                        'image_height': interface_specs['camera_resolution']['height'],
                }],
                remappings=[
                    ('image_raw', 'right/image_rect'),
                    ('image', 'right/image_rect_mono')]
            ),
            'visual_slam_node': ComposableNode(
                name='visual_slam_node',
                package='isaac_ros_visual_slam',
                plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
                parameters=[{
                    'enable_image_denoising': enable_image_denoising,
                    'rectified_images': rectified_images,
                    'enable_slam_visualization': enable_slam_visualization,
                    'enable_landmarks_view': enable_landmarks_view,
                    'enable_observations_view': enable_observations_view,
                    'camera_optical_frames': camera_optical_frames,
                    'base_frame': base_frame,
                    'num_cameras': num_cameras,
                    'enable_imu_fusion': enable_imu_fusion,
                    'imu_frame': imu_frame,
                    'gyro_noise_density': gyro_noise_density,
                    'gyro_random_walk': gyro_random_walk,
                    'accel_noise_density': accel_noise_density,
                    'accel_random_walk': accel_random_walk,
                    'calibration_frequency': calibration_frequency,
                    'image_jitter_threshold_ms': image_jitter_threshold_ms,
                }],
                remappings=[
                    ('/visual_slam/image_0', 'left/image_rect_mono'),
                    ('/visual_slam/camera_info_0', 'left/camera_info_rect'),
                    ('/visual_slam/image_1', 'right/image_rect_mono'),
                    ('/visual_slam/camera_info_1', 'right/camera_info_rect'),
                    ('/visual_slam/image_2', 'rear_left/image_rect_mono'),
                    ('/visual_slam/camera_info_2', 'rear_left/camera_info_rect'),
                    ('/visual_slam/image_3', 'rear_right/image_rect_mono'),
                    ('/visual_slam/camera_info_3', 'rear_right/camera_info_rect'),
                ]
            )
        }

    @staticmethod
    def get_launch_actions(interface_specs: Dict[str, Any]) -> \
            Dict[str, launch.actions.OpaqueFunction]:
        return {
            'enable_image_denoising': DeclareLaunchArgument(
                'enable_image_denoising',
                default_value='False'
            ),
            'rectified_images': DeclareLaunchArgument(
                'rectified_images',
                default_value='True'
            ),
            'enable_slam_visualization': DeclareLaunchArgument(
                'enable_slam_visualization',
                default_value='True'
            ),
            'enable_landmarks_view': DeclareLaunchArgument(
                'enable_landmarks_view',
                default_value='True'
            ),
            'enable_observations_view': DeclareLaunchArgument(
                'enable_observations_view',
                default_value='True'
            ),
            'camera_optical_frames': DeclareLaunchArgument(
                'camera_optical_frames',
                default_value='[front_stereo_camera_left_optical, \
                                front_stereo_camera_right_optical, \
                                rear_stereo_camera_left_optical, \
                                rear_stereo_camera_right_optical,\
                                ]'
            ),
            'base_frame': DeclareLaunchArgument(
                'base_frame',
                default_value='base_link'
            ),
            'num_cameras': DeclareLaunchArgument(
                'num_cameras',
                default_value='2'
            ),
            'enable_imu_fusion': DeclareLaunchArgument(
                'enable_imu_fusion',
                default_value='False'
            ),
            'imu_frame': DeclareLaunchArgument(
                'imu_frame',
                default_value='camera_gyro_optical_frame'
            ),
            'gyro_noise_density': DeclareLaunchArgument(
                'gyro_noise_density',
                default_value='0.000244'
            ),
            'gyro_random_walk': DeclareLaunchArgument(
                'gyro_random_walk',
                default_value='0.000019393'
            ),
            'accel_noise_density': DeclareLaunchArgument(
                'accel_noise_density',
                default_value='0.001862'
            ),
            'accel_random_walk': DeclareLaunchArgument(
                'accel_random_walk',
                default_value='0.003'
            ),
            'calibration_frequency': DeclareLaunchArgument(
                'calibration_frequency',
                default_value='200.0'
            ),
            'image_jitter_threshold_ms': DeclareLaunchArgument(
                'image_jitter_threshold_ms',
                default_value='34.0'
            )
        }


def generate_launch_description():
    """Launch file to bring up visual slam node standalone."""
    visual_slam_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=IsaacROSVisualSlamLaunchFragment
        .get_composable_nodes().values(),
        output='screen'
    )

    return launch.LaunchDescription([visual_slam_launch_container])
