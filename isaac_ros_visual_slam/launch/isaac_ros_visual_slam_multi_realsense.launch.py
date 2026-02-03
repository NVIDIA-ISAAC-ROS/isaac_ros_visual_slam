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

import os

from ament_index_python.packages import get_package_share_directory
import isaac_ros_launch_utils as lu
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
import yaml


def realsense_capture(common_params, camera_params):
    stereo_capture = ComposableNode(
        name=camera_params['camera_name'],
        namespace='',
        package='realsense2_camera',
        plugin='realsense2_camera::RealSenseNodeFactory',
        parameters=[common_params | camera_params],
        extra_arguments=[{'use_intra_process_comms': True}]
    )
    return stereo_capture


def launch_setup(context, *args, **kwargs):
    config_path = context.perform_substitution(LaunchConfiguration('config_path'))

    config_directory = get_package_share_directory('isaac_ros_visual_slam')

    if config_path:
        rs_config_path = config_path
    else:
        rs_config_path = os.path.join(config_directory, 'config', 'multi_realsense.yaml')

    with open(rs_config_path, 'r') as rs_config_file:
        rs_config = yaml.safe_load(rs_config_file)

    remapping_list, optical_frames = [], []
    num_cameras = 2*len(rs_config['cameras'])

    for idx in range(num_cameras):
        infra_cnt = idx % 2+1
        camera_cnt = rs_config['cameras'][idx//2]['camera_name']
        optical_frames += [f'{camera_cnt}_infra{infra_cnt}_optical_frame']
        remapping_list += [(f'visual_slam/image_{idx}',
                            f'/{camera_cnt}/infra{infra_cnt}/image_rect_raw'),
                           (f'visual_slam/camera_info_{idx}',
                            f'/{camera_cnt}/infra{infra_cnt}/camera_info')]

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            'tracking_mode': 0,  # multi camera
            'enable_image_denoising': False,
            'rectified_images': True,
            'image_jitter_threshold_ms': 34.00,
            'base_frame': 'base_link',
            'enable_slam_visualization': True,
            'enable_landmarks_view': True,
            'enable_observations_view': True,
            'enable_ground_constraint_in_odometry': False,
            'enable_ground_constraint_in_slam': False,
            'enable_localization_n_mapping': True,
            'enable_debug_mode': False,
            'num_cameras': num_cameras,
            'min_num_images': num_cameras,
            'camera_optical_frames': optical_frames,
        }],
        remappings=remapping_list,
    )

    visual_slam_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=([visual_slam_node]),
        output='screen',
    )

    launch_actions = [visual_slam_launch_container]

    camera_load_actions = []
    incremental_delay_seconds = 5.0

    for idx, camera_config in enumerate(rs_config['cameras']):
        delay_seconds = idx * incremental_delay_seconds
        load_action = TimerAction(
            period=delay_seconds,
            actions=[
                LoadComposableNodes(
                    target_container='visual_slam_launch_container',
                    composable_node_descriptions=[realsense_capture(
                        rs_config['common_params'], camera_config)],
                )
            ]
        )
        camera_load_actions.append(load_action)
    launch_actions.extend(camera_load_actions)

    return launch_actions


def generate_launch_description():

    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value='',
        description=(
            'Path to multi RealSense configuration file.'
        )
    )

    camera_model_arg = DeclareLaunchArgument(
        'camera_model',
        default_value='d455',
        description='RealSense camera model (d435 or d455)'
    )

    realsense_rig_description = lu.include(
        'thor_devkit_realsense_rig_description',
        'launch/thor_devkit_realsense_rig_description.launch.py',
        launch_arguments={'camera_model': LaunchConfiguration('camera_model')},
    )

    config_directory = get_package_share_directory('isaac_ros_visual_slam')
    foxglove_xml_config = os.path.join(
        config_directory, 'config', 'foxglove_bridge_launch.xml')
    foxglove_bridge_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource([foxglove_xml_config])
    )

    return LaunchDescription([
        config_path_arg,
        camera_model_arg,
        realsense_rig_description,
        foxglove_bridge_launch,
        OpaqueFunction(function=launch_setup)
    ])
