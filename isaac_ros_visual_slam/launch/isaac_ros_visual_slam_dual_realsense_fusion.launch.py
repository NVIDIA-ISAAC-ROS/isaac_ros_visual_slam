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

import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """
    Launch file for 2 visual RealSense SLAM nodes fused with robot_localization.

    Note: Only the differential of the position data is fused.
          This is because isaac_ros_visual slam currently only provides covariance
          data based on the last 10 position estimates
    Note: The cmd_vel data is used a input setpoints in the robot_localization ekf
    """
    isaac_ros_visual_slam_params_dir = os.path.join(
        get_package_share_directory('isaac_ros_visual_slam'), 'params')

    realsense_back_static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['-0.579136', '-0.075', '0.344389', '0',
                   '0', '1', '0', 'base_link_vslam_back', 'camera_back_link']
    )

    realsense_front_static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.115744', '0.075', '0.344389', '0',
                   '0', '0', '1', 'base_link_vslam_front', 'camera_front_link']
    )

    realsense_camera_back_node = Node(
        name='camera_back',
        namespace='camera_back',
        package='realsense2_camera',
        executable='realsense2_camera_node',
        parameters=[{
                'camera_name': 'camera_back',
                # Steps to determine serial number of realsense camera
                # Open realsenseviewer, add the second camera using the Add source button
                # Click on the Info button both cameras to view their serial numbers
                # Toggle depth camera streaming to determine if it is the front or back camera
                'serial_no': '213622252215',
                'enable_infra1': True,
                'enable_infra2': True,
                'enable_color': False,
                'enable_depth': False,
                'depth_module.emitter_enabled': 0,
                'depth_module.profile': '640x360x90'
        }]
    )

    realsense_camera_front_node = Node(
        name='camera_front',
        namespace='camera_front',
        package='realsense2_camera',
        executable='realsense2_camera_node',
        parameters=[{
                'camera_name': 'camera_front',
                # Steps to determine serial number of realsense camera
                # Open realsenseviewer, add the second camera using the Add source button
                # Click on the Info button both cameras to view their serial numbers
                # Toggle depth camera streaming to determine if it is the front or back camera
                'serial_no': '151422251695',
                'enable_infra1': True,
                'enable_infra2': True,
                'enable_color': False,
                'enable_depth': False,
                'depth_module.emitter_enabled': 0,
                'depth_module.profile': '640x360x90'
        }]
    )

    visual_slam_back_node = ComposableNode(
        name='visual_slam_back_node',
        namespace='visual_slam_back',
        package='isaac_ros_visual_slam',
        plugin='isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
                    'enable_rectified_pose': True,
                    'denoise_input_images': False,
                    'rectified_images': True,
                    'enable_debug_mode': False,
                    'debug_dump_path': '/tmp/cuvslam',
                    'enable_slam_visualization': True,
                    'enable_landmarks_view': True,
                    'enable_observations_view': True,
                    'map_frame': 'map',
                    'odom_frame': 'odom',
                    'base_frame': 'base_link_vslam_back',
                    'input_base_frame': 'base_link_vslam_back',
                    'input_left_camera_frame': 'camera_back_infra1_frame',
                    'input_right_camera_frame': 'camera_back_infra2_frame',
                    'publish_map_to_odom_tf': False
                    }],
        remappings=[('stereo_camera/left/image', '/camera_back/infra1/image_rect_raw'),
                    ('stereo_camera/left/camera_info',
                     '/camera_back/infra1/camera_info'),
                    ('stereo_camera/right/image',
                     '/camera_back/infra2/image_rect_raw'),
                    ('stereo_camera/right/camera_info', '/camera_back/infra2/camera_info')]
    )

    visual_slam_front_node = ComposableNode(
        name='visual_slam_front_node',
        namespace='visual_slam_front',
        package='isaac_ros_visual_slam',
        plugin='isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
                    'enable_rectified_pose': True,
                    'denoise_input_images': False,
                    'rectified_images': True,
                    'enable_debug_mode': False,
                    'debug_dump_path': '/tmp/elbrus',
                    'enable_slam_visualization': True,
                    'enable_landmarks_view': True,
                    'enable_observations_view': True,
                    'map_frame': 'map',
                    'odom_frame': 'odom',
                    'base_frame': 'base_link_vslam_front',
                    'input_base_frame': 'base_link_vslam_front',
                    'input_left_camera_frame': 'camera_front_infra1_frame',
                    'input_right_camera_frame': 'camera_front_infra2_frame',
                    'publish_map_to_odom_tf': False
                    }],
        remappings=[('stereo_camera/left/image', '/camera_front/infra1/image_rect_raw'),
                    ('stereo_camera/left/camera_info',
                     '/camera_front/infra1/camera_info'),
                    ('stereo_camera/right/image',
                     '/camera_front/infra2/image_rect_raw'),
                    ('stereo_camera/right/camera_info', '/camera_front/infra2/camera_info')]
    )

    visual_slam_back_launch_container = ComposableNodeContainer(
        name='visual_slam_back_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            visual_slam_back_node
        ],
        output='screen'
    )

    visual_slam_front_launch_container = ComposableNodeContainer(
        name='visual_slam_front_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            visual_slam_front_node
        ],
        output='screen'
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(
            isaac_ros_visual_slam_params_dir, 'ekf_dual_nvslam.yaml')],
        remappings=[('/odometry/filtered', '/odom')]
    )

    return launch.LaunchDescription([
                                    realsense_back_static_transform_publisher,
                                    realsense_front_static_transform_publisher,
                                    visual_slam_back_launch_container,
                                    visual_slam_front_launch_container,
                                    realsense_camera_back_node,
                                    realsense_camera_front_node,
                                    robot_localization_node
                                    ])
