# Copyright (c) 2021-2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Launch file which brings up visual slam node configured for Isaac Sim."""
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='isaac_ros::visual_slam::VisualSlamNode',
        remappings=[('/stereo_camera/left/camera_info', '/camera_info_left'),
                    ('/stereo_camera/right/camera_info', '/camera_info_right'),
                    ('/stereo_camera/left/image', '/rgb_left'),
                    ('/stereo_camera/right/image', '/rgb_right')],
        parameters=[{
                    'use_sim_time': True,
                    'denoise_input_images': True,
                    'rectified_images': True,
                    'enable_slam_visualization': True,
                    'enable_observations_view': True,
                    'enable_landmarks_view': True,
                    'enable_debug_mode': False,
                    'debug_dump_path': '/tmp/elbrus',
                    'map_frame': 'map',
                    'odom_frame': 'odom',
                    'base_frame': 'base_link'
                    }]

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

    return launch.LaunchDescription([visual_slam_launch_container])
