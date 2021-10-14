# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
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
    """Launch file which brings up visual odometry node configured for Isaac Sim."""
    left_image_format_converter_node = ComposableNode(
        name='left_image_format_converter_node',
        package='isaac_ros_image_proc',
        plugin='isaac_ros::image_proc::ImageFormatConverterNode',
        remappings=[('image_raw', '/rgb_left'),
                    ('image', 'stereo_camera/left/image')],
        parameters=[{'encoding_desired': 'mono8'}]
    )

    right_image_format_converter_node = ComposableNode(
        name='right_image_format_converter_node',
        package='isaac_ros_image_proc',
        plugin='isaac_ros::image_proc::ImageFormatConverterNode',
        remappings=[('image_raw', '/rgb_right'),
                    ('image', 'stereo_camera/right/image')],
        parameters=[{'encoding_desired': 'mono8'}]
    )

    visual_odometry_node = ComposableNode(
        name='visual_odometry_node',
        package='isaac_ros_visual_odometry',
        plugin='isaac_ros::visual_odometry::VisualOdometryNode',
        remappings=[('stereo_camera/left/camera_info', '/camera_info_left'),
                    ('stereo_camera/right/camera_info', '/camera_info_right')],
        parameters=[{
                    'enable_rectified_pose': True,
                    'denoise_input_images': True,
                    'rectified_images': True,
                    'enable_debug_mode': False,
                    'debug_dump_path': '/tmp/elbrus',
                    'left_camera_frame': 'carter_camera_stereo_left',
                    'right_camera_frame': 'carter_camera_stereo_right',
                    'fixed_frame': 'odom',
                    'current_smooth_frame': 'base_link_smooth',
                    'current_rectified_frame': 'base_link_rectified'
                    }]
    )

    visual_odometry_launch_container = ComposableNodeContainer(
        name='visual_odometry_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            visual_odometry_node,
            left_image_format_converter_node,
            right_image_format_converter_node
        ],
        output='screen'
    )

    return launch.LaunchDescription([visual_odometry_launch_container])
