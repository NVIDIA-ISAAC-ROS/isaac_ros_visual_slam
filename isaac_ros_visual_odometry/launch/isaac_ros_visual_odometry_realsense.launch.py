# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

import launch
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Launch file which brings up visual odometry node configured for RealSense."""
    realsense_camera_node = Node(
        package='realsense2_camera',
        node_executable='realsense2_camera_node',
        parameters=[{
                'infra_height': 360,
                'infra_width': 640,
                'enable_color': False,
                'enable_depth': False,
                'emitter_enabled': False,
                'infra_fps': 90.0
        }],
        remappings=[('/infra1/image_rect_raw', 'stereo_camera/left/image'),
                    ('/infra1/camera_info', 'stereo_camera/left/camera_info'),
                    ('/infra2/image_rect_raw', 'stereo_camera/right/image'),
                    ('/infra2/camera_info', 'stereo_camera/right/camera_info')])

    visual_odometry_node = ComposableNode(
        name='visual_odometry_node',
        package='isaac_ros_visual_odometry',
        plugin='isaac_ros::visual_odometry::VisualOdometryNode',
        parameters=[{
                    'enable_rectified_pose': True,
                    'denoise_input_images': False,
                    'rectified_images': True,
                    'enable_debug_mode': False,
                    'debug_dump_path': '/tmp/elbrus',
                    'left_camera_frame': 'camera_infra1_frame',
                    'right_camera_frame': 'camera_infra2_frame',
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
            visual_odometry_node
        ],
        output='screen'
    )

    return launch.LaunchDescription([visual_odometry_launch_container, realsense_camera_node])
