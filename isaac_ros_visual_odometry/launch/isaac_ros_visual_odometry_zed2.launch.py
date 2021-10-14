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
    """Launch file which brings up visual odometry node remapped for Zed2."""
    visual_odometry_node = ComposableNode(
        name='visual_odometry_node',
        package='isaac_ros_visual_odometry',
        plugin='isaac_ros::visual_odometry::VisualOdometryNode',
        remappings=[('stereo_camera/left/camera_info', '/zed2/zed_node/left/camera_info'),
                    ('stereo_camera/right/camera_info', '/zed2/zed_node/right/camera_info'),
                    ('stereo_camera/left/image', '/zed2/zed_node/left/image_rect_gray'),
                    ('stereo_camera/right/image', '/zed2/zed_node/right/image_rect_gray'),
                    ('visual_odometry/imu', '/zed2/zed_node/imu/data')],
        # namespace='/visual_odometry',
        parameters=[{
                    'enable_rectified_pose': True,
                    'denoise_input_images': False,
                    'rectified_images': True,
                    'enable_debug_mode': False,
                    'enable_imu': True,
                    'debug_dump_path': '/tmp/elbrus',
                    'left_camera_frame': 'zed2_left_camera_frame',
                    'right_camera_frame': 'zed2_right_camera_frame',
                    'imu_frame': 'zed2_imu_link',
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

    return launch.LaunchDescription([visual_odometry_launch_container])
