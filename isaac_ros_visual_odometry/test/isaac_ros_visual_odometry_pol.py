# Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.


import os
import pathlib
import time

from isaac_ros_test import IsaacROSBaseTest

from isaac_ros_visual_odometry_interfaces.msg import VisualOdometryTransformStamped  # NOLINT

import launch

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import pytest

import rclpy

_TEST_CASE_NAMESPACE = 'visual_odometry_test'


@pytest.mark.rostest
def generate_test_description():
    visual_odometry_node = ComposableNode(
        name='visual_odometry_node',
        package='isaac_ros_visual_odometry',
        plugin='isaac_ros::visual_odometry::VisualOdometryNode',
        namespace=IsaacROSVisualOdometryTest.generate_namespace(
            _TEST_CASE_NAMESPACE),
        parameters=[{
            'enable_slam': True,
            'denoise_input_images': False,
        }]
    )

    container = ComposableNodeContainer(
        name='visual_odometry_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[visual_odometry_node],
        output='screen'
    )

    rosbag_play = launch.actions.ExecuteProcess(
        cmd=['ros2', 'bag', 'play', os.path.dirname(__file__) +
             '/test_cases/rosbags/small_pol_test',
             '--remap',
             '/stereo_camera/left/image:=' +
             IsaacROSVisualOdometryTest.generate_namespace(
             _TEST_CASE_NAMESPACE) + '/stereo_camera/left/image',
             '/camera_info_left:=' +
             IsaacROSVisualOdometryTest.generate_namespace(
             _TEST_CASE_NAMESPACE) + '/stereo_camera/left/camera_info',
             '/stereo_camera/right/image:=' +
             IsaacROSVisualOdometryTest.generate_namespace(
             _TEST_CASE_NAMESPACE) + '/stereo_camera/right/image',
             '/camera_info_right:=' +
             IsaacROSVisualOdometryTest.generate_namespace(
             _TEST_CASE_NAMESPACE) + '/stereo_camera/right/camera_info'],
        output='screen'
    )

    return IsaacROSVisualOdometryTest.generate_test_description(
        [container, rosbag_play])


class IsaacROSVisualOdometryTest(IsaacROSBaseTest):
    filepath = pathlib.Path(os.path.dirname(__file__))

    @ IsaacROSBaseTest.for_each_test_case('rosbags')
    def test_visual_odometry_node(self, test_folder):
        TIMEOUT = 20
        received_messages = {}
        self.generate_namespace_lookup(['visual_odometry/tf_stamped'], _TEST_CASE_NAMESPACE)
        subs = self.create_logging_subscribers(
            [('visual_odometry/tf_stamped', VisualOdometryTransformStamped)], received_messages,
            use_namespace_lookup=True, accept_multiple_messages=True)

        try:
            end_time = time.time() + TIMEOUT
            done = False

            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.1)

            if len(received_messages['visual_odometry/tf_stamped']) > 0:
                done = True

            self.assertTrue(done, 'Didnt recieve output on visual_odometry/tf_stamped topic')

        finally:
            [self.node.destroy_subscription(sub) for sub in subs]
