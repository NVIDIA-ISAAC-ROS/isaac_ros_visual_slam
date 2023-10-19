# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
import pathlib
import time

from isaac_ros_test import IsaacROSBaseTest
from isaac_ros_visual_slam_interfaces.srv import Reset
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from nav_msgs.msg import Odometry

import pytest

import rclpy

_TEST_CASE_NAMESPACE = 'visual_slam_test_srv'


@pytest.mark.rostest
def generate_test_description():
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        namespace=IsaacROSVisualSlamTest.generate_namespace(
            _TEST_CASE_NAMESPACE),
        parameters=[{
            'denoise_input_images': False,
        }]
    )

    container = ComposableNodeContainer(
        name='visual_slam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[visual_slam_node],
        output='screen'
    )

    rosbag_play = launch.actions.ExecuteProcess(
        cmd=['ros2', 'bag', 'play', os.path.dirname(__file__) +
             '/test_cases/rosbags/small_pol_test',
             '--remap',
             '/stereo_camera/left/image:=' +
             IsaacROSVisualSlamTest.generate_namespace(
             _TEST_CASE_NAMESPACE) + '/stereo_camera/left/image',
             '/camera_info_left:=' +
             IsaacROSVisualSlamTest.generate_namespace(
             _TEST_CASE_NAMESPACE) + '/stereo_camera/left/camera_info',
             '/stereo_camera/right/image:=' +
             IsaacROSVisualSlamTest.generate_namespace(
             _TEST_CASE_NAMESPACE) + '/stereo_camera/right/image',
             '/camera_info_right:=' +
             IsaacROSVisualSlamTest.generate_namespace(
             _TEST_CASE_NAMESPACE) + '/stereo_camera/right/camera_info'],
        output='screen'
    )

    return IsaacROSVisualSlamTest.generate_test_description(
        [container, rosbag_play])


class IsaacROSVisualSlamTest(IsaacROSBaseTest):
    filepath = pathlib.Path(os.path.dirname(__file__))

    @ IsaacROSBaseTest.for_each_test_case('rosbags')
    def test_visual_slam_node(self, test_folder):
        TIMEOUT = 10
        received_messages = {}
        self.generate_namespace_lookup(['visual_slam/tracking/odometry'], _TEST_CASE_NAMESPACE)
        subs = self.create_logging_subscribers(
            [('visual_slam/tracking/odometry', Odometry)], received_messages,
            use_namespace_lookup=True, accept_multiple_messages=True)

        # Create service client
        self.cli = self.node.create_client(
            Reset, '/isaac_ros_test/visual_slam_test_srv/visual_slam/reset')
        # Check if the a service is available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')
        self.req = Reset.Request()

        try:
            done = False

            time.sleep(TIMEOUT)

            self.future = self.cli.call_async(self.req)
            rclpy.spin_until_future_complete(self.node, self.future)
            response = self.future.result()
            msg = f'Reset service response (success:{response.success})'
            self.node.get_logger().info(msg)

            if response.success:
                done = True

            self.assertTrue(
                done, ' service did not run successfully')

        finally:
            [self.node.destroy_subscription(sub) for sub in subs]
