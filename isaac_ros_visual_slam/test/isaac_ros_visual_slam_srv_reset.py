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

_TEST_CASE_NAMESPACE = 'visual_slam_test_srv_reset'


def create_camera_remapping(camera: str, identifier: str, idx: int):
    image_topic_src = f'/{camera}_stereo_camera/{identifier}/image_{idx}'
    image_topic_dst = IsaacROSVisualSlamTest.generate_namespace(
        _TEST_CASE_NAMESPACE) + f'/visual_slam/image_{idx}'

    info_topic_src = f'/{camera}/{identifier}/camera_info_{idx}'
    info_topic_dst = IsaacROSVisualSlamTest.generate_namespace(
        _TEST_CASE_NAMESPACE) + f'/visual_slam/camera_info_{idx}'

    return [f'{image_topic_src}:={image_topic_dst}', f'{info_topic_src}:={info_topic_dst}']


def create_decoder(camera: str, identifier: str):
    return ComposableNode(
        name=f'{camera}_{identifier}_decoder_node',
        package='isaac_ros_h264_decoder',
        plugin='nvidia::isaac_ros::h264_decoder::DecoderNode',
        namespace=IsaacROSVisualSlamTest.generate_namespace(_TEST_CASE_NAMESPACE) +
        f'/{camera}_stereo_camera/{identifier}',
        remappings=[
            ('image_uncompressed', 'image_raw'),
        ],
    )


@pytest.mark.rostest
def generate_test_description():
    nodes = []
    nodes.append(create_decoder('front', 'left'))
    nodes.append(create_decoder('front', 'right'))

    nodes.append(
        ComposableNode(
            name='visual_slam_node',
            package='isaac_ros_visual_slam',
            plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
            namespace=IsaacROSVisualSlamTest.generate_namespace(_TEST_CASE_NAMESPACE),
            parameters=[{
                'num_cameras': 2,
                'min_num_images': 2,
                'enable_image_denoising': False,
                'rectified_images': False,
                'enable_localization_n_mapping': False,
                'enable_imu_fusion': False,
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
            }],
            remappings=[
                ('visual_slam/image_0', 'front_stereo_camera/left/image_raw'),
                ('visual_slam/image_1', 'front_stereo_camera/right/image_raw'),
                ('visual_slam/camera_info_0', 'front_stereo_camera/left/camera_info'),
                ('visual_slam/camera_info_1', 'front_stereo_camera/right/camera_info'),
            ],
        ))

    container = ComposableNodeContainer(
        name='visual_slam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=nodes,
        output='screen',
    )

    cmd = [
        'ros2', 'bag', 'play',
        os.path.dirname(__file__) + '/test_cases/rosbags/r2b_galileo', '--remap'
    ]
    cmd.extend(create_camera_remapping('front', 'left', 0))
    cmd.extend(create_camera_remapping('front', 'right', 1))
    rosbag_play = launch.actions.ExecuteProcess(cmd=cmd, output='screen')

    return IsaacROSVisualSlamTest.generate_test_description([container, rosbag_play])


class IsaacROSVisualSlamTest(IsaacROSBaseTest):
    filepath = pathlib.Path(os.path.dirname(__file__))

    @IsaacROSBaseTest.for_each_test_case('rosbags')
    def test_visual_slam_node(self, test_folder):
        TIMEOUT = 10
        END_TIME = time.time() + 2 * TIMEOUT
        received_messages = {}
        self.generate_namespace_lookup(['visual_slam/tracking/odometry'], _TEST_CASE_NAMESPACE)
        subs = self.create_logging_subscribers(
            [('visual_slam/tracking/odometry', Odometry)],
            received_messages,
            use_namespace_lookup=True,
            accept_multiple_messages=True,
        )

        # Create service client
        self.cli = self.node.create_client(
            Reset, '/isaac_ros_test/visual_slam_test_srv_reset/visual_slam/reset')
        # Check if the service is available and wait for service till 2 times the TIMEOUT
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.assertLess(time.time(), END_TIME,
                            'Timeout occurred while waiting for the reset service')
            self.node.get_logger().info('service not available, waiting again...')
        self.req = Reset.Request()

        try:
            done = False

            time.sleep(TIMEOUT)

            self.future = self.cli.call_async(self.req)
            rclpy.spin_until_future_complete(self.node, self.future, timeout_sec=TIMEOUT)
            response = self.future.result()
            self.assertNotEqual(
                response, None,
                'Timeout occurred while waiting for a response from reset service.')

            msg = f'Reset service response (success:{response.success})'
            self.node.get_logger().info(msg)

            if response.success:
                done = True

            self.assertTrue(done, ' service did not run successfully')

        finally:
            [self.node.destroy_subscription(sub) for sub in subs]
