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
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from nav_msgs.msg import Odometry

import pytest

import rclpy

_TEST_CASE_NAMESPACE = 'visual_slam_test_pol_multi_cam'


def create_camera_remapping(camera: str, identifier: str, idx: int):
    image_topic_src = f'/{camera}_stereo_camera/{identifier}/image_compressed'
    image_topic_dst = IsaacROSVisualSlamTest.generate_namespace(
        _TEST_CASE_NAMESPACE) + image_topic_src

    info_topic_src = f'/{camera}_stereo_camera/{identifier}/camera_info'
    info_topic_dst = IsaacROSVisualSlamTest.generate_namespace(
        _TEST_CASE_NAMESPACE) + info_topic_src

    return [f'{image_topic_src}:={image_topic_dst}', f'{info_topic_src}:={info_topic_dst}']


def create_imu_remapping():
    imu_topic_src = '/front_stereo_imu/imu'
    imu_topic_dst = IsaacROSVisualSlamTest.generate_namespace(_TEST_CASE_NAMESPACE) + imu_topic_src
    return [f'{imu_topic_src}:={imu_topic_dst}']


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
    nodes.append(create_decoder('back', 'left'))
    nodes.append(create_decoder('back', 'right'))

    nodes.append(
        ComposableNode(
            name='visual_slam_node',
            package='isaac_ros_visual_slam',
            plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
            namespace=IsaacROSVisualSlamTest.generate_namespace(_TEST_CASE_NAMESPACE),
            parameters=[{
                'num_cameras': 4,
                'min_num_images': 4,
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
                ('visual_slam/image_2', 'back_stereo_camera/left/image_raw'),
                ('visual_slam/image_3', 'back_stereo_camera/right/image_raw'),
                ('visual_slam/camera_info_0', 'front_stereo_camera/left/camera_info'),
                ('visual_slam/camera_info_1', 'front_stereo_camera/right/camera_info'),
                ('visual_slam/camera_info_2', 'back_stereo_camera/left/camera_info'),
                ('visual_slam/camera_info_3', 'back_stereo_camera/right/camera_info'),
                ('visual_slam/imu', 'front_stereo_imu/imu'),
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
    cmd.extend(create_camera_remapping('back', 'left', 2))
    cmd.extend(create_camera_remapping('back', 'right', 3))
    cmd.extend(create_imu_remapping())
    rosbag_play = launch.actions.ExecuteProcess(cmd=cmd, output='screen')

    return IsaacROSVisualSlamTest.generate_test_description([container, rosbag_play])


class IsaacROSVisualSlamTest(IsaacROSBaseTest):
    filepath = pathlib.Path(os.path.dirname(__file__))

    @IsaacROSBaseTest.for_each_test_case('rosbags')
    def test_visual_slam_node(self, test_folder):
        TIMEOUT = 25
        received_messages = {}
        self.generate_namespace_lookup(['visual_slam/tracking/odometry'], _TEST_CASE_NAMESPACE)
        subs = self.create_logging_subscribers(
            [('visual_slam/tracking/odometry', Odometry)],
            received_messages,
            use_namespace_lookup=True,
            accept_multiple_messages=True,
        )

        try:
            end_time = time.time() + TIMEOUT
            done = False

            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.1)

            if len(received_messages['visual_slam/tracking/odometry']) > 0:
                done = True

            self.assertTrue(done, 'Didnt receive output on visual_slam/tracking/odometry topic')

        finally:
            [self.node.destroy_subscription(sub) for sub in subs]
