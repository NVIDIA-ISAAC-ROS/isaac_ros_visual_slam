# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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
import pathlib
import time
from typing import Any

import isaac_ros_launch_utils.all_types as lut
import launch_testing.actions
from nav_msgs.msg import Odometry
import rclpy.node


def create_camera_remapping(namespace: str, camera: str, identifier: str, idx: int) -> list[str]:
    """Remap the camera topics from a recording to prepend a namespace."""
    image_topic_src = f'/{camera}_stereo_camera/{identifier}/image_compressed'
    image_topic_dst = namespace + image_topic_src

    info_topic_src = f'/{camera}_stereo_camera/{identifier}/camera_info'
    info_topic_dst = namespace + info_topic_src

    return [f'{image_topic_src}:={image_topic_dst}', f'{info_topic_src}:={info_topic_dst}']


def create_imu_remapping(namespace: str) -> list[str]:
    """Remap the imu topic from a recording to prepend a namespace."""
    imu_topic_src = '/front_stereo_imu/imu'
    imu_topic_dst = namespace + imu_topic_src
    return [f'{imu_topic_src}:={imu_topic_dst}']


def create_decoder(namespace: str, camera: str, identifier: str) -> lut.ComposableNode:
    return lut.ComposableNode(
        name=f'{camera}_{identifier}_decoder_node',
        package='isaac_ros_h264_decoder',
        plugin='nvidia::isaac_ros::h264_decoder::DecoderNode',
        namespace=f'{namespace}/{camera}_stereo_camera/{identifier}',
        remappings=[('image_uncompressed', 'image_raw')],
    )


def run_cuvslam_from_bag(namespace: str,
                         bag_path: pathlib.Path,
                         vslam_override_parameters: dict = None,
                         cameras: list[str] = ['front']) -> list[lut.Action]:
    nodes = []
    for camera in cameras:
        nodes.append(create_decoder(namespace, camera, 'left'))
        nodes.append(create_decoder(namespace, camera, 'right'))

    vslam_parameters = {
        'num_cameras': len(cameras) * 2,
        'min_num_images': len(cameras) * 2,
        'enable_image_denoising': False,
        'rectified_images': False,
        'enable_localization_n_mapping': True,
        'enable_imu_fusion': True,
        'map_frame': 'map',
        'odom_frame': 'odom',
        'base_frame': 'base_link',
    }

    if vslam_override_parameters:
        vslam_parameters = vslam_parameters | vslam_override_parameters

    remappings = []
    for idx, camera in enumerate(cameras):
        remappings.extend([
            (f'visual_slam/image_{idx*2}', f'{camera}_stereo_camera/left/image_raw'),
            (f'visual_slam/image_{idx*2+1}', f'{camera}_stereo_camera/right/image_raw'),
            (f'visual_slam/camera_info_{idx*2}', f'{camera}_stereo_camera/left/camera_info'),
            (f'visual_slam/camera_info_{idx*2+1}', f'{camera}_stereo_camera/right/camera_info'),
        ])
    remappings.append(('visual_slam/imu', 'front_stereo_imu/imu'))

    nodes.append(
        lut.ComposableNode(
            name='visual_slam_node',
            package='isaac_ros_visual_slam',
            plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
            namespace=namespace,
            parameters=[vslam_parameters],
            remappings=remappings,
        ))

    container = lut.ComposableNodeContainer(
        name='visual_slam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=nodes,
        output='screen',
    )

    cmd = ['ros2', 'bag', 'play', str(bag_path), '--remap']
    for idx, camera in enumerate(cameras):
        cmd.extend(create_camera_remapping(namespace, camera, 'left', idx * 2))
        cmd.extend(create_camera_remapping(namespace, camera, 'right', idx * 2 + 1))
    cmd.extend(create_imu_remapping(namespace))
    rosbag_play = lut.ExecuteProcess(cmd=cmd, output='screen', on_exit=lut.Shutdown())

    ready_to_test = lut.TimerAction(
        period=2.0,
        actions=[launch_testing.actions.ReadyToTest()],
    )

    return lut.LaunchDescription([
        container,
        rosbag_play,
        ready_to_test,
    ])


def wait_for_one_message(node: rclpy.node.Node, message_type: Any, topic: str,
                         timeout_s: float) -> Any | None:
    """Wait until a single message is received."""
    received_messages = []

    def callback(message: any):
        received_messages.append(message)

    end_time = time.time() + timeout_s
    subscription = node.create_subscription(
        message_type,
        topic,
        callback,
        10,
    )

    while time.time() < end_time and not received_messages:
        SPIN_TIMEOUT = 0.1
        rclpy.spin_once(node, timeout_sec=SPIN_TIMEOUT)

    node.destroy_subscription(subscription)
    return received_messages[0] if received_messages else None


def wait_for_odometry_message(node: rclpy.node.Node,
                              namespace: str,
                              timeout_s: float = 20.0) -> bool:
    """Wait until a single odometry message was received from cuvslam."""
    return bool(
        wait_for_one_message(node, Odometry, f'{namespace}/visual_slam/tracking/odometry',
                             timeout_s))
