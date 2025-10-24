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

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Launch the DNN Image encoder, TensorRT node and UNet decoder node."""
    launch_args = [
        DeclareLaunchArgument(
            'engine_file_path',
            default_value='',
            description='The absolute file path to the TensorRT engine file')
    ]

    engine_file_path = LaunchConfiguration('engine_file_path')

    """Launch file which brings up visual slam node configured for RealSense."""
    realsense_camera_node = Node(
        name='camera',
        namespace='',
        package='realsense2_camera',
        executable='realsense2_camera_node',
        parameters=[{
            'enable_infra1': True,
            'enable_infra2': True,
            'enable_color': True,
            'enable_depth': False,
            'depth_module.emitter_enabled': 0,
            'depth_module.infra_profile': '640x360x30',
            'depth_module.profile': '640x360x30',  # For backwards compatibility
            'rgb_camera.color_profile': '640x360x30',
            'enable_gyro': False,
            'enable_accel': False,
        }],
    )

    # Parameters preconfigured for PeopleSemSegNet.
    container_name = 'segformer_vslam_container'
    encoder_dir = get_package_share_directory('isaac_ros_dnn_image_encoder')
    encoder_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(encoder_dir, 'launch', 'dnn_image_encoder.launch.py')]
        ),
        launch_arguments={
            'input_image_width': '640',
            'input_image_height': '360',
            'network_image_width': '512',
            'network_image_height':  '512',
            'enable_padding': 'True',
            'keep_aspect_ratio': 'False',
            'input_encoding': 'mono8',
            'encoding_desired': 'rgb8',
            'image_input_topic': '/camera/infra1/image_rect_raw',
            'camera_info_input_topic': '/camera/infra1/camera_info',
            'tensor_output_topic': '/tensor_pub',
            'attach_to_shared_component_container': 'True',
            'component_container_name': container_name,
        }.items(),
    )

    tensorrt_node = ComposableNode(
        name='tensor_rt_node',
        package='isaac_ros_tensor_rt',
        plugin='nvidia::isaac_ros::dnn_inference::TensorRTNode',
        parameters=[{
            'engine_file_path': engine_file_path,
            'input_tensor_names': ['input_tensor'],
            'input_binding_names': ['input'],
            'input_tensor_formats': ['nitros_tensor_list_nchw_rgb_f32'],
            'output_tensor_names': ['output_tensor'],
            'output_binding_names': ['output'],
            'output_tensor_formats': ['nitros_tensor_list_nchw_rgb_f32'],
            'force_engine_update': False
        }])

    segformer_decoder_node = ComposableNode(
        name='segformer_decoder_node',
        package='isaac_ros_unet',
        plugin='nvidia::isaac_ros::unet::UNetDecoderNode',
        parameters=[{
            'network_output_type': 'argmax',
            'color_palette': [0x556B2F, 0x800000, 0x008080, 0x000080, 0x9ACD32, 0xFF0000, 0xFF8C00,
                              0xFFD700, 0x00FF00, 0xBA55D3, 0x00FA9A, 0x00FFFF, 0x0000FF, 0xF08080,
                              0xFF00FF, 0x1E90FF, 0xDDA0DD, 0xFF1493, 0x87CEFA, 0xFFDEAD],
        }],
        remappings=[
            ('unet/colored_segmentation_mask', 'segformer/colored_segmentation_mask'),
            ('unet/raw_segmentation_mask', 'segformer/raw_segmentation_mask')
        ])

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            'num_input_masks': 1,
            'enable_image_denoising': False,
            'rectified_images': True,
            'enable_imu_fusion': False,
            'image_jitter_threshold_ms': 35.00,
            'base_frame': 'camera_link',
            'imu_frame': 'camera_gyro_optical_frame',
            'enable_slam_visualization': True,
            'enable_landmarks_view': True,
            'enable_observations_view': True,
            'camera_optical_frames': [
                'camera_infra1_optical_frame',
                'camera_infra2_optical_frame',
            ],
        }],
        remappings=[
            ('visual_slam/image_0', '/camera/infra1/image_rect_raw'),
            ('visual_slam/camera_info_0', '/camera/infra1/camera_info'),
            ('visual_slam/image_1', '/camera/infra2/image_rect_raw'),
            ('visual_slam/camera_info_1', '/camera/infra2/camera_info'),
            ('visual_slam/seg_mask_0', '/segformer/raw_segmentation_mask'),
        ],
    )

    container = ComposableNodeContainer(
        name=container_name,
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[tensorrt_node, segformer_decoder_node, visual_slam_node],
        output='screen'
    )

    final_launch_description = launch_args + [
        container,
        encoder_node_launch,
        realsense_camera_node
    ]

    return launch.LaunchDescription(final_launch_description)
