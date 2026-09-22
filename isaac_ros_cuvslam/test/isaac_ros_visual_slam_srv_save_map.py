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

import os
import pathlib
import sys
import uuid

from isaac_ros_test import IsaacROSBaseTest
from isaac_ros_visual_slam_interfaces.srv import FilePath
import pytest
import rclpy

sys.path.append(os.path.dirname(__file__))
from helpers import run_cuvslam_from_bag, wait_for_odometry_message  # noqa: I100 E402

_TEST_CASE_NAMESPACE = '/visual_slam_test_srv_save_map'


@pytest.mark.rostest
def generate_test_description():
    bag_path = pathlib.Path(__file__).parent / 'test_cases/rosbags/r2b_galileo'
    return run_cuvslam_from_bag(_TEST_CASE_NAMESPACE, bag_path)


class IsaacRosVisualSlamServiceTest(IsaacROSBaseTest):
    """This test checks the functionality of the `visual_slam/save_map` service."""

    def test_save_map_service(self):
        self.assertTrue(wait_for_odometry_message(self.node, _TEST_CASE_NAMESPACE))

        service_client = self.node.create_client(
            FilePath,
            f'{_TEST_CASE_NAMESPACE}/visual_slam/save_map',
        )
        self.assertTrue(service_client.wait_for_service(timeout_sec=20))

        map_path = pathlib.Path(f'/tmp/cuvslam_map_{uuid.uuid4().hex.lower()}')
        request = FilePath.Request()
        request.file_path = str(map_path)

        response_future = service_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, response_future)
        response = response_future.result()
        self.assertTrue(response.success)

        self.assertTrue(map_path.exists())
