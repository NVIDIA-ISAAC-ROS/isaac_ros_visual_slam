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

from isaac_ros_test import IsaacROSBaseTest
import pytest

sys.path.append(os.path.dirname(__file__))
from helpers import run_cuvslam_from_bag, wait_for_odometry_message  # noqa: I100 E402

_TEST_CASE_NAMESPACE = '/visual_slam_test_pol_multi_cam'


@pytest.mark.rostest
def generate_test_description():
    bag_path = pathlib.Path(__file__).parent / 'test_cases/rosbags/r2b_galileo'
    cameras = ['front', 'back']
    description = run_cuvslam_from_bag(_TEST_CASE_NAMESPACE, bag_path, cameras=cameras)
    return description


class IsaacRosVisualSlamServiceTest(IsaacROSBaseTest):
    """This test checks that vslam works with multiple cameras."""

    def test_multi_cam_pol(self):
        self.assertTrue(wait_for_odometry_message(self.node, _TEST_CASE_NAMESPACE))
