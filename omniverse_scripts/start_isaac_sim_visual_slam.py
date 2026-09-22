# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import argparse
import os
import random
import time

import omni

ADDITIONAL_EXTENSIONS_BASE = ['omni.isaac.ros2_bridge-humble']


def enable_extensions_for_sim():
    """
    Enable the required extensions for the simulation.

    """
    from omni.isaac.core.utils.extensions import enable_extension
    add_exten = ADDITIONAL_EXTENSIONS_BASE
    for idx in range(len(add_exten)):
        enable_extension(add_exten[idx])


def setting_camera():
    """
    camera setting 

    """
    from pxr import Sdf, Gf
    import omni.kit.commands
    print('Setting camera')
    omni.kit.commands.execute(
    "ChangeProperty", prop_path=Sdf.Path(
        f"/World/Carter_ROS/ROS_Cameras/enable_camera_right.inputs:condition"),
    value=True, prev=None)
    omni.kit.commands.execute(
    "ChangeProperty", prop_path=Sdf.Path(
        f"/World/Carter_ROS/ROS_Cameras/ros2_create_camera_right_info.inputs:stereoOffset"),
    value=Gf.Vec2f([-175.92, 0.0]), prev=None)

    print('finish setting camera.')


def main(scenario_path: str,
         environment_prim_path: str,
         tick_rate_hz: float = 20.0):

    # Start up the simulator
    from omni.isaac.kit import SimulationApp
    simulation_app = SimulationApp({
        'renderer': 'RayTracedLighting',
        'headless': False 
    })

    import omni.kit.commands
    from omni.isaac.core import SimulationContext
    from omni.isaac.core.utils.nucleus import get_assets_root_path

    # Enables the simulation extensions
    enable_extensions_for_sim()

    assets_root_path = get_assets_root_path()
    if assets_root_path is None:
        print(
            'Could not find Isaac Sim assets folder. Make sure you have an up to date local \
             Nucleus server or that you have a proper connection to the internet.'
        )
        simulation_app.close()
        exit()

    usd_path = assets_root_path + scenario_path
    print(f'Opening stage {usd_path}...')

    # Load the stage
    omni.usd.get_context().open_stage(usd_path, None)

    # Wait two frames so that stage starts loading
    simulation_app.update()
    simulation_app.update()

    print('Loading stage...')
    from omni.isaac.core.utils.stage import is_stage_loading

    while is_stage_loading():
        simulation_app.update()
    print('Loading Complete')

    setting_camera()
    time_dt = 1.0 / tick_rate_hz
    print(f'Running sim at {tick_rate_hz} Hz, with dt of {time_dt}')
    # Run physics at 60 Hz and render time at the set frequency to see the sim as real time
    simulation_context = SimulationContext(stage_units_in_meters=1.0,
                                           physics_dt=1.0 / 60,
                                           rendering_dt=time_dt)

    simulation_context.play()
    simulation_context.step()

    # Simulate for a few seconds to warm up sim and let everything settle
    for _ in range(2*round(tick_rate_hz)):
        simulation_context.step()

    # Run the sim
    last_frame_time = time.monotonic()
    while simulation_app.is_running():
        simulation_context.step()
        current_frame_time = time.monotonic()
        if current_frame_time - last_frame_time < time_dt:
            time.sleep(time_dt - (current_frame_time - last_frame_time))
        last_frame_time = time.monotonic()

    simulation_context.stop()
    simulation_app.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Sample app for running Carter in a Warehouse for NVblox.')
    parser.add_argument(
        '--scenario_path',
        help='Path of the scenario to launch relative to the nucleus server '
        'base path. Scenario must contain a carter robot. If the scene '
        'contains animated humans, the script expects to find them under /World/Humans. '
        'Example scenarios are /Isaac/Samples/NvBlox/carter_warehouse_navigation_with_people.usd '
        'or /Isaac/Samples/NvBlox/carter_warehouse_navigation.usd',
        default='/Isaac/Samples/ROS2/Scenario/carter_warehouse_apriltags_worker.usd')
    parser.add_argument('--environment_prim_path',
                        default='/World/WareHouse',
                        help='Path to the world to create a navigation mesh.')
    parser.add_argument(
        '--tick_rate_hz',
        type=int,
        help='The rate (in hz) that we step the simulation at.',
        default=20)
    # This allows for IsaacSim options to be passed on the SimulationApp.
    args, unknown = parser.parse_known_args()

    main(args.scenario_path, args.environment_prim_path,
         args.tick_rate_hz)
