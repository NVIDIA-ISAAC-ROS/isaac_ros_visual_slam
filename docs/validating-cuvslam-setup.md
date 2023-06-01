# Validating Isaac ROS Visual SLAM on a Robot

## Overview

This document outlines a procedure that can be used to validate the successful installation and configuration of Isaac ROS Visual SLAM on real hardware.

## Table of Contents
- [Validating Isaac ROS Visual SLAM on a Robot](#validating-isaac-ros-visual-slam-on-a-robot)
  - [Overview](#overview)
  - [Table of Contents](#table-of-contents)
  - [Prerequisites](#prerequisites)
  - [Prepare Physical Environment](#prepare-physical-environment)
  - [Download and Configure Packages](#download-and-configure-packages)
  - [Build ROS Workspace](#build-ros-workspace)
  - [Select and Start ROS Visualization Tool](#select-and-start-ros-visualization-tool)
  - [Run Isaac ROS Visual SLAM and Monitor Results](#run-isaac-ros-visual-slam-and-monitor-results)
    - [Test 1: Visual Odometry](#test-1-visual-odometry)
    - [Test 2: Loop Closure](#test-2-loop-closure)
  - [Next Steps](#next-steps)
    - [Troubleshooting](#troubleshooting)
    - [Integrating Isaac ROS Visual SLAM](#integrating-isaac-ros-visual-slam)

## Prerequisites

- [ ] NVIDIA Jetson
- [ ] Standalone computer with ROS data visualization tool (RViz, Foxglove, etc.)
- [ ] Compatible stereo camera
- [ ] Enough space to navigate (~40m loop) 
- [ ] Tape measure 


## Prepare Physical Environment

1.  Examine the designated testing area and identify a single, non-intersecting loop whose path length is approximately 40 meters. This will be the path along which the camera is moved and tracked using Isaac ROS Visual SLAM.
2.  Define a starting pose along the identified loop. Label this pose with masking tape or a different marker. 
    
    One possible approach is to mark an L-shape on the floor, aligning the camera's position and orientation off of the corner of the shape.
3.  Using a measuring tool, measure out a secondary pose along the loop that is exactly 1 meter along the forward axis from the starting pose. Label this pose in the same manner. 

<div align="center"><img src="../resources/validation_tape.jpg" width="600px"/></div>

## Download and Configure Packages

1. On each of the Jetson and standalone computer, set up your development environment by following the instructions [here](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docs/dev-env-setup.md).

    > Note: `${ISAAC_ROS_WS}` is defined to point to either `/ssd/workspaces/isaac_ros-dev/` or `~/workspaces/isaac_ros-dev/`.

2. Set up your stereo camera and connect it to the Jetson.
  
   - For HAWK cameras, complete the [HAWK setup tutorial](https://github.com/NVIDIA-ISAAC-ROS/.github/blob/main/profile/hawk-setup.md).
   - For RealSense cameras, complete the [RealSense setup tutorial](https://github.com/NVIDIA-ISAAC-ROS/.github/blob/main/profile/realsense-setup.md).

3. On both devices, clone this repository and its dependencies under `${ISAAC_ROS_WS}/src`.

    ```bash
    cd ${ISAAC_ROS_WS}/src && \
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam && \
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common && \
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros
    ```

## Build ROS Workspace

1. On the Jetson, launch the Docker container in a new terminal:

    ```bash
    cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
      ./scripts/run_dev.sh ${ISAAC_ROS_WS}
    ```

2. Inside the container, build and source the workspace:

    ```bash
    cd /workspaces/isaac_ros-dev && \
      colcon build --symlink-install && \
      source install/setup.bash
    ```

## Select and Start ROS Visualization Tool

1. Ensure that the standalone computer and Jetson device are on the same network.
2. On the standalone computer, start RViz from the provided configuration file:

    Start the Docker container for RViz on the standalone computer:

    ```bash
    cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
      ./scripts/run_dev.sh ${ISAAC_ROS_WS}
    ```

    > **(Optional)** Configure the `ROS_DOMAIN_ID` environment variable on both devices to eliminate any cross-talk from other ROS nodes on the same network.
    > 
    > In each terminal on each device:
    > ```bash
    > export ROS_DOMAIN_ID=5 # Any number between 1-101; use the same number for each terminal
    > ```
    > **Note**: This environment variable must be set in each terminal, on both the Jetson and the standalone computer. 

    From this terminal, run Rviz2 to display the output:

    ```bash
    source /workspaces/isaac_ros-dev/install/setup.bash && \
      rviz2 -d src/isaac_ros_visual_slam/isaac_ros_visual_slam/rviz/default.cfg.rviz 
    ```

3. On the standalone computer, start logging the estimated transform:

    Attach another terminal to the running container for `tf2_echo`:

    ```bash
    cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
      ./scripts/run_dev.sh ${ISAAC_ROS_WS}
    ```

    From this terminal, run `tf2_echo` to display the transform between the `/map` frame and the appropriate camera frame:

    - HAWK: `/camera`
    - RealSense: `/camera_link`

    ```bash
    source /workspaces/isaac_ros-dev/install/setup.bash && \
      ros2 run tf2_ros tf2_echo map camera_link
    ```

## Run Isaac ROS Visual SLAM and Monitor Results

### Test 1: Visual Odometry

The purpose of this test is to validate that the system is able to provide accurate measurements of the robot's local motion.

1. Initialize the camera at the marked starting pose. 
2. On the Jetson, run the Isaac ROS Visual SLAM launch file appropriate for your camera:

    - HAWK:
      ```bash
      ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_hawk.launch.py
      ```
    - RealSense:
      ```bash
      ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_realsense.launch.py
      ```

    At this point, the `/map`, `/odom`, and camera frames will all be initialized to the starting pose.
3. Precisely move the camera straight forward to the marked secondary pose.

4. On the standalone computer and within the `tf2_echo` terminal, note the latest transform reported between `/map` and the appropriate camera frame. 

   <div align="center"><img src="../resources/validation_test1_tf2.png" width="600px"/></div>

   1. Validate Orientation:
   
      The orientation error between the measured rotation unit quaternion and the expected rotation unit quaternion (`[0 0 0 1]`) should be less than 15 degrees. This error can be calculated using the process explained [here](https://gitlab.tudelft.nl/-/snippets/190):

      $$q_{measured} = w_m + x_m i + y_m j + z_m k$$
      $$q_{expected} = w_e + x_e i + y_e j + z_e k$$
      $$2 \arccos (w_m w_e + x_m x_e + y_m y_e + z_m z_e) \leq 15 \degree $$

   2. Validate Translation:

      The magnitude of the measured translation should be within 5 centimeters of the expected translation (1 meter):

      $$ 0.95 \text{m} \leq \|t_{measured}\| \leq 1.05 \text{m}$$ 

If all real-world measurements correspond to the reported estimates within the expected tolerance, the Visual Odometry test is a sucess. 

### Test 2: Loop Closure

The purpose of this test is to validate that the system is able to appropriately realign the estimated pose path to previously-seen features.

1. Initialize the camera at the marked starting pose. 
2. On the Jetson, run the Isaac ROS Visual SLAM launch file appropriate for your camera:

    - HAWK:
      ```bash
      ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_hawk.launch.py
      ```
    - RealSense:
      ```bash
      ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_realsense.launch.py
      ```

    At this point, the `/map`, `/odom`, and camera frames will all be initialized to the starting pose.

3. Precisely move the camera along the predetermined 40m path.

   > **Note**: To maximize the accuracy of this test, ensure that dynamic obstacles are removed from the robot's field of view. If a nontrivial dynamic obstacle is introduced while the robot is performing the loop, please restart this test.

4. As the camera nears the completion of its first lap, move the camera in a small circle of radius ~0.5m around the origin of the starting pose. This step helps ensure that the previously-seen visual features are recognized and matched.
5. Place the camera at rest, taking care to precisely align it with the floor marker indicating the starting pose.
6. On the standalone computer and within the RViz window, ensure that a nontrivial number of visual features now appear red. Additionally, ensure that the camera frame appears nearly coincident with the `/map` frame.

    > **Note**: It is acceptable for the `/odom` frame to jump around significantly with respect to the `/map` frame. For the purposes of this test, only the transform between the `/map` and camera frames is relevant. For additional information about the design intent behind these frames, please see [REP-0105](https://www.ros.org/reps/rep-0105.html).

    <div align="center"><img src="../resources/validation_test2_rviz.png" width="600px"/></div>
  
7. On the standalone computer and within the `tf2_echo` terminal, note the latest transform reported between `/map` and the appropriate camera frame. 
   <div align="center"><img src="../resources/validation_test2_tf2.png" width="600px"/></div>

   1. Validate Orientation:
   
      The orientation error between the measured rotation unit quaternion and the expected rotation unit quaternion (`[0 0 0 1]`, since the path is a closed loop) should be less than 15 degrees. This error can be calculated using the process explained [here](https://gitlab.tudelft.nl/-/snippets/190):

      $$q_{measured} = w_m + x_m i + y_m j + z_m k$$
      $$q_{expected} = w_e + x_e i + y_e j + z_e k$$
      $$2 \arccos (w_m w_e + x_m x_e + y_m y_e + z_m z_e) \leq 15 \degree $$

   2. Validate Translation:

      The magnitude of the measured translation should be within 2 meters (5% of the 40m path length) of the expected translation (0 meters, since the path is a closed loop):

      $$ 0 \text{m} \leq \|t_{measured}\| \leq 2 \text{m}$$
   
If all real-world measurements correspond to the reported estimates within the expected tolerance, the Loop Closure test is a sucess. 

## Next Steps

### Troubleshooting
If any step of the above tests did not succeed, then there may be a configuration error with your system. Please follow these [troubleshooting steps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam#troubleshooting).

If the prepared troubleshooting steps are unable to resolve the test failure, please contact your NVIDIA support representative and file an issue report with all relevant details.

### Integrating Isaac ROS Visual SLAM

While the Isaac ROS Visual SLAM node exposes a [multitude of interfaces](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam#isaac_ros_visual_slam), the core way to consume the node's output is through the reported transform between the `/map` and camera frames.

Based on your robot's precise specifications, you can set up a static transform publisher or URDF-based publisher that connects the camera frame to a relevant planning frame on the robot (typically named `/base_link`). Then, your navigation stack can use the transform between `/base_link` and `/map` to plan movements within the local space.

Consider further augmenting your robot's navigation stack with other Isaac ROS packages, such as [Isaac ROS Nvblox](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox) for 3D reconstruction.
