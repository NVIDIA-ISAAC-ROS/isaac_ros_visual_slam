# Tutorial for Visual SLAM using a Depthai OAK Camera


## Overview

This tutorial walks you through setting up [Isaac ROS Visual SLAM](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam) with a [Luxonis OAK camera](https://www.luxonis.com/).


## Tutorial Walkthrough - VSLAM execution

Most of the steps overlap with [Realsense setup](./tutorial-realsense.md). Main differences include setting up Docker image for use with the cameras.

1. Clone this repository and its dependencies under `${ISAAC_ROS_WS}/src`.

    ```bash
    cd ${ISAAC_ROS_WS}/src
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros
    ```
2. Clone `depthai` and `xacro` repositories to the workspace

    ```bash
    cd ${ISAAC_ROS_WS}/src
    git clone --branch ros-release https://github.com/luxonis/depthai-core
    git clone --branch humble https://github.com/luxonis/depthai-ros
    git clone --branch ros2 https://github.com/ros/xacro
    ```
3. Setup udev rules on host

    ```bash
    echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
    sudo udevadm control --reload-rules && sudo udevadm trigger
    ```

4. Setup configuration for docker creation

    ```bash
    cd ${ISAAC_ROS_WS}/src/isaac_ros_common/scripts && \
    touch .isaac_ros_common-config && \
    echo CONFIG_IMAGE_KEY=ros2_humble.depthai > .isaac_ros_common-config && \
    touch .isaac_ros_dev-dockerargs && \
    echo "-v /dev/bus/usb:/dev/bus/usb" > .isaac_ros_dev-dockerargs
    ```
5. Run the docker image

    ```bash
        isaac_ros_container
    ```

    > Or if you did not add the command in [step 1-3 of the quickstart section](../README.md#quickstart):
    >
    > ```bash
    > cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
    >   ./scripts/run_dev.sh ${ISAAC_ROS_WS}
    > ```

5. When inside the container, build the workspace:

    ```bash
        cd /workspaces/isaac_ros-dev && \
      colcon build --symlink-install && \
      source install/setup.bash
    ```

6. Run launch file for the camera

    ```bash
        ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_depthai.launch.py
    ```

7. `depthai-ros-driver` allows for a large amount of customization using parameters. Example parameters can be found in `isaac_ros_visual_slam/params.depthai.yaml`