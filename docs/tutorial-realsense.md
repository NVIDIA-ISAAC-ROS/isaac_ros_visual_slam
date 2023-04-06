# Tutorial for Visual SLAM using a RealSense camera with integrated IMU

<div align="center"><img src="../resources/realsense.gif" width="600px"/></div>

## Overview

This tutorial walks you through setting up [Isaac ROS Visual SLAM](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam) with a [Realsense camera](https://www.intel.com/content/www/us/en/architecture-and-technology/realsense-overview.html).

> **Note**: The [launch file](../isaac_ros_visual_slam/launch/isaac_ros_visual_slam_realsense.launch.py) provided in this tutorial is designed for a RealSense camera with integrated IMU. If you want to run this tutorial with a RealSense camera without an IMU (like RealSense D435), then change `enable_imu` param in the launch file to `False`.
<!-- Split blockquote -->
> **Note**: This tutorial requires a compatible RealSense camera from the list available [here](https://github.com/NVIDIA-ISAAC-ROS/.github/blob/main/profile/realsense-setup.md#camera-compatibility).

## Tutorial Walkthrough - VSLAM execution

1. Complete the [RealSense setup tutorial](https://github.com/NVIDIA-ISAAC-ROS/.github/blob/main/profile/realsense-setup.md).

2. Complete the [Quickstart section](../README.md#quickstart) in the main README.

3. \[Terminal 1\] Run `realsense-camera` node and `visual_slam` node

   Make sure you have your RealSense camera attached to the system, and then start the Isaac ROS container.

    ```bash
        isaac_ros_container
    ```

    > Or if you did not add the command in [step 1-3 of the quickstart section](../README.md#quickstart):
    >
    > ```bash
    > cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
    >   ./scripts/run_dev.sh ${ISAAC_ROS_WS}
    > ```

4. \[Terminal 1\] Inside the container, build and source the workspace:  

    ```bash
    cd /workspaces/isaac_ros-dev && \
      colcon build --symlink-install && \
      source install/setup.bash
    ```

5. \[Terminal 1\] Run the launch file, which launches the example and wait for 5 seconds:

    ```bash
    ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_realsense.launch.py
    ```

6. \[Terminal 2\] Attach a second terminal to check the operation.

    Attach another terminal to the running container for issuing other ROS2 commands.

    ```bash
    isaac_ros_container
    ```

    First check if you can see all the ROS2 topics expected.

    ```bash
    ros2 topic list
    ```

    > Output example:
    >
    > ```bash
    > /camera/accel/imu_info
    > /camera/accel/metadata
    > /camera/accel/sample
    > /camera/extrinsics/depth_to_accel
    > /camera/extrinsics/depth_to_gyro
    > /camera/extrinsics/depth_to_infra1
    > /camera/extrinsics/depth_to_infra2
    > /camera/gyro/imu_info
    > /camera/gyro/metadata
    > /camera/gyro/sample
    > /camera/imu
    > /camera/infra1/camera_info
    > /camera/infra1/image_rect_raw
    > /camera/infra1/image_rect_raw/compressed
    > /camera/infra1/image_rect_raw/compressedDepth
    > /camera/infra1/image_rect_raw/theora
    > /camera/infra1/metadata
    > /camera/infra2/camera_info
    > /camera/infra2/image_rect_raw
    > /camera/infra2/image_rect_raw/compressed
    > /camera/infra2/image_rect_raw/compressedDepth
    > /camera/infra2/image_rect_raw/theora
    > /camera/infra2/metadata
    > /parameter_events
    > /rosout
    > /tf
    > /tf_static
    > /visual_slam/imu
    > /visual_slam/status
    > /visual_slam/tracking/odometry
    > /visual_slam/tracking/slam_path
    > /visual_slam/tracking/vo_path
    > /visual_slam/tracking/vo_pose
    > /visual_slam/tracking/vo_pose_covariance
    > /visual_slam/vis/gravity
    > /visual_slam/vis/landmarks_cloud
    > /visual_slam/vis/localizer
    > /visual_slam/vis/localizer_loop_closure_cloud
    > /visual_slam/vis/localizer_map_cloud
    > /visual_slam/vis/localizer_observations_cloud
    > /visual_slam/vis/loop_closure_cloud
    > /visual_slam/vis/observations_cloud
    > /visual_slam/vis/pose_graph_edges
    > /visual_slam/vis/pose_graph_edges2
    > /visual_slam/vis/pose_graph_nodes
    > /visual_slam/vis/velocity
    > ```

    Check the frequency of the `realsense-camera` node's output frequency.

    ```bash
    ros2 topic hz /camera/infra1/image_rect_raw --window 20
    ```

    > Example output:
    >
    > ```bash
    > average rate: 89.714
    >         min: 0.011s max: 0.011s std dev: 0.00025s window: 20
    > average rate: 90.139
    >         min: 0.010s max: 0.012s std dev: 0.00038s window: 20
    > average rate: 89.955
    >         min: 0.011s max: 0.011s std dev: 0.00020s window: 20
    > average rate: 89.761
    >         min: 0.009s max: 0.013s std dev: 0.00074s window: 20
    > ```
    >
    > `Ctrl` + `c` to stop the output.

    You can also check the frequency of IMU topic.

    ```bash
    ros2 topic hz /camera/imu --window 20
    ```

    > Example output:
    >
    > ```bash
    > average rate: 199.411
    >         min: 0.004s max: 0.006s std dev: 0.00022s window: 20
    > average rate: 199.312
    >         min: 0.004s max: 0.006s std dev: 0.00053s window: 20
    > average rate: 200.409
    >         min: 0.005s max: 0.005s std dev: 0.00007s window: 20
    > average rate: 200.173
    >         min: 0.004s max: 0.006s std dev: 0.00028s window: 20
    > ```

    Lastly, check if you are getting the output from the `visual_slam` node at the same rate as the input.

    ```bash
    ros2 topic hz /visual_slam/tracking/odometry --window 20
    ```

    > Example output
    >
    > ```bash
    > average rate: 58.086
    >         min: 0.002s max: 0.107s std dev: 0.03099s window: 20
    > average rate: 62.370
    >         min: 0.001s max: 0.109s std dev: 0.03158s window: 20
    > average rate: 90.559
    >         min: 0.009s max: 0.013s std dev: 0.00066s window: 20
    > average rate: 85.612
    >         min: 0.002s max: 0.100s std dev: 0.02079s window: 20
    > average rate: 90.032
    >         min: 0.010s max: 0.013s std dev: 0.00059s window: 20
    > ```

## Tutorial Walkthrough - Visualization

At this point, you have two options for checking the `visual_slam` output.

- **Live visualization**:  Run Rviz2 live while running `realsense-camera` node and `visual_slam` nodes.
- **Offline visualization**: Record rosbag file and check the recorded data offline (possibly on a different machine)

Running Rviz2 on a remote PC over the network is tricky and is very difficult especially when you have image message topics to subscribe due to added burden on ROS 2 network transport.

Working on Rviz2 in a X11-forwarded window is also difficult because of the network speed limitation.

Therefore, if you are running `visual_slam` on Jetson, it is generally recommended **NOT** to evaluate with live visualization (1).

### Live visualization

1. \[Terminal 2\] Open Rviz2 from the second terminal:

    ```bash
    rviz2 -d src/isaac_ros_visual_slam/isaac_ros_visual_slam/rviz/realsense.cfg.rviz
    ```

    As you move the camera, the position and orientation of the frames should correspond to how the camera moved relative to its starting pose.
    <div align="center"><img src="../resources/realsense.gif" width="600px"/></div>

### Offline visualization

1. \[Terminal 2\] Save rosbag file

    Record the output in your rosbag file (along with the input data for later visual inspection).

    ```bash
    export ROSBAG_NAME=courtyard-d435i
    ros2 bag record -o ${ROSBAG_NAME} \
      /camera/imu /camera/accel/metadata /camera/gyro/metadata \
      /camera/infra1/camera_info /camera/infra1/image_rect_raw \
      /camera/infra1/metadata \
      /camera/infra2/camera_info /camera/infra2/image_rect_raw \
      /camera/infra2/metadata \
      /tf_static /tf \
      /visual_slam/status \
      /visual_slam/tracking/odometry \
      /visual_slam/tracking/slam_path /visual_slam/tracking/vo_path \
      /visual_slam/tracking/vo_pose /visual_slam/tracking/vo_pose_covariance \
      /visual_slam/vis/landmarks_cloud /visual_slam/vis/loop_closure_cloud \
      /visual_slam/vis/observations_cloud \
      /visual_slam/vis/pose_graph_edges /visual_slam/vis/pose_graph_edges2 \
      /visual_slam/vis/pose_graph_nodes
    ros2 bag info ${ROSBAG_NAME} 
    ```

    If you plan to run the rosbag on a remote machine (PC) for evaluation, you can send the rosbag file to your remote machine.

    ```bash
    export IP_PC=192.168.1.100
    scp -r ${ROSBAG_NAME} ${PC_USER}@${IP_PC}:/home/${PC_USER}/workspaces/isaac_ros-dev/
    ```

2. \[Terminal A\] Launch Rviz2

    > If you are SSH-ing into Jetson from your PC, make sure you enabled X forwarding by adding `-X` option with SSH command.
    >
    > ```bash
    > ssh -X ${USERNAME_ON_JETSON}@${IP_JETSON}
    > ```

    Launch the Isaac ROS container.

    ```bash
    isaac_ros_container
    ```

    Run Rviz with a configuration file for visualizing set of messages from Visual SLAM node.

    ```bash
    cd /workspaces/isaac_ros-dev
    rviz2 -d src/isaac_ros_visual_slam/isaac_ros_visual_slam/rviz/vslam_keepall.cfg.rviz
    ```

3. \[Terminal B\] Playback the recorded rosbag

    Attach another terminal to the running container.

    ```bash
    isaac_ros_container
    ```

    Play the recorded rosbag file.

    ```bash
    ros2 bag play ${ROSBAG_NAME} 
    ```

    RViz should start showing visualization like the following.
    <div align="center"><img src="../resources/RViz_0217-cube_vslam-keepall.png" width="600px"/></div>
