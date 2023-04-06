# Tutorial for Visual SLAM with Isaac Sim

<div align="center"><img src="../resources/After_localization.png" width="600px" alt="After localization" title="After localization."/></div>

## Overview

This tutorial walks you through a graph to estimate 3D pose of the camera with [Visual SLAM](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam) using images from Isaac Sim.

Last validated with [Isaac Sim 2022.2.1](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/release_notes.html#id1)

## Tutorial Walkthrough

1. Complete the [Quickstart section](../README.md#quickstart) in the main README.
2. Launch the Docker container using the `run_dev.sh` script:

    ```bash
    cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common && \
      ./scripts/run_dev.sh
    ```

3. Inside the container, build and source the workspace:  

    ```bash
    cd /workspaces/isaac_ros-dev && \
      colcon build --symlink-install && \
      source install/setup.bash
    ```

4. Install and launch Isaac Sim following the steps in the [Isaac ROS Isaac Sim Setup Guide](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docs/isaac-sim-sil-setup.md)
5. Open up the Isaac ROS Common USD scene (using the *Content* tab) located at:

    ```text
    http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/2022.2.1/Isaac/Samples/ROS2/Scenario/carter_warehouse_apriltags_worker.usd
    ```

   And wait for it to load completely.
6. Go to the *Stage* tab and select `/World/Carter_ROS/ROS_Cameras/ros2_create_camera_right_info`, then in *Property* tab *-> OmniGraph Node -> Inputs -> stereoOffset X* change `0` to `-175.92`.
    <div align="center"><img src="../resources/Isaac_sim_set_stereo_offset.png" width="500px"/></div>
7. Enable the right camera for a stereo image pair. Go to the *Stage* tab and select `/World/Carter_ROS/ROS_Cameras/enable_camera_right`, then tick the *Condition* checkbox.
    <div align="center"><img src="../resources/Isaac_sim_enable_stereo.png" width="500px"/></div>
8. Press **Play** to start publishing data from the Isaac Sim application.
    <div align="center"><img src="../resources/Isaac_sim_play.png" width="800px"/></div>
9. In a separate terminal, start `isaac_ros_visual_slam` using the launch files:

    ```bash
    ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_isaac_sim.launch.py
    ```

10. In a separate terminal, send the signal to rotate the robot about its own axis as follows:

    ```bash
    ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.05}}'
    ```

11. In a separate terminal, spin up RViz with default configuration file to see the rich visualizations as the robot moves.

    ```bash
    rviz2 -d src/isaac_ros_visual_slam/isaac_ros_visual_slam/rviz/default.cfg.rviz
    ```

    <div align="center"><img src="../resources/RViz_isaac_sim.png" width="800px"/></div>
12. To see the odometry messages, in a separate terminal echo the contents of the `/visual_slam/tracking/odometry` topic with the following command:

    ```bash
    ros2 topic echo /visual_slam/tracking/odometry
    ```

    <div align="center"><img src="../resources/Terminal_output.png" width="600px"/></div>

## Saving and using the map

As soon as you start the visual SLAM node, it starts storing the landmarks and the pose graph. You can save them in a map and store the map onto a disk. Make a call to the `SaveMap` ROS 2 Action with the following command:

```bash
ros2 action send_goal /visual_slam/save_map isaac_ros_visual_slam_interfaces/action/SaveMap "{map_url: /path/to/save/the/map}"
```

<br>
<div align="center"><img src="../resources/Save_map.png" width="400px"/></div>
<br>
<div align="center"><img src="../resources/RViz_isaac_sim_mapping.png" width="800px" alt="Sample run before saving the map" title="Sample run before saving the map."/></div>
<br>

Now, you will try to load and localize in the previously saved map. First, stop the `visual_slam` node lauched for creating and saving the map, then relaunch it.

Use the following command to load the map from the disk and provide an approximate start location(prior):

```bash
ros2 action send_goal /visual_slam/load_map_and_localize isaac_ros_visual_slam_interfaces/action/LoadMapAndLocalize "{map_url: /path/to/save/the/map, localize_near_point: {x: x_val, y: y_val, z: z_val}}"
```

<div align="center"><img src="../resources/Load_and_localize.png" width="400px"/></div>
<br>

Once the above step returns success, you have successfully loaded and localized your robot in the map. If it results in failure, there might be a possibilty that the current landmarks from the approximate start location are not matching with stored landmarks and you need to provide another valid value.

<div align="center">
    <figure class="image">
        <img src="../resources/Before_localization.png" width="600px" alt="Before localization" title="Before localization"/>
        <figcaption>Before Localization</figcaption>
    </figure>
</div>
<br>
<div align="center">
    <figure class="image">
        <img src="../resources/After_localization.png" width="600px" alt="After localization" title="After localization."/>
        <figcaption>After Localization</figcaption>
    </figure>
</div>
