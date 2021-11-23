# Isaac ROS Visual Odometry

<div align="center"><img src="resources/elbrus_live_optimized.gif" width="400px"/></div>

This repository provides a ROS2 package that estimates stereo visual inertial odometry using the [Isaac Elbrus](https://docs.nvidia.com/isaac/isaac/packages/visual_slam/doc/elbrus_visual_slam.html) GPU-accelerated library. It takes in a time synced pair of stereo images (grayscale) along with respective camera intrinsics to publish the current pose of the camera relative to its start pose. 

Elbrus is based on two core technologies: Visual Odometry (VO) and Simultaneous Localization and Mapping (SLAM).

Visual Odometry is a method for estimating a camera position relative to its start position. This method has an iterative nature. At each iteration it considers two consequential input frames (stereo pairs). On both frames, it finds a set of keypoints. Matching keypoints in these two sets gives the ability to estimate the transition and relative rotation of the camera between frames.

Simultaneous Localization and Mapping is a method built on top of the VO predictions. It aims to improve the quality of VO estimations by leveraging the knowledge of previously seen parts of a trajectory. It detects if the current scene was seen in the past (i.e. a loop in camera movement) and runs an additional optimization procedure to tune previously obtained poses.

Along with visual data Elbrus can optionally use Inertial Measurement Unit (IMU) measurements. It automatically switches to IMU when VO is unable to estimate a pose–for example, when there is dark lighting or long solid surfaces in front of a camera.

Elbrus delivers real-time tracking performance: more than 60 FPS for VGA resolution. For the KITTI benchmark, the algorithm achieves a drift of ~1% in localization and an orientation error of 0.003 degrees per meter of motion.

Elbrus allows for robust tracking in various environments and with different use cases: indoor, outdoor, aerial, HMD, automotive, and robotics.

This has been tested on ROS2 (Foxy) and should build and run on x86_64 and aarch64 (Jetson).

## System Requirements
This Isaac ROS package is designed and tested to be compatible with ROS2 Foxy on Jetson hardware.

### Jetson
- AGX Xavier or Xavier NX
- JetPack 4.6

### x86_64
- CUDA 10.2+ supported discrete GPU
- Ubuntu 18.04+

**Note:** For best performance on Jetson, ensure that power settings are configured appropriately ([Power Management for Jetson](https://docs.nvidia.com/jetson/l4t/index.html#page/Tegra%20Linux%20Driver%20Package%20Development%20Guide/power_management_jetson_xavier.html#wwpID0EUHA)).

### Docker
Precompiled ROS2 Foxy packages are not available for JetPack 4.6 (based on Ubuntu 18.04 Bionic). You can either manually compile ROS2 Foxy and required dependent packages from source or use the Isaac ROS development Docker image from [Isaac ROS Common](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common).

You must first install the [Nvidia Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html) to make use of the Docker container development/runtime environment.

Configure `nvidia-container-runtime` as the default runtime for Docker by editing `/etc/docker/daemon.json` to include the following:
```
    "runtimes": {
        "nvidia": {
            "path": "nvidia-container-runtime",
            "runtimeArgs": []
        }
    },
    "default-runtime": "nvidia"
```
and then restarting Docker: `sudo systemctl daemon-reload && sudo systemctl restart docker`

Run the following script in `isaac_ros_common` to build the image and launch the container:

`$ scripts/run_dev.sh <optional path>`

You can either provide an optional path to mirror in your host ROS workspace with Isaac ROS packages, which will be made available in the container as `/workspaces/isaac_ros-dev`, or you can setup a new workspace in the container.

### Package Dependencies
- [isaac_ros_common](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common)
- [isaac_ros_image_pipeline](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline)

**Note:** `isaac_ros_common` is used for running tests and/or creating a development container and `isaac_ros_image_pipeline` is used as an executable dependency in launch files.

## Rectified vs Smooth Transform
Elbrus provides the user with two possible modes: visual odometry with SLAM (rectified transform) or pure visual odometry (smooth transform).

Using rectified transform (`enable_rectified_pose = true`) is more beneficial in cases when the camera path is frequently encircled or entangled. Though rectified transform increases computational resource demands for VO, it usually reduces the final position drift dramatically.

On the other hand, using pure VO (smooth tranform) may be useful on relatively short and straight camera trails. This choice will reduce compute usage and increase working speed without causing essential pose drift.

## Coordinate Frames

This section describes the coordinate frames that are involved in the `VisualOdomertyNode`. The frames discussed below are oriented as follows:<br>

<div align="center"><img src="resources/Axes.png" width="300px"/></div>

1. `left_camera_frame`: The frame associated with left eye of the stereo camera. Note that this is not the same as optical frame.
2. `right_camera_frame`: The frame associated with right eye of the stereo camera. Note that this is not the same as optical frame.
3. `imu_frame`: The frame associated with the IMU sensor (if available).
4. `fixed_frame`: The fixed frame that aligns with the start pose of the stereo camera. The tracked poses are published in the [TF](http://wiki.ros.org/tf) tree with respect to this frame.
5. `current_smooth_frame`: The moving frame that tracks the current smooth pose of the stereo camera. The poses in this frame represent a smooth continous motion. Note that the frame can drift over time.
6. `current_rectified_frame`: The moving frame that tracks the current rectified pose of the stereo camera. The poses in this frame represent an accurate position and orientation. Note that the frame can have sudden jumps.

## Quickstart
1. Create a ROS2 workspace if one is not already prepared:  
`mkdir -p your_ws/src`  
**Note**: The workspace can have any name; these steps use the name `your_ws`. <br>
2. Clone this package repository to `your_ws/src/isaac_ros_visual_odometry`. Check that you have [Git LFS](https://git-lfs.github.com/) installed before cloning to pull down all large files.<br>
`sudo apt-get install git-lfs`<br>
`cd your_ws/src && git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_odometry`
3. Build and source the workspace:<br>
`cd your_ws && colcon build --symlink-install && source install/setup.bash`
4. (Optional) Run tests to verify complete and correct installation:  
`colcon test`

This repository comes with pre-configured launch files for a real camera and a simulator. 
###  For RealSense camera
**Note:** You will need to calibrate the intrinsics of your camera if you want the node to determine the 3D pose of your camera. See [here](https://navigation.ros.org/tutorials/docs/camera_calibration.html) for more details.  

5. Start `isaac_ros_visual_odometry` using the launch files:<br>
    `ros2 launch isaac_ros_visual_odometry isaac_ros_visual_odometry_realsense.launch.py`

6. To connect the RealSense camera's default TF tree to the `current_smooth_frame`(default value is *base_link_smooth*), run the following command: <br> `ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 base_link_smooth camera_link`
7. In a separate terminal, spin up RViz to see the tracking of the camera:<br>
    `rviz2`
8. Add the TF tree in the **Displays** panel of the RViz. <br><div align="center"><img src="resources/Rviz_add_tf.png" width="600px"/></div>
9.  Set the **Fixed frame** in the **Global Options** to the name provided in the `fixed_frame` variable. <br> <div align="center"><img src="resources/Rviz_set_fixed_frame.png" width="300px"/></div>
10.  Try moving your camera, and you should see something like this: <br> <div align="center"><img src="resources/Rviz.png" width="600px"/></div>
11.  If you prefer to observe the Visual Odometry output in a text mode, on a separate terminal echo the contents of the `/visual_odometry/tf_stamped` topic with the following command:   
`ros2 topic echo /visual_odometry/tf_stamped` <br> <div align="center"><img src="resources/Terminal_output.png" width="600px"/></div>

### For Isaac Sim

5. Make sure you have Isaac Sim [set up](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim.html#setting-up-isaac-sim) correctly and choose the appropriate working environment[[Native](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/setup.html)/[Docker&Cloud](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/setup.html#docker-cloud-deployment)]. For this walkthrough, we are using the native workstation setup for Isaac Sim.
6. See [Running For The First Time](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/first_run.html#) section to launch Isaac Sim from the [app launcher](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/user_interface_launcher.html) and click on the **Isaac Sim** button.
7. Set up the Isaac Sim ROS2 bridge as described [here](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/ext_omni_isaac_ros_bridge.html#ros2-bridge).
8. Connect to the Nucleus server as shown in the [Getting Started](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/sample_jetbot.html#getting-started) section if you have not done it already.
9. Open up the Isaac ROS Common USD scene located at:
   
   `omniverse://<your_nucleus_server>/Isaac/Samples/ROS/Scenario/carter_warehouse_apriltags_worker.usd`.
   
   And wait for it to load completly.

10. Enable the right camera for a stereo image pair. Go to the stage tab and select `/World/Carter_ROS/ROS_Camera_Stereo_Right` then tick the `enabled` checkbox.
    
    <div align="center"><img src="resources/Isaac_sim_enable_stereo.png" width="500px"/></div>
11. Press **Play** to start publishing data from the Isaac Sim application.
    
    <div align="center"><img src="resources/Isaac_sim_visual_odometry.png" width="800px"/></div>
12. In a separate terminal, start `isaac_ros_visual_odometry` using the launch files:<br>
    `ros2 launch isaac_ros_visual_odometry isaac_ros_visual_odometry_isaac_sim.launch.py`
13. In a separate terminal, send the signal to rotate the robot in the sim as follows: <br>
`ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.05}}'`
14. In a separate terminal, run RViz to visualize the tracking of the pose of the camera:<br>
    `rviz2`
15.  Add the TF tree in the **Displays** RViz panel. <br> <div align="center"><img src="resources/Rviz_add_tf.png" width="600px"/></div>
16.  Set the **Fixed frame** in the **Global Options** to the name provided in the `fixed_frame` variable. <br> <div align="center"><img src="resources/Rviz_set_fixed_frame.png" width="300px"/></div>
17.  You should see something like this: <br> <div align="center"><img src="resources/Rviz_isaac_sim.png" width="600px"/></div>
18.  If you prefer to observe the Visual Odometry output in a text mode, on a separate terminal, echo the contents of the `/visual_odometry/tf_stamped` topic with the following command:   
`ros2 topic echo /visual_odometry/tf_stamped` <br> <div align="center"><img src="resources/Terminal_output.png" width="600px"/></div>

### For Isaac Sim with Hardware in the loop (HIL)

The following instructions are for a setup where we can run the sample on a Jetson device and Isaac Sim on an x86 machine. We will use the ROS_DOMAIN_ID environment variable to have a separate logical network for Isaac Sim and the sample application. 

NOTE: Before executing any of the ROS commands, make sure to set the ROS_DOMAIN_ID variable first.

1. Complete step 5 of [For Isaac Sim](#for-isaac-sim) section if you have not done it already.
2. Open the location of the Isaac Sim package in the terminal by clicking the [**Open in Terminal**](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/user_interface_launcher.html) button.
   
   <div align="center"><img src="resources/Isaac_sim_app_launcher.png" width="400px"/></div>
3. In the terminal opened by the previous step, set the ROS_DOMAIN_ID as shown:

   `export ROS_DOMAIN_ID=<some_number>`

4. Launch Isaac Sim from the script as shown:
   
   `./isaac-sim.sh` 
   <div align="center"><img src="resources/Isaac_sim_app_terminal.png" width="600px"/></div>
5. Continue with step 7 of [For Isaac Sim](#for-isaac-sim) section. Make sure to set the ROS_DOMAIN_ID variable before running the sample application.

# Package Reference

## isaac_ros_visual_odometry

### Available Components
| Component            | Topics Subscribed                                                                                                                                                                                                                                                                                                                                                                                                                              | Topics Published                                                                                                                                                                                                                                                                                                          | Parameters                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    |
| -------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `VisualOdometryNode` | `/stereo_camera/left/image`: The image from the left eye of the stereo camera in grayscale <br> `/stereo_camera/left/camera_info`: CameraInfo from the left eye of the stereo camera <br> `/stereo_camera/right/image`: The image from the right eye of the stereo camera in grayscale <br> `/stereo_camera/right/camera_info`: CameraInfo from the right eye of the stereo camera <br> `visual_odometry/imu`: Sensor data from IMU(optional). | `/visual_odometry/tf_stamped`: A consolidated message that provies information about the tracked poses. It consists of two poses: `smooth_transform` is calculated from pure visual odometry; `rectified_transform` is calculated with SLAM enabled. It also gives the states of both poses. <br> See more details below. | `enable_rectified_pose`: If enabled, `rectified_transform` will be populated in the message. It is enabled by default.<br> `denoise_input_images`: If enabled, input images are denoised. It can be enabled when images are noisy because of low-light conditions. It is disabled by default. <br> `rectified_images`: Flag to mark if the images coming in the subscribed topics are rectified or raw. It is enabled by default. <br> `enable_imu`: If enabled, IMU data is used. It is disabled by default. <br> `enable_debug_mode`: If enabled, a debug dump(image frames, timestamps, and camera info) is saved on to the disk at the path indicated by `debug_dump_path`. It is disabled by default <br> `debug_dump_path`: The path to the directory to store the debug dump data. The default value is `/tmp/elbrus`. <br> `gravitational_force`: The initial gravity vector defined in the odom frame. If the IMU sensor is not parallel to the floor, update all the axes with appropriate values. The default value is `{0.0, 0, -9.8}`. <br> `left_camera_frame`: The name of the left camera frame. The default value is `left_cam`. <br> `right_camera_frame`: The name of the right camera frame. The default value is `right_cam`. <br> `imu_frame`: The name of the imu frame. The default value is `imu`. <br> `fixed_frame`: The name of the frame where odomerty or poses are tracked. This is a fixed frame. The default value is `odom`. <br> `current_smooth_frame`: The name of the frame where `smooth_transform` is published. This is a moving frame. The default value is *base_link_smooth*. <br> `current_rectified_frame`: The name of the frame where `rectified_transform` is published. This is a moving frame. The default value is `base_link_rectified`. |

### Launch files

| Name                                          | Description                                                                |
| --------------------------------------------- | -------------------------------------------------------------------------- |
| isaac_ros_visual_odometry.launch.py           | Launch file to bring up visual odometry node standalone.                   |
| isaac_ros_visual_odometry_isaac_sim.launch.py | Launch file which brings up visual odometry node configured for Isaac Sim. |
| isaac_ros_visual_odometry_realsense.launch.py | Launch file which brings up visual odometry node configured for RealSense. |
| isaac_ros_visual_odometry_zed2.launch.py      | Launch file which brings up visual odometry node remapped for Zed2.        |


### `visual_odometry/tf_stamped`
<pre><code>
# The frame id in the header is used as the reference frame of both the transforms below.
std_msgs/Header header

# The frame id of the child frame to which the transforms point.
string child_frame_id

# Translation and rotation in 3-dimensions of child_frame_id from header.frame_id.
# rectified_tranform represents the most accurate position and orientation.
# It shall be consumed if robust tracking is needed. Note: It can result in sudden jumps.
geometry_msgs/Transform rectified_transform

# smooth_transform represents the position and orientation which corresponds to a smooth continous motion.
# It shall be consumed if using as another signal for e.g. in case of sensor fusion.
# Note: It might drift over time  
geometry_msgs/Transform smooth_transform

# Pure visual odometry return code:
# 0 - Unknown state
# 1 - Success
# 2 - Failed
# 3 - Success but invalidated by IMU
uint8 vo_state

# Integrator status:
# 0 - Unknown state
# 1 - Static
# 2 - Inertial
# 3 - IMU
uint8 integrator_state
</code></pre>

# Troubleshooting
* If RViz is not showing the poses, check the **Fixed Frame** value.
* If you are seeing `Tracker is lost.` messages frequently, it could be caused by the following issues:
   * Fast motion causing the motion blur in the frames.
   * Low-lighting conditions
   * The wrong `camerainfo` is being published
* For better performance:
   * Increase the capture framerate from the camera to yield a better tracking result. 
   * If input images are noisy, you can use the `denoise_input_images` flag in the node.

# Updates

| Date       | Changes                            |
| ---------- | ---------------------------------- |
| 2021-11-15 | Isaac Sim HIL documentation update |
| 2021-10-20 | Initial release                    |
