# Isaac ROS Visual SLAM

<div align="center"><img src="resources/elbrus_ros_3.gif" width="400px"/></div>

This repository provides a ROS2 package that performs stereo visual simultaneous localization and mapping (VSLAM) and estimates stereo visual inertial odometry using the [Isaac Elbrus](https://docs.nvidia.com/isaac/isaac/packages/visual_slam/doc/elbrus_visual_slam.html) GPU-accelerated library. It takes in a time-synced pair of stereo images (grayscale) along with respective camera intrinsics to publish the current pose of the camera relative to its start pose. 

Elbrus is based on two core technologies: Visual Odometry (VO) and Simultaneous Localization and Mapping (SLAM).

Visual SLAM is a method for estimating a camera position relative to its start position. This method has an iterative nature. At each iteration, it considers two consequential input frames (stereo pairs). On both the frames, it finds a set of keypoints. Matching keypoints in these two sets gives the ability to estimate the transition and relative rotation of the camera between frames.

Simultaneous Localization and Mapping is a method built on top of the VO predictions. It aims to improve the quality of VO estimations by leveraging the knowledge of previously seen parts of a trajectory. It detects if the current scene was seen in the past (i.e. a loop in camera movement) and runs an additional optimization procedure to tune previously obtained poses.

Along with visual data, Elbrus can optionally use Inertial Measurement Unit (IMU) measurements. It automatically switches to IMU when VO is unable to estimate a pose–for example, when there is dark lighting or long solid featureless surfaces in front of a camera. Elbrus delivers real-time tracking performance: more than 60 FPS for VGA resolution. For the KITTI benchmark, the algorithm achieves a drift of ~1% in localization and an orientation error of 0.003 degrees per meter of motion. Elbrus allows for robust tracking in various environments and with different use cases: indoor, outdoor, aerial, HMD, automotive, and robotics.

This has been tested on ROS2 (Foxy) and should build and run on x86_64 and aarch64 (Jetson). For solutions to known issues, please visit the [Troubleshooting](#troubleshooting) section below.

## Elbrus SLAM
All slam-related operations work in parallel to visual odometry in a separate thread.
Input images get copied into GPU and then Elbrus starts tracking.

Elbrus uses the following:
* 2D features on the input images
* Landmarks - The patch of pixels in the source images with coordinates of the feature assosicated with the position of the camera from where that feature is visible.
* PoseGraph: A graph that tracks the poses of the camera from where the landmarks are viewed.

Landmarks and PoseGraph are stored in a data structure that doesn't increase its size in the case where the same landmark is visited more than once.

Imagine a robot moving around and returning to the same place. Since odometry always has some accumulated drift, we see a deviation in trajectory from the ground truth. SLAM can be used to solve this problem. During the robot's motion, SLAM stores all landmarks and it continuously verifies if each landmark have been seen before. When Elbrus recognizes several landmarks, it determines their position(from the data structure). The so-called loop closure occurs, so named because at this moment a connection is added to the PoseGraph, which makes a loop from the free trail of the poses. Immediately after that, Elbrus performs a graph optimization that leads to the correction of the current odometric pose and all poses in the graph. The procedure for adding landmarks is designed such that if we do not see a landmark in the place where it was expected, then such landmarks are marked for deletion and will get deleted. This allows you to use Elbrus over a changing terrain.

### List of Useful Visualizations: 
* `visual_slam/vis/observations_cloud` - Point cloud for 2D Features
* `visual_slam/vis/landmarks_cloud` - All mapped landmarks 
* `visual_slam/vis/loop_closure_cloud` - Landmarks visible in last loop closure
* `visual_slam/vis/pose_graph_nodes` - Pose graph nodes
* `visual_slam/vis/pose_graph_edges` - Pose graph edges

### Saving the map
Naturally, we would like to save the stored landmarks and pose graph in a map. We have implemented a ROS2 action to save the map to the disk called SaveMap.

### Loading and Localization in the Map 
Once the map has been saved to the disk, it can be used later to localize the robot. To load the map into the memory, we have made a ROS2 action called LoadMapAndLocalize. It requires a map file path and a prior(initial guess of where the robot is in the map) pose. Given the prior pose and current set of camera frames, Elbrus tries to find the pose of the landmarks in the requested map that matches the current set. If the the localization is successful, it will load the map in the memory and, if it fails, it will continue building a new map.

Both SaveMap and LoadMapAndLocalize can take some time to complete. Hence, they are designed to be asynchronous to avoid interfering with odometry calculations.

## System Requirements
This Isaac ROS package is designed and tested to be compatible with ROS2 Foxy on Jetson hardware.

### Jetson
- [Jetson AGX Xavier or Xavier NX](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/)
- [JetPack 4.6.1](https://developer.nvidia.com/embedded/jetpack)

### x86_64
- Ubuntu 20.04+
- CUDA 11.4 supported discrete GPU
- Nvidia driver version >= 470.103.01

**Note:** For best performance on Jetson, ensure that power settings are configured appropriately ([Power Management for Jetson](https://docs.nvidia.com/jetson/l4t/index.html#page/Tegra%20Linux%20Driver%20Package%20Development%20Guide/power_management_jetson_xavier.html#wwpID0EUHA)).


### Docker
Precompiled ROS2 Foxy packages are not available for JetPack 4.6.1 (based on Ubuntu 18.04 Bionic). You can either manually compile ROS2 Foxy and required dependent packages from source or use the Isaac ROS development Docker image from [Isaac ROS Common](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common).

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
Then restart the Docker service: 
```
sudo systemctl daemon-reload && sudo systemctl restart docker
```

Run the following script in `isaac_ros_common` to build the image and launch the container:

```
$ scripts/run_dev.sh <optional path>
```

You can either provide an optional path to mirror in your host ROS workspace with Isaac ROS packages, which will be made available in the container as `/workspaces/isaac_ros-dev`, or you can set up a new workspace in the container.

### Package Dependencies
- [isaac_ros_common](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common)
- [isaac_ros_image_pipeline](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline)

**Note:** `isaac_ros_common` is used for running tests and/or creating a development container, and `isaac_ros_image_pipeline` is used as an executable dependency in launch files.
  
## Coordinate Frames

This section describes the coordinate frames that are involved in the `VisualSlamNode`. The frames discussed below are oriented as follows:<br>

<div align="center"><img src="resources/Axes.png" width="300px"/></div>

1. `input_base_frame`: The name of the frame used to calculate transformation between baselink and left camera. The default value is empty(''), which means the value of ``base_frame_`` will be used. If ``input_base_frame_`` and ``base_frame_`` are both empty, the left camera is assumed to be in the robot's center.
2. `input_left_camera_frame`: The frame associated with left eye of the stereo camera. Note that this is not the same as the optical frame. The default value is empty(''), which means the left camera is in the robot's center and ``left_pose_right`` will be calculated from the CameraInfo message.
3. `input_right_camera_frame`: The frame associated with the right eye of the stereo camera. Note that this is not the same as the optical frame. The default value is empty(''), which means left and right cameras have identity rotation and are horizontally aligned, so ``left_pose_right`` will be calculated from CameraInfo.
4. `input_imu_frame`: The frame associated with the IMU sensor (if available). It is used to calculate ``left_pose_imu``.

## Quickstart
### Building the Package
1. Create a ROS2 workspace if one is not already prepared:  
    ```
    mkdir -p your_ws/src
    ```  
**Note**: The workspace can have any name; these steps use the name `your_ws`. <br>

2. Clone this package and its dependencies. Check that you have [Git LFS](https://git-lfs.github.com/) installed before cloning to pull down all large files.<br>
    ```
    sudo apt-get install git-lfs && cd your_ws/src
    ```
    ```
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam &&
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline &&
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common
    ```

3. Install package dependencies in the ROS workspace. In our case it is `your_ws`
    ```
    rosdep install -i -r --from-paths src --rosdistro foxy -y --skip-keys "libopencv-dev libopencv-contrib-dev libopencv-imgproc-dev python-opencv python3-opencv"
    ```

4. Build and source the workspace:
    ```
    cd your_ws && colcon build --symlink-install && source install/setup.bash
    ```
5. (Optional) Run tests to verify complete and correct installation:  
    ```
    colcon test
    ```

This repository comes with pre-configured launch files for a real camera and a simulator. 
###  Working with a RealSense Camera
**Note:** You will need to calibrate the intrinsics of your camera if you want the node to determine the 3D pose of your camera. See [here](https://navigation.ros.org/tutorials/docs/camera_calibration.html) for more details.

1. Start `isaac_ros_visual_slam` using the launch files:<br>
    ```
    ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_realsense.launch.py
    ```

2. In a separate terminal, spin up RViz with a default configuration file to see the rich visualizations. <br>
    ```
    rviz2 -d src/isaac_ros_visual_slam/isaac_ros_visual_slam/rviz/default.cfg.rviz
    ```
<div align="center"><img src="resources/RViz_config.png" width="600px"/></div>

3. Try moving your camera, and you should see something like this: <br> <div align="center"><img src="resources/RViz_visual_slam.png" width="600px"/></div>
4. To see the odometry messages, in a separate terminal echo the contents of the `/visual_slam/tracking/odometry` topic with the following command:   
    ```
    ros2 topic echo /visual_slam/tracking/odometry
    ``` 
<div align="center"><img src="resources/Terminal_output.png" width="600px"/></div>

### Working with Isaac Sim

1. Ensure you have Isaac Sim [set up](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim.html#setting-up-isaac-sim) correctly and choose the appropriate working environment [[Native](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_basic.html)/[Docker&Cloud](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_advanced.html)]. This walkthrough uses the native workstation setup for Isaac Sim.
2. Set up the Nucleus server as shown [here](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_faq.html#nucleus-and-cache) to access the assets to work with Isaac Sim.
3. Open an instance of Isaac Sim from [Omniverse Launcher](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_basic.html#isaac-sim-setup-native-workstation-launcher) and use the  [Isaac Sim App Selector](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_basic.html#isaac-sim-app-selector) to start it.
4. Set up the Isaac Sim ROS2 bridge as described [here](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/ext_omni_isaac_ros_bridge.html#ros2-bridge).
5. Open up the Isaac ROS common USD scene located at:
   
   `omniverse://<your_nucleus_server>/Isaac/Samples/ROS/Scenario/carter_warehouse_apriltags_worker.usd`.
   
   Wait for it to load completly.

6.  Enable the right camera for a stereo image pair. Go to the stage tab and select `/World/Carter_ROS/ROS_Camera_Stereo_Right`, then tick the `enabled` checkbox.
    <div align="center"><img src="resources/Isaac_sim_enable_stereo.png" width="500px"/></div>
7.  Press **Play** to start publishing data from the Isaac Sim application.
    <div align="center"><img src="resources/Isaac_sim_visual_slam.png" width="800px"/></div>
8.  In a separate terminal, start `isaac_ros_visual_slam` using the launch files:<br>
    ```
    ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_isaac_sim.launch.py
    ```
9.  In a separate terminal, send the signal to rotate the robot about its own axis as follows: <br>
    ```
    ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.05}}'
    ```
10. In a separate terminal, spin up RViz with default configuration file to see the rich visualizations as the robot moves.<br>
    ```
    rviz2 -d src/isaac_ros_visual_slam/isaac_ros_visual_slam/rviz/default.cfg.rviz
    ```
    <div align="center"><img src="resources/RViz_isaac_sim.png" width="800px"/></div>
11. To see the odometry messages, in a separate terminal echo the contents of the `/visual_slam/tracking/odometry` topic with the following command:   
    ```
    ros2 topic echo /visual_slam/tracking/odometry
    ``` 
    <div align="center"><img src="resources/Terminal_output.png" width="600px"/></div>

### Saving and using the map

As soon as you start the visual SLAM node, it starts storing the landmarks and the pose graph. You can save them in a map and store the map onto a disk. Make a call to the `SaveMap` ROS2 Action with the following command:
```
ros2 action send_goal /visual_slam/save_map isaac_ros_visual_slam_interfaces/action/SaveMap "{map_url: /path/to/save/the/map}"
```
</br>
<div align="center"><img src="resources/Save_map.png" width="400px"/></div>
</br>
<div align="center"><img src="resources/RViz_isaac_sim_mapping.png" width="800px" alt="Sample run before saving the map" title="Sample run before saving the map."/></div>
</br>

Now, you will try to load and localize in the previously saved map. First, stop the `visual_slam` node lauched for creating and saving the map, then relaunch it.

Use the following command to load the map from the disk and provide an approximate start location(prior):
```
ros2 action send_goal /visual_slam/load_map_and_localize isaac_ros_visual_slam_interfaces/action/LoadMapAndLocalize "{map_url: /path/to/save/the/map localize_near_point: {x: x_val, y: y_val, z: z_val}}"
```


<div align="center"><img src="resources/Load_and_localize.png" width="400px"/></div>
</br>

Once the above step returns success, you have successfully loaded and localized your robot in the map. If it results in failure, there might be a possibilty that the current landmarks from the approximate start location are not matching with stored landmarks and you need to provide another valid value.

<div align="center"><img src="resources/Before_localization.png" width="800px" alt="Before localization" title="Before localization."/></div>
</br>
<div align="center"><img src="resources/After_localization.png" width="800px" alt="After localization" title="After localization."/></div>
</br>

### Working with Isaac Sim with Hardware in the loop (HIL)

The following instructions are for a setup where we can run the sample on a Jetson device and Isaac Sim on an x86 machine. These instructions will use the ``ROS_DOMAIN_ID`` environment variable to have a separate logical network for Isaac Sim and the sample application. 

**Note**: Before executing any of the ROS commands, make sure to set the ROS_DOMAIN_ID variable first.

1. Complete steps 1 and 2 of [Working with Isaac Sim](#working-with-isaac-sim) section.
2. Open the location of the Isaac Sim package in the terminal by clicking the **Open in Terminal** button in the [Isaac Sim App Selector](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/reference_user_interface.html#isaac-sim-app-selector) window.
   
   <div align="center"><img src="resources/Isaac_sim_app_launcher.png" width="400px"/></div>
3. In the terminal opened by the previous step, set the ``ROS_DOMAIN_ID`` as shown:

   ```
   export ROS_DOMAIN_ID=<some_number>
   ```

4. Launch Isaac Sim from the script as shown:
   
   ```
   ./isaac-sim.sh
   ``` 
   <div align="center"><img src="resources/Isaac_sim_app_terminal.png" width="600px"/></div>
5. Continue with step 4 of the [Working with Isaac Sim](#working-with-isaac-sim) section. Ensure the ``ROS_DOMAIN_ID`` variable is set before running the sample application.
   **Note**: In this case, step 8 of the [Working with Isaac Sim](#working-with-isaac-sim) section must be performed on a Jetson device (i.e. launching the ``isaac_ros_visual_slam`` package on the "hardware"). 

# Package Reference

## isaac_ros_visual_slam

### Available Components for visual_slam_node



| Topics Subscribed                  | Type                           | Description                                                     |
| ---------------------------------- | ------------------------------ | --------------------------------------------------------------- |
| `/stereo_camera/left/image`        | `sensor_msgs::msg::Image`      | The image from the left eye of the stereo camera in grayscale.  |
| `/stereo_camera/left/camera_info`  | `sensor_msgs::msg::CameraInfo` | CameraInfo from the left eye of the stereo camera.              |
| `/stereo_camera/right/image`       | `sensor_msgs::msg::Image`      | The image from the right eye of the stereo camera in grayscale. |
| `/stereo_camera/right/camera_info` | `sensor_msgs::msg::CameraInfo` | CameraInfo from the right eye of the stereo camera.             |
| `visual_slam/imu`                  | `sensor_msgs::msg::Imu`        | Sensor data from the IMU(optional).                             |

| Topics Published                          | Type                                                      | Description                                       |
| ----------------------------------------- | --------------------------------------------------------- | ------------------------------------------------- |
| `visual_slam/tracking/odometry`           | `nav_msgs::msg::Odometry`                                 | Odometry messages for the `base_link`.            |
| `visual_slam/tracking/vo_pose_covariance` | `geometry_msgs::msg::PoseWithCovarianceStamped`           | Current pose with covariance of the `base_link`.  |
| `visual_slam/tracking/vo_pose`            | `geometry_msgs::msg::PoseStamped`                         | Current pose of the `base_link`.                  |
| `visual_slam/tracking/slam_path`          | `nav_msgs::msg::Path`                                     | Trail of poses generated by SLAM.                 |
| `visual_slam/tracking/vo_path`            | `nav_msgs::msg::Path`                                     | Trail of poses generated by pure Visual Odometry. |
| `visual_slam/status`                      | `isaac_ros_visual_slam_interfaces::msg::VisualSlamStatus` | Status message for diagnostics.                   |

| Parameters                      | Description                                                                                                                                                                                                                                                                                 |
| ------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `denoise_input_images`          | If enabled, input images are denoised. It can be enabled when images are noisy because of low-light conditions. It is disabled by default.                                                                                                                                                  |
| `rectified_images`              | Flag to mark if the images coming in the subscribed topics are rectified or raw. It is enabled by default.                                                                                                                                                                                  |
| `enable_imu`                    | If enabled, IMU data is used. It is disabled by default.                                                                                                                                                                                                                                    |
| `enable_debug_mode`             | If enabled, a debug dump(image frames, timestamps, and camera info) is saved on to the disk at the path indicated by `debug_dump_path`. It is disabled by default.                                                                                                                          |
| `debug_dump_path`               | The path to the directory to store the debug dump data. The default value is `/tmp/elbrus`.                                                                                                                                                                                                 |
| `input_base_frame`              | Name of the frame (baselink) to calculate transformation between the baselink and the left camera. Default is empty, which means the value of the `base_frame` will be used. If `input_base_frame` and `base_frame` are both empty, the left camera is assumed to be in the robot's center. |
| `input_left_camera_frame`       | The name of the left camera frame. the default value is empty(""), which means the left camera is in the robot's center and `left_pose_right` will be calculated from CameraInfo.                                                                                                           |
| `input_right_camera_frame`      | The name of the right camera frame. The default value is empty(""), which means left and right cameras have identity rotation and are horizontally aligned. `left_pose_right` will be calculated from CameraInfo.                                                                           |
| `input_imu_frame`               | Defines the name of the IMU frame used to calculate `left_camera_pose_imu`. The default value is `imu`.                                                                                                                                                                                     |
| `gravitational_force`           | The initial gravity vector defined in the odometry frame. If the IMU sensor is not parallel to the floor, update all the axes with appropriate values. The default value is `{0.0, 0, -9.8}`.                                                                                               |
| `publish_tf`                    | If enabled, it will publish output frame hierarchy to TF tree. The default value is `true`.                                                                                                                                                                                                 |
| `map_frame`                     | The frame name associated with the map origin. The default value is `map`.                                                                                                                                                                                                                  |
| `odom_frame`                    | The frame name associated with the odometry origin. The default value is `odom`.                                                                                                                                                                                                            |
| `base_frame`                    | The frame name associated with the robot. The default value is `base_link`.                                                                                                                                                                                                                 |
| `enable_observations_view`      | If enabled, 2D feature pointcloud will be available for visualization. The default value is `false`.                                                                                                                                                                                        |
| `enable_landmarks_view`         | If enabled, landmark pointcloud will be available for visualization. The default value is `false`.                                                                                                                                                                                          |
| `enable_slam_visualization`     | Main flag to enable or disable visualization. The default value is `false`.                                                                                                                                                                                                                 |
| `enable_localization_n_mapping` | If enabled, SLAM mode is on. If diabled, only Visual Odometry is on. The default value is `true`.                                                                                                                                                                                           |
| `path_max_size`                 | The maximum size of the buffer for pose trail visualization.                                                                                                                                                                                                                                |

| Service                         | Type                                                     | Description                                                  |
| ------------------------------- | -------------------------------------------------------- | ------------------------------------------------------------ |
| `visual_slam/reset`             | `isaac_ros_visual_slam_interfaces::srv::Reset`           | A service to reset the node.                                 |
| `visual_slam/get_all_poses`     | `isaac_ros_visual_slam_interfaces::srv::GetAllPoses`     | A service to get the series of poses for the path traversed. |
| `visual_slam/set_odometry_pose` | `isaac_ros_visual_slam_interfaces::srv::SetOdometryPose` | A service to set the pose of the odometry frame.             |


| Action                              | Type                                                           | Description                                                                        |
| ----------------------------------- | -------------------------------------------------------------- | ---------------------------------------------------------------------------------- |
| `visual_slam/save_map`              | `isaac_ros_visual_slam_interfaces::action::SaveMap`            | An action to save the landmarks and pose graph into a map and onto the disk.       |
| `visual_slam/load_map_and_localize` | `isaac_ros_visual_slam_interfaces::action::LoadMapAndLocalize` | An action to load the map from the disk and localize within it given a prior pose. |

</br>

### Launch files

| Name                                      | Description                                                        |
| ----------------------------------------- | ------------------------------------------------------------------ |
| isaac_ros_visual_slam.launch.py           | Launch file to bring up visual SLAM node standalone.               |
| isaac_ros_visual_slam_isaac_sim.launch.py | Launch file to bring up visual SLAM node configured for Isaac Sim. |
| isaac_ros_visual_slam_realsense.launch.py | Launch file to bring up visual SLAM node configured for RealSense. |

</br>

### Interface - msg
### VisualSlamStatus
<pre><code>
# This is a status message which gives insights about tracker status and execution time which can help in diagnostics and profiling.
# The frame id in the header is used as the reference frame of both the transforms below.
std_msgs/Header header

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

# The total time it takes to process the input frames to output messages in seconds.
float64 node_callback_execution_time

# Time it takes to get current pose out of Elbrus using pure visual slam tracking in seconds.
float64 track_execution_time

# Mean time to get poses out of Elbrus using pure visual slam tracking in seconds.
float64 track_execution_time_mean

# Max time to get poses out of Elbrus using pure visual slam tracking in seconds.
float64 track_execution_time_max
</code></pre>

### Interface - service
#### Reset
<pre><code>
# This service restart and reset the node.

# Request
---
# Response
# Indicate successful run of the service
bool success
</code></pre>

#### GetAllPoses
<pre><code>
# Get all the poses after global optimization, in event of a loop closure

# Max number of poses to query 
int32 max_count
---
# Result
# indicate successful run of the service
bool success

# List of optimized poses
geometry_msgs/PoseStamped[] poses
</code></pre>

#### SetOdometryPose
<pre><code>
# This service set up odometry pose.

# Request
geometry_msgs/Pose pose
---
# Response
# Indicate successful run of the service
bool success
</code></pre>

### Interface - action
#### SaveMap
<pre><code>
# Save map to the disk

# Localhost file path to save the map 
string map_url
---
# Result
bool success
string error
---
# Feedback
int32 progress
</code></pre>

#### LoadMapAndLocalize
<pre><code>
# Load map from the file on the disk and try to localize on it.

# Localhost file path of the map 
string map_url

# Initial guess(translation only) of where the robot is in the saved map.
geometry_msgs/Vector3 localize_near_point

---

# Indicates if localization is successful
bool success

# Pose of the localized robot in the map
geometry_msgs/Pose pose

---
# Feedback
int32 progress
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

| Date       | Changes                               |
| ---------- | ------------------------------------- |
| 2022-03-17 | Documentation update for new features |
| 2022-03-11 | Renamed to isaac_ros_visual_slam      |
| 2021-11-15 | Isaac Sim HIL documentation update    |
| 2021-10-20 | Initial release                       |
