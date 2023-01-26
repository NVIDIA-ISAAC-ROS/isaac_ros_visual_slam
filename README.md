# Isaac ROS Visual SLAM

<div align="center"><img src="resources/elbrus_ros_3.gif" width="400px"/></div>

## Overview

This repository provides a ROS2 package that performs stereo visual simultaneous localization and mapping (VSLAM) and estimates stereo visual inertial odometry using the [Isaac Elbrus](https://docs.nvidia.com/isaac/packages/visual_slam/doc/elbrus_visual_slam.html) GPU-accelerated library. It takes in a time-synced pair of stereo images (grayscale) along with respective camera intrinsics to publish the current pose of the camera relative to its start pose.

Elbrus is based on two core technologies: Visual Odometry (VO) and Simultaneous Localization and Mapping (SLAM).

Visual SLAM is a method for estimating a camera position relative to its start position. This method has an iterative nature. At each iteration, it considers two consequential input frames (stereo pairs). On both the frames, it finds a set of keypoints. Matching keypoints in these two sets gives the ability to estimate the transition and relative rotation of the camera between frames.

Simultaneous Localization and Mapping is a method built on top of the VO predictions. It aims to improve the quality of VO estimations by leveraging the knowledge of previously seen parts of a trajectory. It detects if the current scene was seen in the past (i.e. a loop in camera movement) and runs an additional optimization procedure to tune previously obtained poses.

Along with visual data, Elbrus can optionally use Inertial Measurement Unit (IMU) measurements. It automatically switches to IMU when VO is unable to estimate a pose; for example, when there is dark lighting or long solid featureless surfaces in front of a camera. Elbrus delivers real-time tracking performance: more than 60 FPS for VGA resolution. For the KITTI benchmark, the algorithm achieves a drift of ~1% in localization and an orientation error of 0.003 degrees per meter of motion. Elbrus allows for robust tracking in various environments and with different use cases: indoor, outdoor, aerial, HMD, automotive, and robotics.

To learn more about Elbrus SLAM click [here](docs/elbrus-slam.md).

## Performance

The following are the benchmark performance results of the prepared pipelines in this package, by supported platform:

| Pipeline | AGX Orin           | Orin Nano         | x86_64 w/ RTX 3060 Ti |
| -------- | ------------------ | ----------------- | --------------------- |
| VSLAM    | 250 fps <br> 3.1ms | 105 fps <br> 10ms | 265 fps <br> 5.2ms    |

These data have been collected per the methodology described [here](https://github.com/NVIDIA-ISAAC-ROS/.github/blob/main/profile/performance-summary.md#methodology).

## Commercial Support & Source Access
For commercial support and access to source code, please [contact NVIDIA](https://developer.nvidia.com/isaac-platform-contact-us-form).

## Webinar Available
Learn how to use this package by watching our on-demand webinar: [Pinpoint, 250 fps, ROS 2 Localization with vSLAM on Jetson](https://gateway.on24.com/wcc/experience/elitenvidiabrill/1407606/3998202/isaac-ros-webinar-series)

## Table of Contents

- [Isaac ROS Visual SLAM](#isaac-ros-visual-slam)
  - [Overview](#overview)
  - [Performance](#performance)
  - [Commercial Support \& Source Access](#commercial-support--source-access)
  - [Webinar Available](#webinar-available)
  - [Table of Contents](#table-of-contents)
  - [Latest Update](#latest-update)
  - [Supported Platforms](#supported-platforms)
    - [Docker](#docker)
  - [Coordinate Frames](#coordinate-frames)
  - [Quickstart](#quickstart)
  - [Next Steps](#next-steps)
    - [Try More Examples](#try-more-examples)
    - [Customize your Dev Environment](#customize-your-dev-environment)
  - [Package Reference](#package-reference)
    - [isaac\_ros\_visual\_slam](#isaac_ros_visual_slam)
      - [Usage](#usage)
      - [ROS Parameters](#ros-parameters)
      - [ROS Topics Subscribed](#ros-topics-subscribed)
      - [ROS Topics Published](#ros-topics-published)
      - [ROS Services Advertised](#ros-services-advertised)
      - [ROS Actions Advertised](#ros-actions-advertised)
  - [Troubleshooting](#troubleshooting)
    - [Isaac ROS Troubleshooting](#isaac-ros-troubleshooting)
    - [Troubleshooting Suggestions](#troubleshooting-suggestions)
  - [Updates](#updates)

## Latest Update

Update 2022-10-19: Updated OSS licensing

## Supported Platforms

This package is designed and tested to be compatible with ROS2 Humble running on [Jetson](https://developer.nvidia.com/embedded-computing) or an x86_64 system with an NVIDIA GPU.

> **Note**: Versions of ROS2 earlier than Humble are **not** supported. This package depends on specific ROS2 implementation features that were only introduced beginning with the Humble release.

| Platform | Hardware                                                                                                                                                                                                 | Software                                                                                                             | Notes                                                                                                                                                                                   |
| -------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Jetson   | [Jetson Orin](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/) <br> [Jetson Xavier](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-agx-xavier/) | [JetPack 5.0.2](https://developer.nvidia.com/embedded/jetpack)                                                       | For best performance, ensure that [power settings](https://docs.nvidia.com/jetson/archives/r34.1/DeveloperGuide/text/SD/PlatformPowerAndPerformance.html) are configured appropriately. |
| x86_64   | NVIDIA GPU                                                                                                                                                                                               | [Ubuntu 20.04+](https://releases.ubuntu.com/20.04/) <br> [CUDA 11.6.1+](https://developer.nvidia.com/cuda-downloads) |

### Docker

To simplify development, we strongly recommend leveraging the Isaac ROS Dev Docker images by following [these steps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docs/dev-env-setup.md). This will streamline your development environment setup with the correct versions of dependencies on both Jetson and x86_64 platforms.

> **Note:** All Isaac ROS Quickstarts, tutorials, and examples have been designed with the Isaac ROS Docker images as a prerequisite.

## Coordinate Frames

This section describes the coordinate frames that are involved in the `VisualSlamNode`. The frames discussed below are oriented as follows:

<div align="center"><img src="resources/Axes.png" width="300px"/></div>

1. `input_base_frame`: The name of the frame used to calculate transformation between baselink and left camera. The default value is empty (''), which means the value of ``base_frame_`` will be used. If ``input_base_frame_`` and ``base_frame_`` are both empty, the left camera is assumed to be in the robot's center.
2. `input_left_camera_frame`: The frame associated with left eye of the stereo camera. Note that this is not the same as the optical frame. The default value is empty (''), which means the left camera is in the robot's center and ``left_pose_right`` will be calculated from the CameraInfo message.
3. `input_right_camera_frame`: The frame associated with the right eye of the stereo camera. Note that this is not the same as the optical frame. The default value is empty (''), which means left and right cameras have identity rotation and are horizontally aligned, so ``left_pose_right`` will be calculated from CameraInfo.
4. `input_imu_frame`: The frame associated with the IMU sensor (if available). It is used to calculate ``left_pose_imu``.

## Quickstart

1. Set up your development environment by following the instructions [here](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docs/dev-env-setup.md).  
2. Clone this repository and its dependencies under `~/workspaces/isaac_ros-dev/src`.

    ```bash
    cd ~/workspaces/isaac_ros-dev/src
    ```

    ```bash
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam
    ```

      ```bash
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common
    ```

    ```bash
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros
    ```

3. Pull down a ROS Bag of sample data:

    ```bash
    cd ~/workspaces/isaac_ros-dev/src/isaac_ros_visual_slam && \ 
      git lfs pull -X "" -I isaac_ros_visual_slam/test/test_cases/rosbags/
    ```

4. Launch the Docker container using the `run_dev.sh` script:

    ```bash
    cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common && \
      ./scripts/run_dev.sh
    ```

5. Inside the container, build and source the workspace:

    ```bash
    cd /workspaces/isaac_ros-dev && \
      colcon build --symlink-install && \
      source install/setup.bash
    ```

6. (Optional) Run tests to verify complete and correct installation:  

    ```bash
    colcon test --executor sequential
    ```

7. Run the following launch files in the current terminal:

    ```bash
    ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py
    ```

8. In a second terminal inside the Docker container, prepare rviz to display the output:

    ```bash
    source /workspaces/isaac_ros-dev/install/setup.bash && \
      rviz2 -d src/isaac_ros_visual_slam/isaac_ros_visual_slam/rviz/default.cfg.rviz 
    ```

9. In an another terminal inside the Docker container, run the following ros bag file to start the demo:

    ```bash
    source /workspaces/isaac_ros-dev/install/setup.bash && \
      ros2 bag play src/isaac_ros_visual_slam/isaac_ros_visual_slam/test/test_cases/rosbags/small_pol_test/
    ```

    Rviz should start displaying the point clouds and poses like below:

  <div align="center"><img src="resources/Rviz_quick_start.png" width="600px"/></div>  

## Next Steps

### Try More Examples

To continue your exploration, check out the following suggested examples:

- [Tutorial with Isaac Sim](docs/tutorial-isaac-sim.md#tutorial-with-isaac-sim)

### Customize your Dev Environment

To customize your development environment, reference [this guide](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docs/modify-dockerfile.md).

## Package Reference

### isaac_ros_visual_slam

#### Usage

```bash
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py
```

#### ROS Parameters

| ROS Parameter                   | Type                  | Default          | Description                                                                                                                                                                                                                                                                                 |
| ------------------------------- | --------------------- | ---------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `denoise_input_images`          | `bool`                | `false`          | If enabled, input images are denoised. It can be enabled when images are noisy because of low-light conditions.                                                                                                                                                                             |
| `rectified_images`              | `bool`                | `true`           | Flag to mark if the incoming images are rectified or raw.                                                                                                                                                                                                                                   |
| `enable_imu`                    | `bool`                | `false`          | If enabled, IMU data is used.                                                                                                                                                                                                                                                               |
| `enable_debug_mode`             | `bool`                | `false`          | If enabled, a debug dump (image frames, timestamps, and camera info) is saved on to the disk at the path indicated by `debug_dump_path`                                                                                                                                                     |
| `debug_dump_path`               | `std::string`         | `/tmp/elbrus`    | The path to the directory to store the debug dump data.                                                                                                                                                                                                                                     |
| `input_base_frame`              | `std::string`         | `""`             | Name of the frame (baselink) to calculate transformation between the baselink and the left camera. Default is empty, which means the value of the `base_frame` will be used. If `input_base_frame` and `base_frame` are both empty, the left camera is assumed to be in the robot's center. |
| `input_left_camera_frame`       | `std::string`         | `""`             | The name of the left camera frame. the default value is empty, which means the left camera is in the robot's center and `left_pose_right` will be calculated from CameraInfo.                                                                                                               |
| `input_right_camera_frame`      | `std::string`         | `""`             | The name of the right camera frame. The default value is empty, which means left and right cameras have identity rotation and are horizontally aligned. `left_pose_right` will be calculated from CameraInfo.                                                                               |
| `input_imu_frame`               | `std::string`         | `imu`            | Defines the name of the IMU frame used to calculate `left_camera_pose_imu`.                                                                                                                                                                                                                 |
| `gravitational_force`           | `std::vector<double>` | `{0.0, 0, -9.8}` | The initial gravity vector defined in the odometry frame. If the IMU sensor is not parallel to the floor, update all the axes with appropriate values.                                                                                                                                      |
| `publish_tf`                    | `bool`                | `true`           | If enabled, it will publish output frame hierarchy to TF tree.                                                                                                                                                                                                                              |
| `map_frame`                     | `std::string`         | `map`            | The frame name associated with the map origin.                                                                                                                                                                                                                                              |
| `odom_frame`                    | `std::string`         | `odom`           | The frame name associated with the odometry origin.                                                                                                                                                                                                                                         |
| `base_frame`                    | `std::string`         | `base_link`      | The frame name associated with the robot.                                                                                                                                                                                                                                                   |
| `enable_observations_view`      | `bool`                | `false`          | If enabled, 2D feature pointcloud will be available for visualization.                                                                                                                                                                                                                      |
| `enable_landmarks_view`         | `bool`                | `false`          | If enabled, landmark pointcloud will be available for visualization.                                                                                                                                                                                                                        |
| `enable_slam_visualization`     | `bool`                | `false`          | Main flag to enable or disable visualization.                                                                                                                                                                                                                                               |
| `enable_localization_n_mapping` | `bool`                | `true`           | If enabled, SLAM mode is on. If diabled, only Visual Odometry is on.                                                                                                                                                                                                                        |
| `path_max_size`                 | `int`                 | `1024`           | The maximum size of the buffer for pose trail visualization.                                                                                                                                                                                                                                |
| `publish_odom_to_base_tf`       | `bool`                | `true`           | Enable tf broadcaster for odom_frame->base_frame transform.                                                                                                                                                                                                                                 |
| `publish_map_to_odom_tf`        | `bool`                | `true`           | Enable tf broadcaster for map_frame->odom_frame transform.                                                                                                                                                                                                                                  |
| `invert_odom_to_base_tf`        | `bool`                | `false`          | Invert the odom_frame->base_frame transform before broadcasting to the tf tree.                                                                                                                                                                                                             |
| `invert_map_to_odom_tf`         | `bool`                | `false`          | Invert the map_frame->odom_frame transform before broadcasting  to the tf tree.                                                                                                                                                                                                             |
| `map_frame`                     | `std::string`         | `map`            | String name for the map_frame.                                                                                                                                                                                                                                                              |
| `odom_frame`                    | `std::string`         | `odom`           | String name for the odom_frame.                                                                                                                                                                                                                                                             |
| `base_frame`                    | `std::string`         | `base_link`      | String name for the base_frame.                                                                                                                                                                                                                                                             |
| `override_publishing_stamp`     | `bool`                | `false`          | Override timestamp received from the left image with the timetsamp from rclcpp::Clock.                                                                                                                                                                                                      |
| `msg_filter_queue_size`         | `int`                 | `100`            | Image + Camera Info Synchronizer message filter queue size.                                                                                                                                                                                                                                 |
| `image_qos`                     | `std::string`         | `SENSOR_DATA`    | QoS profile for the left and right image subscribers.                                                                                                                                                                                                                                       |

#### ROS Topics Subscribed

| ROS Topic                          | Interface                                                                                                        | Description                                                     |
| ---------------------------------- | ---------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------- |
| `/stereo_camera/left/image`        | [`sensor_msgs/Image`](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/Image.msg)           | The image from the left eye of the stereo camera in grayscale.  |
| `/stereo_camera/left/camera_info`  | [`sensor_msgs/CameraInfo`](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/CameraInfo.msg) | CameraInfo from the left eye of the stereo camera.              |
| `/stereo_camera/right/image`       | [`sensor_msgs/Image`](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/Image.msg)           | The image from the right eye of the stereo camera in grayscale. |
| `/stereo_camera/right/camera_info` | [`sensor_msgs/CameraInfo`](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/CameraInfo.msg) | CameraInfo from the right eye of the stereo camera.             |
| `visual_slam/imu`                  | [`sensor_msgs/Imu`](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/Imu.msg)               | Sensor data from the IMU(optional).                             |

#### ROS Topics Published

| ROS Topic                                 | Interface                                                                                                                                          | Description                                       |
| ----------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------- |
| `visual_slam/tracking/odometry`           | [`nav_msgs/Odometry`](https://github.com/ros2/common_interfaces/blob/humble/nav_msgs/msg/Odometry.msg)                                             | Odometry messages for the `base_link`.            |
| `visual_slam/tracking/vo_pose_covariance` | [`geometry_msgs/PoseWithCovarianceStamped`](https://github.com/ros2/common_interfaces/blob/humble/geometry_msgs/msg/PoseWithCovarianceStamped.msg) | Current pose with covariance of the `base_link`.  |
| `visual_slam/tracking/vo_pose`            | [`geometry_msgs/PoseStamped`](https://github.com/ros2/common_interfaces/blob/humble/geometry_msgs/msg/PoseStamped.msg)                             | Current pose of the `base_link`.                  |
| `visual_slam/tracking/slam_path`          | [`nav_msgs/Path`](https://github.com/ros2/common_interfaces/blob/humble/nav_msgs/msg/Path.msg)                                                     | Trail of poses generated by SLAM.                 |
| `visual_slam/tracking/vo_path`            | [`nav_msgs/Path`](https://github.com/ros2/common_interfaces/blob/humble/nav_msgs/msg/Path.msg)                                                     | Trail of poses generated by pure Visual Odometry. |
| `visual_slam/status`                      | [`isaac_ros_visual_slam_interfaces/VisualSlamStatus`](isaac_ros_visual_slam_interfaces/msg/VisualSlamStatus.msg)                                   | Status message for diagnostics.                   |

#### ROS Services Advertised

| ROS Service                     | Interface                                                                                                      | Description                                                  |
| ------------------------------- | -------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------ |
| `visual_slam/reset`             | [`isaac_ros_visual_slam_interfaces/Reset`](isaac_ros_visual_slam_interfaces/srv/Reset.srv)                     | A service to reset the node.                                 |
| `visual_slam/get_all_poses`     | [`isaac_ros_visual_slam_interfaces/GetAllPoses`](isaac_ros_visual_slam_interfaces/srv/GetAllPoses.srv)         | A service to get the series of poses for the path traversed. |
| `visual_slam/set_odometry_pose` | [`isaac_ros_visual_slam_interfaces/SetOdometryPose`](isaac_ros_visual_slam_interfaces/srv/SetOdometryPose.srv) | A service to set the pose of the odometry frame.             |

#### ROS Actions Advertised

| ROS Action                          | Interface                                                                                                                  | Description                                                                        |
| ----------------------------------- | -------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------- |
| `visual_slam/save_map`              | [`isaac_ros_visual_slam_interfaces/SaveMap`](isaac_ros_visual_slam_interfaces/action/SaveMap.action)                       | An action to save the landmarks and pose graph into a map and onto the disk.       |
| `visual_slam/load_map_and_localize` | [`isaac_ros_visual_slam_interfaces/LoadMapAndLocalize`](isaac_ros_visual_slam_interfaces/action/LoadMapAndLocalize.action) | An action to load the map from the disk and localize within it given a prior pose. |

## Troubleshooting

### Isaac ROS Troubleshooting

For solutions to problems with Isaac ROS, please check [here](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docs/troubleshooting.md).

### Troubleshooting Suggestions

- If RViz is not showing the poses, check the **Fixed Frame** value.
- If you are seeing `Tracker is lost.` messages frequently, it could be caused by the following issues:
  - Fast motion causing the motion blur in the frames.
  - Low-lighting conditions.
  - The wrong `camerainfo` is being published.
- For better performance:
  - Increase the capture framerate from the camera to yield a better tracking result.
  - If input images are noisy, you can use the `denoise_input_images` flag in the node.

## Updates

| Date       | Changes                                    |
| ---------- | ------------------------------------------ |
| 2022-10-19 | Updated OSS licensing                      |
| 2022-08-31 | Update to be compatible with JetPack 5.0.2 |
| 2022-06-30 | Support for ROS2 Humble                    |
| 2022-03-17 | Documentation update for new features      |
| 2022-03-11 | Renamed to isaac_ros_visual_slam           |
| 2021-11-15 | Isaac Sim HIL documentation update         |
| 2021-10-20 | Initial release                            |
