# Isaac ROS Visual SLAM

<div align="center"><img src="resources/elbrus_ros_3.gif" width="400px"/></div>

---

## Webinar Available

Learn how to use this package by watching our on-demand webinar: [Pinpoint, 250 fps, ROS 2 Localization with vSLAM on Jetson](https://gateway.on24.com/wcc/experience/elitenvidiabrill/1407606/3998202/isaac-ros-webinar-series)

---

## Overview

This repository provides a high-performance, best-in-class ROS 2 package for VSLAM (visual simultaneous localization and mapping).  This package uses a stereo camera with an IMU to estimate odometry as an input to navigation. It is GPU accelerated to provide real-time, low-latency results in a robotics application. VSLAM provides an additional odometry source for mobile robots (ground based) and can be the primary odometry source for drones.

VSLAM provides a method for visually estimating the position of a robot relative to its start position, known as VO (visual odometry). This is particularly useful in environments where GPS is not available (such as indoors) or intermittent (such as urban locations with structures blocking line of sight to GPS satellites). This method is designed to use left and right stereo camera frames and an IMU (inertial measurement unit) as input. It uses input stereo image pairs to find matching key points in the left and right images; using the baseline between the left and right camera, it can estimate the distance to the key point. Using two consecutive input stereo image pairs, VSLAM can track the 3D motion of key points between the two consecutive images to estimate the 3D motion of the camera--which is then used to compute odometry as an output to navigation. Compared to the classic approach to VSLAM, this method uses GPU acceleration to find and match more key points in real-time, with fine tuning to minimize overall reprojection error.

Key points depend on distinctive features in the left and right camera image that can be repeatedly detected with changes in size, orientation, perspective, lighting, and image noise. In some instances, the number of key points may be limited or entirely absent; for example, if the camera field of view is only looking at a large solid colored wall, no key points may be detected. If there are insufficient key points, this module uses motion sensed with the IMU to provide a sensor for motion, which, when measured, can provide an estimate for odometry. This method, known as VIO (visual-inertial odometry), improves estimation performance when there is a lack of distinctive features in the scene to track motion visually.

SLAM (simultaneous localization and mapping) is built on top of VIO, creating a map of key points that can be used to determine if an area is previously seen. When VSLAM determines that an area is previously seen, it reduces uncertainty in the map estimate, which is known as loop closure. VSLAM uses a statistical approach to loop closure that is more compute efficient to provide a real time solution, improving convergence in loop closure.

<div align="center"><img src="resources/vslam_odometry_nav2_diagram.png" width="800px"/></div>

There are multiple methods for estimating odometry as an input to navigation. None of these methods are perfect; each has limitations  because of systematic flaws in the sensor providing measured observations, such as missing LIDAR returns absorbed by black surfaces, inaccurate wheel ticks when the wheel slips on the ground, or a lack of distinctive features in a scene limiting key points in a camera image. A practical approach to tracking odometry is to use multiple sensors with diverse methods so that systemic issues with one method can be compensated for by another method. With three separate estimates of odometry, failures in a single method can be detected, allowing for fusion of the multiple methods into a single higher quality result. VSLAM provides a vision- and IMU-based solution to estimating odometry that is different from the common practice of using LIDAR and wheel odometry. VSLAM can even be used to improve diversity, with multiple stereo cameras positioned in different directions to provide multiple, concurrent visual estimates of odometry.

To learn more about VSLAM, refer to the [Elbrus SLAM](docs/elbrus-slam.md) documentation.

## Accuracy

VSLAM is a best-in-class package with the lowest translation and rotational error as measured on KITTI [Visual Odometry / SLAM Evaluation 2012](https://www.cvlibs.net/datasets/kitti/eval_odometry.php) for real-time applications.  

| Method                                            | Runtime | Translation | Rotation     | Platform                  |
| ------------------------------------------------- | ------- | ----------- | ------------ | ------------------------- |
| [VSLAM](docs/elbrus-slam.md)                      | 0.007s  | 0.94%       | 0.0019 deg/m | Jetson AGX Xavier aarch64 |
| [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2) | 0.06s   | 1.15%       | 0.0027 deg/m | 2 cores @ >3.5 Ghz x86_64 |

In addition to results from standard benchmarks, we test loop closure for VSLAM on sequences of over 1000 meters, with coverage for indoor and outdoor scenes.

## Performance

The following table summarizes the per-platform performance statistics of sample graphs that use this package, with links included to the full benchmark output. These benchmark configurations are taken from the [Isaac ROS Benchmark](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark#list-of-isaac-ros-benchmarks) collection, based on the [`ros2_benchmark`](https://github.com/NVIDIA-ISAAC-ROS/ros2_benchmark) framework.

| Sample Graph                                                                                                                         | Input Size | AGX Orin                                                                                                                                       | Orin NX                                                                                                                                       | Orin Nano 8GB                                                                                                                                       | x86_64 w/ RTX 3060 Ti                                                                                                                                   |
| ------------------------------------------------------------------------------------------------------------------------------------ | ---------- | ---------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [Visual SLAM Node](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/main/scripts//isaac_ros_visual_slam_node.py) | 720p       | [228 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/main/results/isaac_ros_visual_slam_node-agx_orin.json)<br>36 ms | [124 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/main/results/isaac_ros_visual_slam_node-orin_nx.json)<br>39 ms | [116 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/main/results/isaac_ros_visual_slam_node-orin_nano_8gb.json)<br>85 ms | [238 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/main/results/isaac_ros_visual_slam_node-x86_64_rtx_3060Ti.json)<br>50 ms |

## Commercial Support & Source Access

For commercial support and access to source code, please [contact NVIDIA](https://developer.nvidia.com/isaac-platform-contact-us-form).

## Table of Contents

- [Isaac ROS Visual SLAM](#isaac-ros-visual-slam)
  - [Webinar Available](#webinar-available)
  - [Overview](#overview)
  - [Accuracy](#accuracy)
  - [Performance](#performance)
  - [Commercial Support \& Source Access](#commercial-support--source-access)
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

Update 2022-03-23: Update Elbrus library and performance improvements

## Supported Platforms

This package is designed and tested to be compatible with ROS 2 Humble running on [Jetson](https://developer.nvidia.com/embedded-computing) or an x86_64 system with an NVIDIA GPU.

> **Note**: Versions of ROS 2 earlier than Humble are **not** supported. This package depends on specific ROS 2 implementation features that were only introduced beginning with the Humble release.

| Platform | Hardware                                                                                                                                                                                                 | Software                                                                                                           | Notes                                                                                                                                                                                   |
| -------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Jetson   | [Jetson Orin](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/) <br> [Jetson Xavier](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-agx-xavier/) | [JetPack 5.1.1](https://developer.nvidia.com/embedded/jetpack)                                                     | For best performance, ensure that [power settings](https://docs.nvidia.com/jetson/archives/r34.1/DeveloperGuide/text/SD/PlatformPowerAndPerformance.html) are configured appropriately. |
| x86_64   | NVIDIA GPU                                                                                                                                                                                               | [Ubuntu 20.04+](https://releases.ubuntu.com/20.04/) <br> [CUDA 11.8+](https://developer.nvidia.com/cuda-downloads) |

### Docker

To simplify development, we strongly recommend leveraging the Isaac ROS Dev Docker images by following [these steps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docs/dev-env-setup.md). This will streamline your development environment setup with the correct versions of dependencies on both Jetson and x86_64 platforms.

> **Note**: All Isaac ROS Quickstarts, tutorials, and examples have been designed with the Isaac ROS Docker images as a prerequisite.

## Coordinate Frames

This section describes the coordinate frames that are involved in the `VisualSlamNode`. The frames discussed below are oriented as follows:

<div align="center"><img src="resources/Axes.png" width="300px"/></div>

1. `input_base_frame`: The name of the frame used to calculate transformation between baselink and left camera. The default value is empty (''), which means the value of ``base_frame_`` will be used. If ``input_base_frame_`` and ``base_frame_`` are both empty, the left camera is assumed to be in the robot's center.
2. `input_left_camera_frame`: The frame associated with left eye of the stereo camera. Note that this is not the same as the optical frame. The default value is empty (''), which means the left camera is in the robot's center and ``left_pose_right`` will be calculated from the CameraInfo message.
3. `input_right_camera_frame`: The frame associated with the right eye of the stereo camera. Note that this is not the same as the optical frame. The default value is empty (''), which means left and right cameras have identity rotation and are horizontally aligned, so ``left_pose_right`` will be calculated from CameraInfo.
4. `input_imu_frame`: The frame associated with the IMU sensor (if available). It is used to calculate ``left_pose_imu``.

## Quickstart

1. Set up your development environment by following the instructions [here](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common/blob/main/docs/dev-env-setup.md).

    > Note: `${ISAAC_ROS_WS}` is defined to point to either `/ssd/workspaces/isaac_ros-dev/` or `~/workspaces/isaac_ros-dev/`.

2. Clone this repository and its dependencies under `${ISAAC_ROS_WS}/src`.

    ```bash
    cd ${ISAAC_ROS_WS}/src
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common
    git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros
    ```

3. \[Terminal 1\] Launch the Docker container

    ```bash
    cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
      ./scripts/run_dev.sh ${ISAAC_ROS_WS}
    ```

4. \[Terminal 1\] Inside the container, build and source the workspace:

    ```bash
    cd /workspaces/isaac_ros-dev && \
      colcon build --symlink-install && \
      source install/setup.bash
    ```

    > (Optional) Run tests to verify complete and correct installation:  
    >
    >    ```bash
    >    colcon test --executor sequential
    >    ```

    Run the following launch files in the current terminal (Terminal 1):

    ```bash
    ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py
    ```

5. \[Terminal 2\] Attach another terminal to the running container for Rviz2

    Attach another terminal to the running container for RViz2.

    ```bash
    cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
      ./scripts/run_dev.sh ${ISAAC_ROS_WS}
    ```

    From this second terminal, run Rviz2 to display the output:

    ```bash
    source /workspaces/isaac_ros-dev/install/setup.bash && \
      rviz2 -d src/isaac_ros_visual_slam/isaac_ros_visual_slam/rviz/default.cfg.rviz 
    ```

    > If you are SSH-ing in from a remote machine, the Rviz2 window should be forwarded to your remote machine.

6. \[Terminal 3\] Attach the 3rd terminal to start the rosbag

    ```bash
    cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
      ./scripts/run_dev.sh ${ISAAC_ROS_WS}
    ```

    Run the rosbag file to start the demo.

    ```bash
    source /workspaces/isaac_ros-dev/install/setup.bash && \
      ros2 bag play src/isaac_ros_visual_slam/isaac_ros_visual_slam/test/test_cases/rosbags/small_pol_test/
    ```

7. Result:

    Rviz should start displaying the point clouds and poses like below:

    <div align="center"><img src="resources/Rviz_quick_start.png" width="600px"/></div>  

  > Note: By default, IMU data is ignored. To use IMU data in Isaac ROS Visual Slam, please set `enable_imu` param in your launch file to `True`. See example [below](#tutorial-realsense-imu).

## Next Steps

### Try More Examples

To continue your exploration, check out the following suggested examples:

- <span id="tutorial-isaac-sim">[Tutorial with Isaac Sim](docs/tutorial-isaac-sim.md#tutorial-with-isaac-sim)</span>
- <span id="tutorial-realsense-imu">[Tutorial for Visual SLAM using a RealSense camera with integrated IMU](docs/tutorial-realsense.md#tutorial-for-visual-slam-using-a-realsense-camera)</span>

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

1. If the target frames in rviz are being updated at a **lower rate** than the input image rate:
   1. Check if the input image frame rate is equal to the value set in the launch file by opening a new terminal and running the command

      ```bash
      ros2 topic hz --window 100 <image_topic_name>
      ```

2. If RViz is not showing the poses, check the **Fixed Frame** value.
3. If you are seeing `Tracker is lost.` messages frequently, it could be caused by the following issues:
   1. Fast motion causing the motion blur in the frames
   2. Low-lighting conditions.
   3. The wrong `camerainfo` is being published.
4. For better performance:
   1. Increase the capture framerate from the camera to yield a better tracking result.
   2. If input images are noisy, you can use the `denoise_input_images` flag in the node.
5. For RealSense cameras-
   1. The firmware of the RealSense camera is [5.14.0 or newer](https://dev.intelrealsense.com/docs/firmware-releases). Check the RealSense [firmware-update-tool](https://dev.intelrealsense.com/docs/firmware-update-tool) page for details on how to check the version and update the camera firmware.
   2. You are using the [4.51.1 tagged release](https://github.com/IntelRealSense/realsense-ros/releases/tag/4.51.1) for the [realsense-ros package](https://github.com/IntelRealSense/realsense-ros/commit/2a65533ee7431bdc05fe5744798efc7f5713f866).
   3. You are using the parameters set in the [RealSense tutorial launch file](./isaac_ros_visual_slam/launch/isaac_ros_visual_slam_realsense.launch.py)
   4. If the pose estimate frames in RViz is **drifting** from actual real-world poses and you see white dots on nearby objects(refer to the screenshot below), this means that the emitter of the RealSense camera is on and it needs to be [turned off](./isaac_ros_visual_slam/launch/isaac_ros_visual_slam_realsense.launch.py#L35).
   5. Left and right images being published are not compressed or encoded. Isaac ROS Visual Slam expects raw images.

<div align="center"><img src="resources/realsense_emitter.png" width="400px"/></div>

## Updates

| Date       | Changes                                            |
| ---------- | -------------------------------------------------- |
| 2023-04-05 | Update Elbrus library and performance improvements |
| 2022-10-19 | Updated OSS licensing                              |
| 2022-08-31 | Update to be compatible with JetPack 5.0.2         |
| 2022-06-30 | Support for ROS 2 Humble                           |
| 2022-03-17 | Documentation update for new features              |
| 2022-03-11 | Renamed to isaac_ros_visual_slam                   |
| 2021-11-15 | Isaac Sim HIL documentation update                 |
| 2021-10-20 | Initial release                                    |
