# Isaac ROS Visual SLAM

NVIDIA-accelerated, simultaneous localization and mapping (SLAM) using stereo
visual inertial odometry (SVIO).

<div align="center"><a class="reference internal image-reference" href="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/main/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_visual_slam/cuvslam_ros_3.gif/"><img alt="image" src="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/main/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_visual_slam/cuvslam_ros_3.gif/" width="800px"/></a></div>

---

## Webinar Available

Learn how to use this package by watching our on-demand webinar:
[Pinpoint, 250 fps, ROS 2 Localization with vSLAM on
Jetson](https://gateway.on24.com/wcc/experience/elitenvidiabrill/1407606/3998202/isaac-ros-webinar-series)

---

## Overview

[Isaac ROS Visual SLAM](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam)
provides a high-performance, best-in-class ROS 2 package
for VSLAM (visual simultaneous localization and mapping). This package
uses one or more stereo cameras and optionally an IMU to estimate
odometry as an input to navigation. It is GPU accelerated to provide
real-time, low-latency results in a robotics application. VSLAM
provides an additional odometry source for mobile robots
(ground based) and can be the primary odometry source for drones.

VSLAM provides a method for visually estimating the position of a robot
relative to its start position, known as VO (visual odometry). This is
particularly useful in environments where GPS is not available (such as
indoors) or intermittent (such as urban locations with structures
blocking line of sight to GPS satellites). This method is designed to
use multiple stereo camera frames and an IMU (inertial measurement
unit) as input. It uses stereo image pairs to find matching key
points. Using the baseline between the camera pairs, it can estimate
the distance to the key point. Using consecutive images, VSLAM
can track the motion of key points to estimate the 3D motion of the
camera-which is then used to compute odometry as an output
to navigation. Compared to the classic approach to VSLAM, this method
uses GPU acceleration to find and match more key points in real-time,
with fine tuning to minimize overall reprojection error.

Key points depend on distinctive features in the images
that can be repeatedly detected with changes in size, orientation,
perspective, lighting, and image noise. In some instances, the number of
key points may be limited or entirely absent; for example, if the camera
field of view is only looking at a large solid colored wall, no key
points may be detected. If there are insufficient key points, this
module uses motion sensed with the IMU to provide a sensor for motion,
which, when measured, can provide an estimate for odometry. This method,
known as VIO (visual-inertial odometry), improves estimation performance
when there is a lack of distinctive features in the scene to track
motion visually.

SLAM (simultaneous localization and mapping) is built on top of VIO,
creating a map of key points that can be used to determine if an area is
previously seen. When VSLAM determines that an area is previously seen,
it reduces uncertainty in the map estimate, which is known as loop
closure. VSLAM uses a statistical approach to loop closure that is more
compute efficient to provide a real time solution, improving convergence
in loop closure.

<div align="center"><a class="reference internal image-reference" href="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/main/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_visual_slam/vslam_odometry_nav2_diagram.png/"><img alt="image" src="https://media.githubusercontent.com/media/NVIDIA-ISAAC-ROS/.github/main/resources/isaac_ros_docs/repositories_and_packages/isaac_ros_visual_slam/vslam_odometry_nav2_diagram.png/" width="880px"/></a></div>

There are multiple methods for estimating odometry as an input to
navigation. None of these methods are perfect; each has limitations
because of systematic flaws in the sensor providing measured
observations, such as missing LIDAR returns absorbed by black surfaces,
inaccurate wheel ticks when the wheel slips on the ground, or a lack of
distinctive features in a scene limiting key points in a camera image. A
practical approach to tracking odometry is to use multiple sensors with
diverse methods so that systemic issues with one method can be
compensated for by another method. With three separate estimates of
odometry, failures in a single method can be detected, allowing for
fusion of the multiple methods into a single higher quality result.
VSLAM provides a vision- and IMU-based solution to estimating odometry
that is different from the common practice of using LIDAR and wheel
odometry. VSLAM can even be used to improve diversity, with multiple
stereo cameras positioned in different directions to provide multiple,
concurrent visual estimates of odometry.

To learn more about VSLAM, refer to the
[cuVSLAM SLAM](https://nvidia-isaac-ros.github.io/concepts/visual_slam/cuvslam/index.html) documentation.

## Accuracy

VSLAM is a best-in-class package with the lowest translation and
rotational error as measured on KITTI [Visual Odometry / SLAM Evaluation
2012](https://www.cvlibs.net/datasets/kitti/eval_odometry.php) for
real-time applications.

| Method                                                                              | Runtime   | Translation   | Rotation     | Platform                    |
|-------------------------------------------------------------------------------------|-----------|---------------|--------------|-----------------------------|
| [VSLAM](https://nvidia-isaac-ros.github.io/concepts/visual_slam/cuvslam/index.html) | 0.007s    | 0.94%         | 0.0019 deg/m | Jetson AGX Xavier `aarch64` |
| [ORB-SLAM2](https://github.com/raulmur/ORB_SLAM2)                                   | 0.06s     | 1.15%         | 0.0027 deg/m | 2 cores @ >3.5 GHz `x86_64` |

In addition to results from standard benchmarks, we test loop closure
for VSLAM on sequences of over 1000 meters, with coverage for indoor and
outdoor scenes.

## Performance

| Sample Graph<br/><br/>                                                                                                                                                                                                           | Input Size<br/><br/>      | Nova Carter<br/><br/>                                                                                                                                   |
|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------------------------|---------------------------------------------------------------------------------------------------------------------------------------------------------|
| [Multicam Visual SLAM Live Graph](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/main/benchmarks/isaac_ros_perceptor_nova_benchmark/scripts/isaac_ros_visual_slam_graph.py)<br/><br/><br/>4 Hawk Cameras<br/><br/> | 1200p<br/><br/><br/><br/> | [30.0 fps](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_benchmark/blob/main/results/isaac_ros_4_hawk_vslam_graph-carter_v2.4.json)<br/><br/><br/><br/> |

> [!Note]
> This benchmark can only be run on a [Nova Orin](https://developer.nvidia.com/isaac/nova-orin) compatible system.

---

## Documentation

Please visit the [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/index.html) to learn how to use
this repository.

---

## Packages

* [`isaac_ros_visual_slam`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_visual_slam/index.html)
  * [Quickstart](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_visual_slam/index.html#quickstart)
  * [Try More Examples](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_visual_slam/index.html#try-more-examples)
  * [Coordinate Frames](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_visual_slam/index.html#coordinate-frames)
  * [Troubleshooting](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_visual_slam/index.html#troubleshooting)
  * [API](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_visual_slam/index.html#api)
* [`isaac_ros_visual_slam_interfaces`](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/isaac_ros_visual_slam_interfaces/index.html)

## Latest

Update 2024-09-26: Update for ZED compatibility
