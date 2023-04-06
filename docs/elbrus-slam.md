# Elbrus SLAM

All SLAM-related operations work in parallel to visual odometry in a separate thread. Input images get copied into GPU and then Elbrus starts tracking.

Elbrus uses the following:

* 2D features on the input images
* Landmarks: The patch of pixels in the source images with coordinates of the feature associated with the position of the camera from where that feature is visible.
* PoseGraph: A graph that tracks the poses of the camera from where the landmarks are viewed.

Landmarks and PoseGraph are stored in a data structure that doesn't increase its size in the case where the same landmark is visited more than once.

Imagine a robot moving around and returning to the same place. Since odometry always has some accumulated drift, we see a deviation in trajectory from the ground truth. SLAM can be used to solve this problem.

During the robot's motion, SLAM stores all landmarks and it continuously verifies if each landmark has been seen before. When Elbrus recognizes several landmarks, it determines their position from the data structure.

At this moment, a connection is added to the PoseGraph which makes a loop from the free trail of the poses; this event is termed 'loop closure'. Immediately after that, Elbrus performs a graph optimization that leads to the correction of the current odometric pose and all previous poses in the graph.

The procedure for adding landmarks is designed such that if we do not see a landmark in the place where it was expected, then such landmarks are marked for eventual deletion. This allows you to use Elbrus over a changing terrain.

Along with visual data, Elbrus can use Inertial Measurement Unit (IMU) measurements. It automatically switches to IMU when VO is unable to estimate a pose – for example, when there is dark lighting or long solid surfaces in front of a camera. Using an IMU usually leads to a significant performance improvement in cases of poor visual conditions.

In case of severe degradation of image input (lights being turned off, dramatic motion blur on a bump while driving, and other possible scenarios), below mentioned motion estimation algorithms will ensure acceptable quality for pose tracking:

* The IMU readings integrator provides acceptable pose tracking quality for about ~< 1 seconds.

* In case of IMU failure, the constant velocity integrator continues to provide the last linear and angular velocities reported by Stereo VIO before failure. 
This provides acceptable pose tracking quality for ~0.5 seconds.

## List of Useful Visualizations

* `visual_slam/vis/observations_cloud` - Point cloud for 2D Features
* `visual_slam/vis/landmarks_cloud` - All mapped landmarks
* `visual_slam/vis/loop_closure_cloud` - Landmarks visible in last loop closure
* `visual_slam/vis/pose_graph_nodes` - Pose graph nodes
* `visual_slam/vis/pose_graph_edges` - Pose graph edges

## Saving the map

Naturally, we would like to save the stored landmarks and pose graph in a map. We have implemented a ROS 2 action to save the map to the disk called `SaveMap`.

## Loading and Localization in the Map

Once the map has been saved to the disk, it can be used later to localize the robot. To load the map into the memory, we have made a ROS 2 action called `LoadMapAndLocalize`. It requires a map file path and a prior pose, which is an initial guess of where the robot is in the map. Given the prior pose and current set of camera frames, Elbrus tries to find the pose of the landmarks in the requested map that matches the current set. If the localization is successful, Elbrus will load the map in the memory. Otherwise, it will continue building a new map.

Both `SaveMap` and `LoadMapAndLocalize` can take some time to complete. Hence, they are designed to be asynchronous to avoid interfering with odometry calculations.
