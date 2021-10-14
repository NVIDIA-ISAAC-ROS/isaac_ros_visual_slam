/**
 * Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#include "isaac_ros_visual_odometry/visual_odometry_node.hpp"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "cv_bridge/cv_bridge.h"
#include "eigen3/Eigen/Dense"
#include "elbrus.h"  // NOLINT - include .h without directory
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/exact_time.h"
#include "rclcpp/parameter_value.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/buffer_client.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace
{

constexpr char kFisheye4[] = "fisheye4";
constexpr char kBrown5k[] = "brown5k";
constexpr char kPinhole[] = "pinhole";
constexpr int32_t kMaxNumCameraParameters = 9;
constexpr int32_t kNumCameras = 2;
constexpr int32_t LEFT_CAMERA_IDX = 0;
constexpr int32_t RIGHT_CAMERA_IDX = 1;

// ROS' DISTORTION MODELS:
// 1.PLUMB_BOB
// 2.RATIONAL_POLYNOMIAL
// ELBRUS' DISTORTION MODELS:
// 1.brown5k
// 2.fisheye4
// ROS' PLUMB_BOB = Elbrus' brown5k
// ROS' RATIONAL_POLYNOMIAL is not supported in Elbrus
enum DistortionModel
{
  BROWN,
  FISHEYE,
  PINHOLE,
  RATIONAL_POLYNOMIAL
};

// Map the encoding desired string to the DistortionModel
const std::unordered_map<std::string, DistortionModel> g_str_to_distortion_model({
    {"plumb_bob", DistortionModel::BROWN},
    {"fisheye", DistortionModel::FISHEYE},
    {"pinhole", DistortionModel::PINHOLE},
    {"rational_polynomial", DistortionModel::RATIONAL_POLYNOMIAL}});
}  // namespace

namespace isaac_ros
{
namespace visual_odometry
{

struct VisualOdometryNode::VisualOdometryImpl
{
  // Name of the logger that will used to retrieve the logger.
  const std::string logger_name;

  // Clock shared pointer to access clock.
  const std::shared_ptr<rclcpp::Clock> node_clock;

  // Message filter policy to be applied to the synchronizer
  // to synchronize the incoming messages on the topics.
  typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::msg::Image,
      sensor_msgs::msg::CameraInfo,
      sensor_msgs::msg::Image,
      sensor_msgs::msg::CameraInfo
  > ExactTime;
  typedef message_filters::Synchronizer<ExactTime> Synchronizer;
  std::shared_ptr<Synchronizer> sync;

  // Elbrus handle to call Elbrus API.
  ELBRUS_TrackerHandle elbrus_handle = nullptr;

  // Define number of cameras for Elbrus. 2 for stereo camera.
  std::array<ELBRUS_Camera, kNumCameras> elbrus_cameras;

  // Define camera parameters for stereo camera.
  std::array<float, kMaxNumCameraParameters> left_intrinsics, right_intrinsics;

  // Flag to check the status of initialization.
  bool is_initialized = false;

  // Transformation converting from
  // ROS Frame    (x-forward, y-left, z-up) to
  // Elbrus Frame (x-right, y-up, z-backward)
  const tf2::Transform elbrus_pose_base_link;

  // Transformation converting from
  // Elbrus Frame (x-right, y-up, z-backward) to
  // ROS Frame    (x-forward, y-left, z-up)
  const tf2::Transform base_link_pose_elbrus;

  // Buffer for tf2.
  std::unique_ptr<tf2_ros::Buffer> tf_buffer{nullptr};

  // Listener for tf2. It takes in data from tf2 buffer.
  std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};

  // Publisher for tf2.
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_publisher{nullptr};

  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_publisher{nullptr};

  explicit VisualOdometryImpl(VisualOdometryNode & node);

  void Initialize(
    VisualOdometryNode & node,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg_left_ci,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg_right_ci);

  ELBRUS_Configuration getConfiguration(
    const VisualOdometryNode & node,
    const ELBRUS_Pose & imu_from_camera);

  ELBRUS_Pose ToElbrusPose(const tf2::Transform & tf_mat);
  tf2::Transform FromElbrusPose(const ELBRUS_Pose & elbrus_pose);

  void SetElbrusCameraPose(ELBRUS_Camera & cam, const ELBRUS_Pose & elbrus_pose);

  void ReadCameraIntrinsics(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg_ci,
    ELBRUS_Camera & elbrus_camera,
    std::array<float, kMaxNumCameraParameters> & intrinsics);

  ELBRUS_Image ToElbrusImage(int32_t camera_index, const cv::Mat & image, const int64_t & acqtime);

  ELBRUS_ImuMeasurement ToElbrusImuMeasurement(
    const sensor_msgs::msg::Imu::ConstSharedPtr & msg_imu);

  static tf2::Transform ChangeBasis(
    const tf2::Transform & target_pose_source,
    const tf2::Transform & source_pose_source);

  void PublishFrameStaticTransform(
    const tf2::Transform & pose,
    const std::string & target,
    const std::string & source);

  void PublishFrameTransform(
    const tf2::Transform & pose,
    const std::string & target,
    const std::string & source);

  tf2::Transform GetFrameTransform(
    const std::string & target,
    const std::string & source);

  ~VisualOdometryImpl();
};


VisualOdometryNode::VisualOdometryNode(rclcpp::NodeOptions options)
: Node("visual_odometry", options),
  // Node parameters.
  enable_rectified_pose_(declare_parameter<bool>("enable_rectified_pose", true)),
  denoise_input_images_(declare_parameter<bool>("denoise_input_images", false)),
  rectified_images_(declare_parameter<bool>("rectified_images", true)),
  enable_imu_(declare_parameter<bool>("enable_imu", false)),
  enable_debug_mode_(declare_parameter<bool>("enable_debug_mode", false)),
  debug_dump_path_(declare_parameter<std::string>("debug_dump_path", "/tmp/elbrus")),
  left_camera_frame_(declare_parameter<std::string>("left_camera_frame", "left_cam")),
  right_camera_frame_(declare_parameter<std::string>("right_camera_frame", "right_cam")),
  imu_frame_(declare_parameter<std::string>("imu_frame", "imu")),
  // This is the gravity vector in ROS coordinate frame.
  // Default value assumes that camera is horizontal to the floor.
  gravitational_force_(declare_parameter<std::vector<double>>(
      "gravitational_force",
      {0.0, 0, -9.8})),
  fixed_frame_(declare_parameter<std::string>("fixed_frame", "odom")),
  current_smooth_frame_(declare_parameter<std::string>(
      "current_smooth_frame",
      "base_link_smooth")),
  current_rectified_frame_(declare_parameter<std::string>(
      "current_rectified_frame",
      "base_link_rectified")),
  msg_filter_queue_size_(declare_parameter<int>("msg_filter_queue_size", 100)),
  msg_filter_max_interval_duration_(declare_parameter<int32_t>(
      "msg_filter_max_interval_duration",
      1)),
  // Publishers
  visual_odometry_msg_pub_(
    create_publisher<isaac_ros_visual_odometry_interfaces::msg::VisualOdometryTransformStamped>(
      "visual_odometry/tf_stamped", rclcpp::QoS(100))),
  // Subscribers
  left_image_sub_(message_filters::Subscriber<sensor_msgs::msg::Image>(
      this, "stereo_camera/left/image")),
  left_camera_info_sub_(message_filters::Subscriber<sensor_msgs::msg::CameraInfo>(
      this, "stereo_camera/left/camera_info")),
  right_image_sub_(message_filters::Subscriber<sensor_msgs::msg::Image>(
      this, "stereo_camera/right/image")),
  right_camera_info_sub_(message_filters::Subscriber<sensor_msgs::msg::CameraInfo>(
      this, "stereo_camera/right/camera_info")),
  imu_sub_(create_subscription<sensor_msgs::msg::Imu>(
      "visual_odometry/imu", rclcpp::QoS(100),
      std::bind(&VisualOdometryNode::ReadImuData, this, std::placeholders::_1))),
  // Initialize the impl
  impl_(std::make_unique<VisualOdometryImpl>(*this))
{
  impl_->sync.reset(
    new VisualOdometryImpl::Synchronizer(
      VisualOdometryImpl::ExactTime(msg_filter_queue_size_),
      left_image_sub_,
      left_camera_info_sub_,
      right_image_sub_,
      right_camera_info_sub_
  ));
  impl_->sync->registerCallback(
    std::bind(
      &VisualOdometryNode::TrackCameraPose, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
}

VisualOdometryNode::VisualOdometryImpl::VisualOdometryImpl(VisualOdometryNode & node)
: logger_name(std::string(node.get_logger().get_name())),
  node_clock(node.get_clock()),
  // Set elbrus_pose_base_link for converting ROS coordinate to Elbrus coordinate
  // ROS   ->  Elbrus
  // x     ->  -z
  // y     ->   -x
  // z     ->  y
  elbrus_pose_base_link(tf2::Matrix3x3(
      0, -1, 0,
      0, 0, 1,
      -1, 0, 0
    )),
  base_link_pose_elbrus(elbrus_pose_base_link.inverse()),
  tf_buffer(std::make_unique<tf2_ros::Buffer>(node_clock)),
  tf_listener(std::make_shared<tf2_ros::TransformListener>(*tf_buffer)),
  tf_publisher(std::make_unique<tf2_ros::TransformBroadcaster>(&node)),
  tf_static_publisher(std::make_unique<tf2_ros::StaticTransformBroadcaster>(&node))
{}

void VisualOdometryNode::ReadImuData(const sensor_msgs::msg::Imu::ConstSharedPtr msg_imu)
{
  // If elbrus handle is not initialized, then return.
  if (impl_->elbrus_handle != nullptr && impl_->is_initialized && enable_imu_) {
    const auto measurement = impl_->ToElbrusImuMeasurement(msg_imu);

    rclcpp::Time timestamp(msg_imu->header.stamp.sec, msg_imu->header.stamp.nanosec);
    const int64_t acqtime = static_cast<int64_t>(timestamp.nanoseconds());

    if (!ELBRUS_RegisterImuMeasurement(impl_->elbrus_handle, acqtime, &measurement)) {
      RCLCPP_WARN(this->get_logger(), "ELBRUS has failed to register an IMU measurement");
    }
  } else {return;}
}

void VisualOdometryNode::TrackCameraPose(
  const sensor_msgs::msg::Image::ConstSharedPtr & msg_left_img,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg_left_ci,
  const sensor_msgs::msg::Image::ConstSharedPtr & msg_right_img,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg_right_ci)
{
  // Initialize on first call
  if (!(impl_->is_initialized)) {
    impl_->Initialize(*this, msg_left_ci, msg_right_ci);
  }
  // Convert frame to mono image
  const cv::Mat mono_left_img = cv_bridge::toCvShare(msg_left_img, "mono8")->image;
  const cv::Mat mono_right_img = cv_bridge::toCvShare(msg_right_img, "mono8")->image;

  // Track a new pair of frames
  if (impl_->elbrus_handle != nullptr && impl_->is_initialized) {
    std::array<ELBRUS_Image, kNumCameras> elbrus_images = {};
    rclcpp::Time timestamp(msg_left_img->header.stamp.sec, msg_left_img->header.stamp.nanosec);
    // Elbrus expects image timestamps in micro seconds
    const int64_t acqtime = static_cast<int64_t>(timestamp.nanoseconds() / 1000);

    elbrus_images[LEFT_CAMERA_IDX] = impl_->ToElbrusImage(LEFT_CAMERA_IDX, mono_left_img, acqtime);
    elbrus_images[RIGHT_CAMERA_IDX] =
      impl_->ToElbrusImage(RIGHT_CAMERA_IDX, mono_right_img, acqtime);

    ELBRUS_PoseEstimate pose_estimate;

    const ELBRUS_Status status =
      ELBRUS_Track(impl_->elbrus_handle, elbrus_images.data(), nullptr, &pose_estimate);

    if (status == ELBRUS_TRACKING_LOST) {
      RCLCPP_WARN(this->get_logger(), "Tracker is lost");
      return;
    }
    if (status != ELBRUS_SUCCESS) {
      RCLCPP_WARN(this->get_logger(), "Unknown Tracker Error %d", status);
      return;
    }

    // Smooth pose in Elbrus coordinate system. Smooth pose is calculated from pure VO.
    // It will publish poses with respect to start position without any sudden jump.
    // Note: It can drift over time.
    tf2::Transform start_elbrus_pose_smooth_ebrus = impl_->FromElbrusPose(pose_estimate.pose);

    // Rectified pose in Elbrus coordinate system. It uses loop closure to get the robust tracking.
    // Note: It can result in sudden jumps of the final pose.
    // Publish Smooth pose if enable_rectified_pose_ = false
    tf2::Transform start_elbrus_pose_rectified_ebrus = start_elbrus_pose_smooth_ebrus;

    if (enable_rectified_pose_) {
      ELBRUS_PoseSlam pose_rectified;
      const ELBRUS_Status pose_rectified_status = ELBRUS_GetSlamPose(
        impl_->elbrus_handle,
        &pose_rectified);
      if (pose_rectified_status != ELBRUS_SUCCESS) {
        RCLCPP_WARN(this->get_logger(), "Get Rectified Pose Error %d", pose_rectified_status);
        return;
      }
      start_elbrus_pose_rectified_ebrus = impl_->FromElbrusPose(pose_rectified.pose);
    }

    // Change of basis vectors for smooth pose
    const tf2::Transform odom_pose_smooth_base_link{VisualOdometryImpl::ChangeBasis(
        impl_->base_link_pose_elbrus,
        start_elbrus_pose_smooth_ebrus)};

    impl_->PublishFrameTransform(
      odom_pose_smooth_base_link,
      fixed_frame_,
      current_smooth_frame_);

    // Change of basis vectors for rectified pose
    const tf2::Transform odom_pose_rectified_base_link{VisualOdometryImpl::ChangeBasis(
        impl_->base_link_pose_elbrus, start_elbrus_pose_rectified_ebrus)};

    impl_->PublishFrameTransform(
      odom_pose_rectified_base_link,
      fixed_frame_,
      current_rectified_frame_);

    isaac_ros_visual_odometry_interfaces::msg::VisualOdometryTransformStamped visual_odometry_msg;
    visual_odometry_msg.header = msg_left_img->header;
    visual_odometry_msg.child_frame_id = msg_right_img->header.frame_id;
    visual_odometry_msg.smooth_transform = tf2::toMsg(odom_pose_smooth_base_link);
    visual_odometry_msg.rectified_transform = tf2::toMsg(odom_pose_rectified_base_link);
    visual_odometry_msg.vo_state = pose_estimate.vo_state;
    visual_odometry_msg.integrator_state = pose_estimate.integrator_state;
    visual_odometry_msg_pub_->publish(visual_odometry_msg);
  }
}

void VisualOdometryNode::VisualOdometryImpl::Initialize(
  VisualOdometryNode & node,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg_left_ci,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg_right_ci)
{
  assert(elbrus_handle == nullptr && "Already initialized.");

  if (elbrus_handle == nullptr) {
    int32_t elbrus_major, elbrus_minor;
    ELBRUS_GetVersion(&elbrus_major, &elbrus_minor);
    RCLCPP_INFO(
      rclcpp::get_logger(logger_name), "Elbrus version: %d.%d", elbrus_major,
      elbrus_minor);

    tf2::Transform left_camera_pose_imu;
    if (!node.enable_imu_) {
      left_camera_pose_imu.setIdentity();
    } else {
      RCLCPP_INFO(rclcpp::get_logger(logger_name), "left_camera_pose_imu reading from TF Tree");
      // Convert the left_camera_pose_imu from ROS to Elbrus frame
      left_camera_pose_imu = elbrus_pose_base_link * GetFrameTransform(
        node.left_camera_frame_,
        node.imu_frame_);
    }

    // Elbrus configuration needs IMU pose Left camera
    const ELBRUS_Configuration configuration = getConfiguration(
      node, ToElbrusPose(left_camera_pose_imu.inverse()));

    ELBRUS_CameraRig rig;
    rig.cameras = elbrus_cameras.data();
    rig.num_cameras = kNumCameras;

    // Get camera intrinsics to set camera coordinate frames
    const double * right_p = msg_right_ci->p.data();

    tf2::Transform elbrus_pose_left_camera;
    // Set left camera rotation to identity and translation vector to zero
    // If camera info is used to set the left_pose_right, it still supports it.
    // See camera matrix info for more details : https://docs.ros2.org/foxy/api/sensor_msgs/msg/CameraInfo.html
    elbrus_pose_left_camera.setIdentity();

    tf2::Transform left_camera_pose_right_camera;
    // Set right camera rotation to identity
    left_camera_pose_right_camera.setIdentity();
    // Set right camera translation relative to left camera from camera info
    // Tx = -fx' * B, where B is the baseline between the cameras.
    // B = -Tx/fx'
    Eigen::Vector3d right_camera_translation(-right_p[3] / right_p[0], right_p[7], right_p[11]);
    left_camera_pose_right_camera.setOrigin(
      tf2::Vector3(
        right_camera_translation[0],
        right_camera_translation[1],
        right_camera_translation[2]));

    // Set the extrinsics to tf tree from camerainfo.
    // If not present in CameraInfo, get it from the tf tree which user has defined.
    if (!right_camera_translation.isZero()) {
      RCLCPP_INFO(
        rclcpp::get_logger(logger_name),
        "left_cam_pose_right_cam reading from CameraInfo");
      PublishFrameStaticTransform(
        left_camera_pose_right_camera,
        node.left_camera_frame_,
        node.right_camera_frame_);
    } else {
      RCLCPP_INFO(rclcpp::get_logger(logger_name), "left_cam_pose_right_cam reading from TF Tree");
      left_camera_pose_right_camera =
        ChangeBasis(
        elbrus_pose_base_link,
        GetFrameTransform(node.left_camera_frame_, node.right_camera_frame_));
    }
    SetElbrusCameraPose(elbrus_cameras[LEFT_CAMERA_IDX], ToElbrusPose(elbrus_pose_left_camera));
    SetElbrusCameraPose(
      elbrus_cameras[RIGHT_CAMERA_IDX], ToElbrusPose(left_camera_pose_right_camera));

    elbrus_cameras[LEFT_CAMERA_IDX].parameters = left_intrinsics.data();
    elbrus_cameras[RIGHT_CAMERA_IDX].parameters = right_intrinsics.data();
    ReadCameraIntrinsics(msg_left_ci, elbrus_cameras[LEFT_CAMERA_IDX], left_intrinsics);
    ReadCameraIntrinsics(msg_right_ci, elbrus_cameras[RIGHT_CAMERA_IDX], right_intrinsics);

    ELBRUS_TrackerHandle tracker;
    const ELBRUS_Status status = ELBRUS_CreateTracker(&tracker, &rig, &configuration);
    if (status != ELBRUS_SUCCESS) {
      RCLCPP_ERROR(
        node.get_logger(),
        "Failed to initialize ELBRUS tracker: %d", status);
      return;
    }
    elbrus_handle = tracker;
    is_initialized = true;
  }
}

ELBRUS_Configuration VisualOdometryNode::VisualOdometryImpl::getConfiguration(
  const VisualOdometryNode & node,
  const ELBRUS_Pose & imu_pose_camera)
{
  ELBRUS_Configuration configuration;
  ELBRUS_InitDefaultConfiguration(&configuration);
  // Converting gravity force from ROS frame to Elbrus frame
  const auto & gravitational_force = node.gravitational_force_;
  const tf2::Vector3 ros_g(gravitational_force[0], gravitational_force[1], gravitational_force[2]);
  const tf2::Vector3 elbrus_g = elbrus_pose_base_link * ros_g;
  configuration.g[0] = elbrus_g[0];
  configuration.g[1] = elbrus_g[1];
  configuration.g[2] = elbrus_g[2];
  configuration.use_motion_model = 1;
  configuration.use_bundle_adjustment = 1;
  configuration.imu_from_left = imu_pose_camera;
  configuration.use_denoising = node.denoise_input_images_ ? 1 : 0;
  configuration.horizontal_stereo_camera = node.rectified_images_ ? 1 : 0;
  // If IMU is present, set it to 1
  configuration.enable_imu_integrator = node.enable_imu_ ? 1 : 0;
  if (node.enable_debug_mode_) {configuration.debug_dump_directory = node.debug_dump_path_.c_str();}
  configuration.enable_localization_n_mapping = node.enable_rectified_pose_ ? 1 : 0;
  return configuration;
}

// Helper function that converts transform into ELBRUS_Pose
ELBRUS_Pose VisualOdometryNode::VisualOdometryImpl::ToElbrusPose(const tf2::Transform & tf_mat)
{
  ELBRUS_Pose elbrusPose;
  // tf2::Matrix3x3 is row major, but elbrus is column major
  const tf2::Matrix3x3 rotation(tf_mat.getRotation());
  const int32_t kRotationMatCol = 3;
  const int32_t kRotationMatRow = 3;
  int elbrus_idx = 0;
  for (int col_idx = 0; col_idx < kRotationMatCol; ++col_idx) {
    const tf2::Vector3 & rot_col = rotation.getColumn(col_idx);
    for (int row_idx = 0; row_idx < kRotationMatRow; ++row_idx) {
      elbrusPose.r[elbrus_idx] = rot_col[row_idx];
      elbrus_idx++;
    }
  }

  const tf2::Vector3 & translation = tf_mat.getOrigin();
  elbrusPose.t[0] = translation.x();
  elbrusPose.t[1] = translation.y();
  elbrusPose.t[2] = translation.z();
  return elbrusPose;
}

// Helper function to convert elbrus pose estimate to tf2::Transform
tf2::Transform VisualOdometryNode::VisualOdometryImpl::FromElbrusPose(
  const ELBRUS_Pose & elbrus_pose)
{
  const auto & r = elbrus_pose.r;
  // tf2::Matrix3x3 is row major and Elbrus rotation mat is column major.
  const tf2::Matrix3x3 rotation(r[0], r[3], r[6], r[1], r[4], r[7], r[2], r[5], r[8]);
  const tf2::Vector3 translation(elbrus_pose.t[0], elbrus_pose.t[1], elbrus_pose.t[2]);

  return tf2::Transform(rotation, translation);
}

// Helper function to set internal parameters for the elbrus camera structure
void VisualOdometryNode::VisualOdometryImpl::SetElbrusCameraPose(
  ELBRUS_Camera & cam,
  const ELBRUS_Pose & elbrus_pose) {cam.pose = elbrus_pose;}

void VisualOdometryNode::VisualOdometryImpl::ReadCameraIntrinsics(
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg_ci,
  ELBRUS_Camera & elbrus_camera,
  std::array<float, kMaxNumCameraParameters> & intrinsics)
{
  // Get pinhole
  elbrus_camera.width = msg_ci->width;
  elbrus_camera.height = msg_ci->height;
  elbrus_camera.distortion_model = msg_ci->distortion_model.c_str();
  elbrus_camera.num_parameters = 4;

  // the expected order is cx, cy, fx, fy
  const double * k = msg_ci->k.data();
  intrinsics[0] = k[2];
  intrinsics[1] = k[5];
  intrinsics[2] = k[0];
  intrinsics[3] = k[4];

  // Get lens distortion
  std::string distortion_model{msg_ci->distortion_model};
  const auto coefficients = msg_ci->d.data();
  if (distortion_model.empty() && Eigen::Vector3d(coefficients).isZero()) {
    // Continue with pinhole parameters
    RCLCPP_WARN(
      rclcpp::get_logger(
        logger_name), "Distortion Model is empty! Continuing with pinhole model.");
    distortion_model = "pinhole";
  }
  const auto distortion = g_str_to_distortion_model.find(distortion_model);
  if (distortion == std::end(g_str_to_distortion_model)) {
    RCLCPP_ERROR(
      rclcpp::get_logger(logger_name),
      "Unsupported distortion model[%s].", distortion_model.c_str());
    throw std::runtime_error("Unsupported distortion model.");
  } else {
    switch (distortion->second) {
      case (DistortionModel::BROWN): {
          if (Eigen::Vector3d(coefficients).isZero()) {
            // Continue with pinhole parameters
            elbrus_camera.distortion_model = kPinhole;
            elbrus_camera.num_parameters = 4;
          } else {
            elbrus_camera.distortion_model = kBrown5k;

            const float k1 = coefficients[0];
            const float k2 = coefficients[1];
            const float t1 = coefficients[2];
            const float t2 = coefficients[3];
            const float k3 = coefficients[4];

            // 4-6: radial distortion coeffs (k1, k2, k3)
            intrinsics[4] = k1;
            intrinsics[5] = k2;
            intrinsics[6] = k3;

            // 7-8: tangential distortion coeffs (p1, p2)
            intrinsics[7] = t1;
            intrinsics[8] = t2;

            elbrus_camera.num_parameters = 9;
          }
          break;
        }
      case (DistortionModel::FISHEYE): {
          elbrus_camera.distortion_model = kFisheye4;
          // 4-7: fisheye distortion coeffs (k1, k2, k3, k4)
          intrinsics[4] = coefficients[0];
          intrinsics[5] = coefficients[1];
          intrinsics[6] = coefficients[2];
          intrinsics[7] = coefficients[3];

          elbrus_camera.num_parameters = 8;
          break;
        }
      case (DistortionModel::PINHOLE): {
          elbrus_camera.distortion_model = kPinhole;
          elbrus_camera.num_parameters = 4;
          break;
        }
      case (DistortionModel::RATIONAL_POLYNOMIAL): {
          RCLCPP_WARN(
            rclcpp::get_logger(logger_name),
            "Distortion Model = rational_polynomial not supported. Continuing with Pinhole model.");
          elbrus_camera.distortion_model = kPinhole;
          elbrus_camera.num_parameters = 4;
          break;
        }
      default: {
          RCLCPP_ERROR(rclcpp::get_logger(logger_name), "Unavailable input distortion model.");
          throw std::runtime_error("Unavailable input distortion model.");
        }
    }
  }
}

// Helper function to create ELBRUS_Image from cv::Mat
ELBRUS_Image VisualOdometryNode::VisualOdometryImpl::ToElbrusImage(
  int32_t camera_index,
  const cv::Mat & image,
  const int64_t & acqtime)
{
  ELBRUS_Image elbrus_image;
  elbrus_image.timestamp_us = acqtime;
  elbrus_image.pixels = image.ptr();
  elbrus_image.width = image.cols;
  elbrus_image.height = image.rows;
  elbrus_image.camera_index = camera_index;
  return elbrus_image;
}

// Helper function to pass IMU data to Elbrus
ELBRUS_ImuMeasurement VisualOdometryNode::VisualOdometryImpl::ToElbrusImuMeasurement(
  const sensor_msgs::msg::Imu::ConstSharedPtr & msg_imu)
{
  ELBRUS_ImuMeasurement measurement;
  // Set linear accelerations
  measurement.linear_accelerations[0] = msg_imu->linear_acceleration.x;
  measurement.linear_accelerations[1] = msg_imu->linear_acceleration.y;
  measurement.linear_accelerations[2] = msg_imu->linear_acceleration.z;
  // Set angular velocity
  measurement.angular_velocities[0] = msg_imu->angular_velocity.x;
  measurement.angular_velocities[1] = msg_imu->angular_velocity.y;
  measurement.angular_velocities[2] = msg_imu->angular_velocity.z;

  return measurement;
}

// Helper funtion to change basis from frame source to frame target
// target_pose_source = Transformation(Rotation only; translation is zero) matrix between target
// and source.
// source_pose_source = Transformation(Rotation and translation) inside source frame.
// It is any arbritary transformation in source frame.
tf2::Transform VisualOdometryNode::VisualOdometryImpl::ChangeBasis(
  const tf2::Transform & target_pose_source, const tf2::Transform & source_pose_source)
{
  return target_pose_source * source_pose_source * target_pose_source.inverse();
}

// Helper function to publish statically source frame pose wrt target frame to the tf tree
void VisualOdometryNode::VisualOdometryImpl::PublishFrameStaticTransform(
  const tf2::Transform & pose,
  const std::string & target,
  const std::string & source)
{
  geometry_msgs::msg::TransformStamped target_pose_source;
  target_pose_source.header.stamp = node_clock->now();
  target_pose_source.header.frame_id = target;
  target_pose_source.child_frame_id = source;
  target_pose_source.transform = tf2::toMsg(pose);
  RCLCPP_INFO(
    rclcpp::get_logger(logger_name),
    "Target = %s, Source = %s: [%f %f %f %f %f %f %f]",
    target.c_str(), source.c_str(),
    target_pose_source.transform.translation.x,
    target_pose_source.transform.translation.y,
    target_pose_source.transform.translation.z,
    target_pose_source.transform.rotation.x,
    target_pose_source.transform.rotation.y,
    target_pose_source.transform.rotation.z,
    target_pose_source.transform.rotation.w
  );
  tf_static_publisher->sendTransform(target_pose_source);
}

// Helper function to publish source frame pose wrt target frame to the tf tree
void VisualOdometryNode::VisualOdometryImpl::PublishFrameTransform(
  const tf2::Transform & pose,
  const std::string & target,
  const std::string & source)
{
  geometry_msgs::msg::TransformStamped target_pose_source;
  target_pose_source.header.stamp = node_clock->now();
  target_pose_source.header.frame_id = target;
  target_pose_source.child_frame_id = source;
  target_pose_source.transform = tf2::toMsg(pose);
  tf_publisher->sendTransform(target_pose_source);
}

// Helper function to get child frame pose wrt parent frame from the tf tree
tf2::Transform VisualOdometryNode::VisualOdometryImpl::GetFrameTransform(
  const std::string & target,
  const std::string & source)
{
  geometry_msgs::msg::TransformStamped transform_stamped;
  tf2::Transform pose;

  // Time out duration for TF tree lookup before throwing an exception.
  const int32_t kTimeOutSeconds = 1;
  const uint32_t kTimeOutNanoSeconds = 0;
  try {
    if (tf_buffer->canTransform(
        target, source, node_clock->now(), rclcpp::Duration(kTimeOutSeconds, kTimeOutNanoSeconds)))
    {
      RCLCPP_INFO(rclcpp::get_logger(logger_name), "Found the transform!");
    }
    transform_stamped = tf_buffer->lookupTransform(
      target,
      source, tf2::TimePointZero);
    const tf2::Quaternion rotation(
      transform_stamped.transform.rotation.x,
      transform_stamped.transform.rotation.y,
      transform_stamped.transform.rotation.z,
      transform_stamped.transform.rotation.w);
    const tf2::Vector3 translation(
      transform_stamped.transform.translation.x,
      transform_stamped.transform.translation.y,
      transform_stamped.transform.translation.z);
    pose =
      tf2::Transform(rotation, translation);
  } catch (tf2::TransformException & ex) {
    RCLCPP_INFO(
      rclcpp::get_logger(logger_name), "Could not transform %s to %s: %s",
      source.c_str(), target.c_str(), ex.what());
    throw std::runtime_error("Could not find the requested transform!");
  }
  return pose;
}

VisualOdometryNode::VisualOdometryImpl::~VisualOdometryImpl()
{
  if (elbrus_handle != nullptr) {ELBRUS_DestroyTracker(elbrus_handle);}
}

VisualOdometryNode::~VisualOdometryNode()
{
  impl_.reset();
}

}  // namespace visual_odometry
}  // namespace isaac_ros

// Register as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(isaac_ros::visual_odometry::VisualOdometryNode)
