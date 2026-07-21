/****************************************************************************
 * Copyright (c) 2026 Xiangyuan Xie <dragonboat_xxy@163.com>.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <am_ee_pose_commander/offboard_conversion.hpp>
#include <am_ee_pose_commander/arm_sync.hpp>
#include <am_ee_pose_commander/policy_contract.hpp>
#include <am_ee_pose_commander/vehicle_state.hpp>
#include <am_ee_pose_commander/x500_arm_kinematics.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <policy_inference/onnx_policy.hpp>
#include <policy_inference/recurrent_state.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <iterator>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace am_ee_pose_commander
{
namespace
{

constexpr uint64_t kControlPeriodUs = 20000;
constexpr uint64_t kControlPeriodToleranceUs = 1000;
constexpr double kVehicleStateTimeoutS = 0.2;
constexpr double kFollowerStateTimeoutS = 0.5;
constexpr double kSyncStatusTimeoutS = 0.5;
constexpr double kReadyDwellS = 0.2;
constexpr float kQuaternionNormEpsilon = 1.0e-6F;

double steadyNowS()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

template<typename T>
T declareOrGet(rclcpp::Node & node, const std::string & name, const T & default_value)
{
  if (!node.has_parameter(name)) {
    node.declare_parameter<T>(name, default_value);
  }
  return node.get_parameter(name).get_value<T>();
}

bool validAbsoluteTopic(const std::string & topic)
{
  return !topic.empty() && topic.front() == '/';
}

}  // namespace

class AmEePoseCommander final : public rclcpp::Node
{
public:
  AmEePoseCommander()
  : rclcpp::Node("am_ee_pose_commander")
  {
    target_frame_id_ = declareOrGet(*this, "target_frame_id", target_frame_id_);
    target_preview_topic_ = declareOrGet(*this, "target_preview_topic", target_preview_topic_);
    vehicle_odometry_topic_ = declareOrGet(
      *this, "vehicle_odometry_topic", vehicle_odometry_topic_);
    offboard_control_mode_topic_ = declareOrGet(
      *this, "offboard_control_mode_topic", offboard_control_mode_topic_);
    trajectory_setpoint_topic_ = declareOrGet(
      *this, "trajectory_setpoint_topic", trajectory_setpoint_topic_);
    arm_state_topic_ = declareOrGet(*this, "arm_state_topic", arm_state_topic_);
    gripper_state_topic_ = declareOrGet(*this, "gripper_state_topic", gripper_state_topic_);
    follower_sync_status_topic_ = declareOrGet(
      *this, "follower_sync_status_topic", follower_sync_status_topic_);
    arm_command_topic_ = declareOrGet(*this, "arm_command_topic", arm_command_topic_);
    gripper_command_topic_ = declareOrGet(
      *this, "gripper_command_topic", gripper_command_topic_);
    leader_sync_mode_topic_ = declareOrGet(
      *this, "leader_sync_mode_topic", leader_sync_mode_topic_);
    current_ee_pose_topic_ = declareOrGet(
      *this, "current_ee_pose_topic", current_ee_pose_topic_);
    target_timeout_s_ = declareOrGet(*this, "target_timeout_s", target_timeout_s_);

    validateParameters();
    const std::string upper_model_path = declareOrGet(
      *this, "upper_model_path", std::string{});
    if (upper_model_path.empty()) {
      throw std::invalid_argument("upper_model_path must be provided explicitly");
    }
    const std::filesystem::path model_path(upper_model_path);
    if (!model_path.is_absolute()) {
      throw std::invalid_argument(
              "upper_model_path must be absolute; '~' is not expanded by ROS launch arguments");
    }
    upper_policy_ = std::make_unique<policy_inference::OnnxPolicy>(model_path);
    validateUpperModelContract();
    upper_rnn_state_.configure(upper_policy_->inputShape("h_in"));
    resetHandshake(steadyNowS());

    const auto input_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    target_preview_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      target_preview_topic_, input_qos,
      std::bind(&AmEePoseCommander::targetPreviewCallback, this, std::placeholders::_1));
    vehicle_odometry_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
      vehicle_odometry_topic_, input_qos,
      std::bind(&AmEePoseCommander::vehicleOdometryCallback, this, std::placeholders::_1));
    follower_arm_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      arm_state_topic_, input_qos,
      std::bind(&AmEePoseCommander::followerArmStateCallback, this, std::placeholders::_1));
    follower_gripper_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      gripper_state_topic_, input_qos,
      std::bind(&AmEePoseCommander::followerGripperStateCallback, this, std::placeholders::_1));
    follower_sync_status_sub_ = create_subscription<std_msgs::msg::String>(
      follower_sync_status_topic_, input_qos,
      std::bind(&AmEePoseCommander::followerSyncStatusCallback, this, std::placeholders::_1));

    offboard_control_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      offboard_control_mode_topic_, output_qos);
    trajectory_setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      trajectory_setpoint_topic_, output_qos);
    arm_command_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      arm_command_topic_, output_qos);
    gripper_command_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      gripper_command_topic_, output_qos);
    leader_sync_mode_pub_ = create_publisher<std_msgs::msg::String>(
      leader_sync_mode_topic_, output_qos);
    current_ee_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      current_ee_pose_topic_, output_qos);

    RCLCPP_INFO(
      get_logger(),
      "AM EE Pose commander loaded: upper policy=50 Hz, PX4 output='%s', target='%s'.",
      trajectory_setpoint_topic_.c_str(), target_preview_topic_.c_str());
  }

private:
  void validateParameters() const
  {
    if (target_frame_id_.empty()) {
      throw std::invalid_argument("target_frame_id must not be empty");
    }
    if (target_timeout_s_ <= 0.0) {
      throw std::invalid_argument("target_timeout_s must be positive");
    }
    for (const auto & topic : {
        target_preview_topic_, vehicle_odometry_topic_, offboard_control_mode_topic_,
        trajectory_setpoint_topic_, arm_state_topic_, gripper_state_topic_,
        follower_sync_status_topic_, arm_command_topic_, gripper_command_topic_,
        leader_sync_mode_topic_, current_ee_pose_topic_})
    {
      if (!validAbsoluteTopic(topic)) {
        throw std::invalid_argument(
                "AM EE Pose commander topics must use absolute ROS names: " + topic);
      }
    }
  }

  void validateUpperModelContract() const
  {
    if (!upper_policy_->hasInput("obs")) {
      throw std::invalid_argument("AM EE Pose upper ONNX model must expose input 'obs'");
    }
    const auto observation_shape = upper_policy_->inputShape("obs");
    if (observation_shape.size() != 2 || observation_shape[0] != 1 ||
      observation_shape[1] != static_cast<int64_t>(kUpperObservationDim))
    {
      throw std::invalid_argument("AM EE Pose upper ONNX input 'obs' must have shape [1, 63]");
    }
    if (!upper_policy_->hasInput("h_in")) {
      throw std::invalid_argument("AM EE Pose upper ONNX model must expose GRU input 'h_in'");
    }
    const auto hidden_shape = upper_policy_->inputShape("h_in");
    if (hidden_shape.size() != 3 || hidden_shape[0] != 1 || hidden_shape[1] != 1 ||
      hidden_shape[2] != 64)
    {
      throw std::invalid_argument("AM EE Pose upper ONNX input 'h_in' must have shape [1, 1, 64]");
    }
    if (!upper_policy_->hasOutput("actions")) {
      throw std::invalid_argument("AM EE Pose upper ONNX model must expose output 'actions'");
    }
    const auto action_shape = upper_policy_->outputShape("actions");
    if (action_shape.size() != 2 || action_shape[0] != 1 ||
      action_shape[1] != static_cast<int64_t>(kUpperActionDim))
    {
      throw std::invalid_argument("AM EE Pose upper ONNX output 'actions' must have shape [1, 8]");
    }
    if (!upper_policy_->hasOutput("h_out")) {
      throw std::invalid_argument("AM EE Pose upper ONNX model must expose GRU output 'h_out'");
    }
    const auto recurrent_shape = upper_policy_->outputShape("h_out");
    if (recurrent_shape != hidden_shape) {
      throw std::invalid_argument(
              "AM EE Pose upper ONNX output 'h_out' must match input 'h_in' shape [1, 1, 64]");
    }
  }

  void targetPreviewCallback(const geometry_msgs::msg::PoseArray::SharedPtr message)
  {
    if (message->header.frame_id != target_frame_id_) {
      RCLCPP_WARN(
        get_logger(), "Ignoring AM EE Pose preview in frame '%s'; expected '%s'.",
        message->header.frame_id.c_str(), target_frame_id_.c_str());
      return;
    }
    if (message->poses.size() != kPreviewPoseCount) {
      RCLCPP_WARN(
        get_logger(), "Ignoring AM EE Pose preview with %zu poses; expected exactly 5.",
        message->poses.size());
      return;
    }

    TargetPreview next_preview;
    for (std::size_t index = 0; index < kPreviewPoseCount; ++index) {
      const auto & source = message->poses[index];
      Pose target;
      target.position_w = Eigen::Vector3f(
        static_cast<float>(source.position.x),
        static_cast<float>(source.position.y),
        static_cast<float>(source.position.z));
      Eigen::Quaternionf orientation(
        static_cast<float>(source.orientation.w),
        static_cast<float>(source.orientation.x),
        static_cast<float>(source.orientation.y),
        static_cast<float>(source.orientation.z));
      if (!target.position_w.allFinite() || !orientation.coeffs().allFinite() ||
        orientation.norm() < kQuaternionNormEpsilon)
      {
        RCLCPP_WARN(get_logger(), "Ignoring AM EE Pose preview with invalid pose data.");
        return;
      }
      orientation.normalize();
      target.rotation_w = orientation.toRotationMatrix();
      next_preview[index] = target;
    }

    external_target_preview_ = next_preview;
    last_target_preview_s_ = steadyNowS();
  }

  void vehicleOdometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr message)
  {
    std::string error;
    const auto state = vehicleStateFromPx4Odometry(*message, &error);
    if (!state.has_value()) {
      if (!reported_invalid_vehicle_state_) {
        RCLCPP_WARN(get_logger(), "Ignoring invalid PX4 vehicle odometry: %s.", error.c_str());
        reported_invalid_vehicle_state_ = true;
      }
      return;
    }

    reported_invalid_vehicle_state_ = false;
    vehicle_state_ = state;
    const double now_s = steadyNowS();
    last_vehicle_state_s_ = now_s;
    publishCurrentEndEffectorPose();

    uint64_t sample_time_us = state->timestamp_sample_us;
    if (sample_time_us == 0) {
      sample_time_us = state->timestamp_us;
    }
    if (sample_time_us == 0) {
      sample_time_us = static_cast<uint64_t>(now_s * 1.0e6);
    }

    if (last_control_sample_us_ != 0 && sample_time_us >= last_control_sample_us_ &&
      sample_time_us - last_control_sample_us_ < kControlPeriodUs - kControlPeriodToleranceUs)
    {
      return;
    }
    if (last_control_sample_us_ != 0 && sample_time_us < last_control_sample_us_) {
      resetPolicyState();
    }
    last_control_sample_us_ = sample_time_us;
    controlTick(now_s, state->timestamp_us == 0 ? sample_time_us : state->timestamp_us);
  }

  void followerArmStateCallback(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    ArmVector position;
    if (!readArmPosition(*message, position)) {
      RCLCPP_WARN(get_logger(), "Ignoring invalid ACETele follower arm state.");
      return;
    }
    follower_arm_position_ = position;
    last_follower_arm_state_s_ = steadyNowS();
  }

  void followerGripperStateCallback(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    float position = 0.0F;
    if (!readGripperPosition(*message, position)) {
      RCLCPP_WARN(get_logger(), "Ignoring invalid ACETele follower gripper state.");
      return;
    }
    follower_gripper_public_position_ = position;
    last_follower_gripper_state_s_ = steadyNowS();
  }

  void followerSyncStatusCallback(const std_msgs::msg::String::SharedPtr message)
  {
    static const std::vector<std::string> valid_statuses{
      "idle", "aligning", "ready", "tracking", "lost", "fault"};
    if (std::find(valid_statuses.begin(), valid_statuses.end(), message->data) ==
      valid_statuses.end())
    {
      RCLCPP_WARN(
        get_logger(), "Ignoring invalid ACETele follower sync status '%s'.",
        message->data.c_str());
      return;
    }
    follower_sync_status_ = message->data;
    last_follower_sync_status_s_ = steadyNowS();
  }

  void controlTick(double now_s, uint64_t timestamp_us)
  {
    initializeStartupHoldTarget(now_s);
    if (!prerequisitesReady(now_s)) {
      if (input_gate_open_) {
        resetHandshake(now_s);
        resetPolicyState();
        input_gate_open_ = false;
      }
      publishSyncMode("sync_request");
      publishVelocityCommand(WorldVelocityCommand{}, timestamp_us);
      if (!warned_missing_prerequisites_) {
        RCLCPP_WARN(
          get_logger(),
          "AM EE Pose commander is waiting for PX4 odometry, arm, gripper, and follower sync state before holding its startup EE pose.");
        warned_missing_prerequisites_ = true;
      }
      return;
    }

    if (!input_gate_open_) {
      resetHandshake(now_s);
      input_gate_open_ = true;
      warned_missing_prerequisites_ = false;
    }
    sync_handshake_->notifyFollowerState(*last_follower_arm_state_s_);
    sync_handshake_->notifyFollowerSyncStatus(
      follower_sync_status_, *last_follower_sync_status_s_);
    const ArmSyncUpdate sync = sync_handshake_->update(now_s);
    publishSyncMode(sync.leader_mode);
    if (!sync.commands_allowed) {
      publishVelocityCommand(WorldVelocityCommand{}, timestamp_us);
      return;
    }

    if (sync.tracking_started || !arm_target_initialized_) {
      resetPolicyState();
      arm_position_target_ = follower_arm_position_;
      arm_target_initialized_ = true;
    }

    try {
      runUpperPolicy(now_s, timestamp_us);
      inference_fault_reported_ = false;
    } catch (const std::exception & error) {
      publishVelocityCommand(WorldVelocityCommand{}, timestamp_us);
      publishSyncMode("sync_request");
      resetHandshake(now_s);
      resetPolicyState();
      input_gate_open_ = false;
      if (!inference_fault_reported_) {
        RCLCPP_ERROR(get_logger(), "AM EE Pose inference stopped: %s", error.what());
        inference_fault_reported_ = true;
      }
    }
  }

  bool prerequisitesReady(double now_s) const
  {
    const bool follower_faulted =
      follower_sync_status_ == "lost" || follower_sync_status_ == "fault";
    return vehicle_state_.has_value() && !follower_faulted &&
           startup_hold_preview_.has_value() &&
           isRecent(last_vehicle_state_s_, now_s, kVehicleStateTimeoutS) &&
           isRecent(last_follower_arm_state_s_, now_s, kFollowerStateTimeoutS) &&
           isRecent(last_follower_gripper_state_s_, now_s, kFollowerStateTimeoutS) &&
           isRecent(last_follower_sync_status_s_, now_s, kSyncStatusTimeoutS);
  }

  static bool isRecent(
    const std::optional<double> & sample_time_s,
    double now_s,
    double timeout_s)
  {
    return sample_time_s.has_value() && now_s >= *sample_time_s &&
           now_s - *sample_time_s <= timeout_s;
  }

  void resetHandshake(double now_s)
  {
    sync_handshake_ = std::make_unique<ArmSyncHandshake>(
      kFollowerStateTimeoutS, kSyncStatusTimeoutS, kReadyDwellS);
    if (isRecent(last_follower_arm_state_s_, now_s, kFollowerStateTimeoutS)) {
      sync_handshake_->notifyFollowerState(*last_follower_arm_state_s_);
    }
    if (isRecent(last_follower_sync_status_s_, now_s, kSyncStatusTimeoutS)) {
      sync_handshake_->notifyFollowerSyncStatus(
        follower_sync_status_, *last_follower_sync_status_s_);
    }
  }

  void resetPolicyState()
  {
    upper_rnn_state_.reset();
    last_upper_action_.setZero();
    arm_target_initialized_ = false;
  }

  void initializeStartupHoldTarget(double now_s)
  {
    if (startup_hold_preview_.has_value() || !vehicle_state_.has_value() ||
      !isRecent(last_vehicle_state_s_, now_s, kVehicleStateTimeoutS) ||
      !isRecent(last_follower_arm_state_s_, now_s, kFollowerStateTimeoutS) ||
      !isRecent(last_follower_gripper_state_s_, now_s, kFollowerStateTimeoutS))
    {
      return;
    }

    const Pose startup_pose = currentEndEffectorPose();
    startup_hold_preview_ = makeConstantTargetPreview(startup_pose);
    Eigen::Quaternionf orientation(startup_pose.rotation_w);
    orientation.normalize();
    RCLCPP_INFO(
      get_logger(),
      "Locked startup EE hold target: position_enu_m=(%.3f, %.3f, %.3f), "
      "orientation_xyzw=(%.3f, %.3f, %.3f, %.3f).",
      startup_pose.position_w.x(), startup_pose.position_w.y(), startup_pose.position_w.z(),
      orientation.x(), orientation.y(), orientation.z(), orientation.w());
  }

  const TargetPreview & activeTargetPreview(double now_s)
  {
    if (external_target_preview_.has_value() &&
      isRecent(last_target_preview_s_, now_s, target_timeout_s_))
    {
      if (!external_target_active_) {
        RCLCPP_INFO(
          get_logger(), "Using external EE trajectory preview from '%s'.",
          target_preview_topic_.c_str());
      }
      external_target_active_ = true;
      return *external_target_preview_;
    }

    if (external_target_active_) {
      RCLCPP_WARN(
        get_logger(),
        "External EE trajectory preview timed out; returning to the locked startup EE pose.");
      external_target_active_ = false;
    }
    if (!startup_hold_preview_.has_value()) {
      throw std::logic_error("startup EE hold target is unavailable");
    }
    return *startup_hold_preview_;
  }

  void runUpperPolicy(double now_s, uint64_t timestamp_us)
  {
    if (!vehicle_state_.has_value()) {
      throw std::logic_error("vehicle state is unavailable");
    }
    const Pose current_ee_pose = currentEndEffectorPose();
    policy_inference::TensorMap inputs;
    inputs["obs"] = PolicyContract::buildObservation(
      activeTargetPreview(now_s), current_ee_pose, follower_arm_position_,
      vehicle_state_->linear_velocity_flu_m_s,
      vehicle_state_->angular_velocity_flu_rad_s,
      last_upper_action_);
    upper_rnn_state_.appendInput(inputs);
    const policy_inference::TensorMap output = upper_policy_->infer(inputs);
    const auto action_it = output.find("actions");
    if (action_it == output.end()) {
      throw std::runtime_error("upper ONNX model must expose float output 'actions'");
    }

    const auto & model_action = action_it->second;
    const ProcessedUpperAction action = PolicyContract::processAction(
      model_action, arm_position_target_, joint_lower_limit_, joint_upper_limit_);
    upper_rnn_state_.updateFromOutput(output);

    const WorldVelocityCommand velocity_command = headingVelocityToWorld(
      action.heading_velocity_command, vehicle_state_->attitude_enu);
    publishVelocityCommand(velocity_command, timestamp_us);
    publishArmCommand(action);
    last_upper_action_ = action.raw_action;
    arm_position_target_ = action.arm_position_target;
  }

  void publishVelocityCommand(
    const WorldVelocityCommand & command,
    uint64_t timestamp_us)
  {
    offboard_control_mode_pub_->publish(makeVelocityOffboardControlMode(timestamp_us));
    trajectory_setpoint_pub_->publish(makeVelocityTrajectorySetpoint(command, timestamp_us));
  }

  void publishArmCommand(const ProcessedUpperAction & action)
  {
    const auto stamp = get_clock()->now();
    sensor_msgs::msg::JointState arm_message;
    arm_message.header.stamp = stamp;
    arm_message.name = arm_joint_names_;
    arm_message.position.assign(
      action.arm_position_target.data(), action.arm_position_target.data() + kArmJointCount);
    arm_message.velocity.assign(
      action.arm_velocity_target.data(), action.arm_velocity_target.data() + kArmJointCount);
    arm_message.effort.assign(kArmJointCount, 0.0);
    arm_command_pub_->publish(arm_message);

    sensor_msgs::msg::JointState gripper_message;
    gripper_message.header.stamp = stamp;
    gripper_message.name = {gripper_joint_name_};
    gripper_message.position = {static_cast<double>(follower_gripper_public_position_)};
    gripper_message.velocity = {0.0};
    gripper_message.effort = {0.0};
    gripper_command_pub_->publish(gripper_message);
  }

  void publishSyncMode(const std::string & mode)
  {
    std_msgs::msg::String message;
    message.data = mode;
    leader_sync_mode_pub_->publish(message);
  }

  Pose currentEndEffectorPose() const
  {
    const Eigen::Isometry3f body_to_ee = arm_kinematics_.link5PoseFlu(
      follower_arm_position_, follower_gripper_public_position_);
    Eigen::Isometry3f world_to_body = Eigen::Isometry3f::Identity();
    world_to_body.linear() = vehicle_state_->attitude_enu.normalized().toRotationMatrix();
    world_to_body.translation() = vehicle_state_->position_enu_m;
    const Eigen::Isometry3f world_to_ee = world_to_body * body_to_ee;

    Pose pose;
    pose.position_w = world_to_ee.translation();
    pose.rotation_w = world_to_ee.linear();
    return pose;
  }

  void publishCurrentEndEffectorPose()
  {
    if (!vehicle_state_.has_value() || !last_follower_arm_state_s_.has_value() ||
      !last_follower_gripper_state_s_.has_value())
    {
      return;
    }
    const Pose pose = currentEndEffectorPose();
    Eigen::Quaternionf orientation(pose.rotation_w);
    orientation.normalize();

    geometry_msgs::msg::PoseStamped message;
    message.header.stamp = get_clock()->now();
    message.header.frame_id = target_frame_id_;
    message.pose.position.x = pose.position_w.x();
    message.pose.position.y = pose.position_w.y();
    message.pose.position.z = pose.position_w.z();
    message.pose.orientation.w = orientation.w();
    message.pose.orientation.x = orientation.x();
    message.pose.orientation.y = orientation.y();
    message.pose.orientation.z = orientation.z();
    current_ee_pose_pub_->publish(message);
  }

  bool readArmPosition(
    const sensor_msgs::msg::JointState & message,
    ArmVector & position) const
  {
    if (message.name.empty()) {
      if (message.position.size() != kArmJointCount) {
        return false;
      }
      for (std::size_t index = 0; index < kArmJointCount; ++index) {
        position[static_cast<int>(index)] = static_cast<float>(message.position[index]);
      }
    } else {
      if (message.name.size() != message.position.size()) {
        return false;
      }
      for (std::size_t joint = 0; joint < kArmJointCount; ++joint) {
        const auto name_it = std::find(
          message.name.begin(), message.name.end(), arm_joint_names_[joint]);
        if (name_it == message.name.end()) {
          return false;
        }
        const auto index = static_cast<std::size_t>(
          std::distance(message.name.begin(), name_it));
        position[static_cast<int>(joint)] = static_cast<float>(message.position[index]);
      }
    }
    return position.allFinite();
  }

  bool readGripperPosition(
    const sensor_msgs::msg::JointState & message,
    float & position) const
  {
    if (message.position.empty()) {
      return false;
    }
    std::size_t index = 0;
    if (!message.name.empty()) {
      if (message.name.size() != message.position.size()) {
        return false;
      }
      const auto name_it = std::find(
        message.name.begin(), message.name.end(), gripper_joint_name_);
      if (name_it == message.name.end()) {
        return false;
      }
      index = static_cast<std::size_t>(std::distance(message.name.begin(), name_it));
    } else if (message.position.size() != 1) {
      return false;
    }
    position = static_cast<float>(message.position[index]);
    return std::isfinite(position) && position >= 0.0F && position <= 1.0F;
  }

  std::unique_ptr<policy_inference::OnnxPolicy> upper_policy_;
  policy_inference::RecurrentState upper_rnn_state_;
  X500ArmKinematics arm_kinematics_;
  std::unique_ptr<ArmSyncHandshake> sync_handshake_;

  std::optional<VehicleState> vehicle_state_;
  std::optional<TargetPreview> startup_hold_preview_;
  std::optional<TargetPreview> external_target_preview_;
  ArmVector follower_arm_position_{ArmVector::Zero()};
  float follower_gripper_public_position_{0.0F};
  ArmVector arm_position_target_{ArmVector::Zero()};
  UpperActionVector last_upper_action_{UpperActionVector::Zero()};
  const ArmVector joint_lower_limit_{
    (ArmVector() << -2.6485F, 0.0F, -2.6485F, -3.1415F).finished()};
  const ArmVector joint_upper_limit_{
    (ArmVector() << 2.6485F, 3.1415F, 2.6485F, 3.1415F).finished()};

  std::optional<double> last_target_preview_s_;
  std::optional<double> last_vehicle_state_s_;
  std::optional<double> last_follower_arm_state_s_;
  std::optional<double> last_follower_gripper_state_s_;
  std::optional<double> last_follower_sync_status_s_;
  std::string follower_sync_status_{"idle"};

  std::string target_frame_id_{"world"};
  std::string target_preview_topic_{"/am_ee_pose/trajectory_preview"};
  std::string vehicle_odometry_topic_{"/fmu/out/vehicle_odometry"};
  std::string offboard_control_mode_topic_{"/fmu/in/offboard_control_mode"};
  std::string trajectory_setpoint_topic_{"/fmu/in/trajectory_setpoint"};
  std::string arm_state_topic_{"/ace_follower/arm/state"};
  std::string gripper_state_topic_{"/ace_follower/gripper/state"};
  std::string follower_sync_status_topic_{"/ace_follower/arm/sync_status"};
  std::string arm_command_topic_{"/ace_leader/arm/command"};
  std::string gripper_command_topic_{"/ace_leader/gripper/command"};
  std::string leader_sync_mode_topic_{"/ace_leader/arm/sync_mode"};
  std::string current_ee_pose_topic_{"/am_ee_pose/current_ee_pose"};
  double target_timeout_s_{0.2};

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr target_preview_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr follower_arm_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr follower_gripper_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr follower_sync_status_sub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr arm_command_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr gripper_command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr leader_sync_mode_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_ee_pose_pub_;

  const std::vector<std::string> arm_joint_names_{
    "joint_1", "joint_2", "joint_3", "joint_4"};
  const std::string gripper_joint_name_{"joint_5"};
  uint64_t last_control_sample_us_{0};
  bool input_gate_open_{false};
  bool arm_target_initialized_{false};
  bool warned_missing_prerequisites_{false};
  bool reported_invalid_vehicle_state_{false};
  bool inference_fault_reported_{false};
  bool external_target_active_{false};
};

}  // namespace am_ee_pose_commander

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<am_ee_pose_commander::AmEePoseCommander>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("am_ee_pose_commander"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
