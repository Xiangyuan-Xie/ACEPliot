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

#include <am_position_motor_mode.hpp>
#include <algorithm>
#include <cmath>
#include <functional>
#include <inference/onnx_ort_backend.hpp>
#include <math.hpp>
#include <px4_ros2/utils/message_version.hpp>
#include <rcl/time.h>
#include <std_msgs/msg/float32_multi_array.hpp>

namespace
{
constexpr std::size_t kActionDim = 4;
// pos_err_b(3) + yaw_err_b(4) + gravity(3) + lin_vel_err(3) + ang_vel_err(3)
constexpr std::size_t kCoreObsDim = 16;
constexpr std::size_t kArmJointObsDim = 4;
constexpr std::size_t kPolicyObsDim = kCoreObsDim + kArmJointObsDim + kActionDim;
static_assert(kPolicyObsDim == 24);
constexpr float kCmdZeroEps = 1e-6f;
}  // namespace

AmPositionMotorMode::AmPositionMotorMode(
  rclcpp::Node & node,
  const std::string & mode_name,
  bool activate_disarmed,
  const std::string & topic_namespace_prefix,
  const std::string & root_dir)
: RLModeBase(
    node,
    std::make_unique<OnnxOrtBackend>(&node),
    px4_ros2::ModeBase::Settings(mode_name).activateEvenWhileDisarmed(activate_disarmed),
    topic_namespace_prefix,
    root_dir),
  action_obs_buffer_(static_cast<int>(kActionDim), 1)
{
  // Configure mode requirements and switch to arm-extended robot data.
  modeRequirements().local_position = true;
  setRobotData(std::make_unique<ArmRobotData>(*this));
  configureClockSource(node);

  // Initialize motor output and prefill action history for stable observation shape.
  motor_setpoint_ = std::make_shared<px4_ros2::DirectActuatorsSetpointType>(*this);
  for (int i = 0; i < action_obs_buffer_.get_history_length(); ++i) {
    action_obs_buffer_.insert(std::vector<float>(kActionDim, 0.0f));
  }

  // Configure Offboard reference interface.
  offboard_control_mode_topic_ +=
    px4_ros2::getMessageNameVersion<px4_msgs::msg::OffboardControlMode>();
  trajectory_setpoint_topic_ +=
    px4_ros2::getMessageNameVersion<px4_msgs::msg::TrajectorySetpoint>();

  if (!node.has_parameter("offboard_control_mode_topic")) {
    node.declare_parameter("offboard_control_mode_topic", offboard_control_mode_topic_);
  }
  if (!node.has_parameter("trajectory_setpoint_topic")) {
    node.declare_parameter("trajectory_setpoint_topic", trajectory_setpoint_topic_);
  }
  if (!node.has_parameter("offboard_setpoint_timeout_s")) {
    node.declare_parameter("offboard_setpoint_timeout_s", offboard_setpoint_timeout_s_);
  }

  offboard_control_mode_topic_ = node.get_parameter("offboard_control_mode_topic").as_string();
  trajectory_setpoint_topic_ = node.get_parameter("trajectory_setpoint_topic").as_string();
  offboard_setpoint_timeout_s_ =
    std::max(0.0, node.get_parameter("offboard_setpoint_timeout_s").as_double());

  offboard_control_mode_sub_ = node.create_subscription<px4_msgs::msg::OffboardControlMode>(
    offboard_control_mode_topic_,
    10,
    std::bind(&AmPositionMotorMode::offboardControlModeCallback, this, std::placeholders::_1));
  trajectory_setpoint_sub_ = node.create_subscription<px4_msgs::msg::TrajectorySetpoint>(
    trajectory_setpoint_topic_,
    10,
    std::bind(&AmPositionMotorMode::trajectorySetpointCallback, this, std::placeholders::_1));
}

void AmPositionMotorMode::onActivate()
{
  rnnState().initialize(backend());
  has_offboard_control_mode_msg_ = false;
  has_trajectory_setpoint_msg_ = false;
  hover_lock_active_ = false;
  current_cmd_ref_ = CommandReference{};
  warned_waiting_for_sim_time_ = false;
}

void AmPositionMotorMode::onDeactivate() {}

void AmPositionMotorMode::getObservation(TensorMap & inputs, float dt_s)
{
  // Refresh targets and logs, then read the latest robot state.
  updateTargets(dt_s);
  logTargetValues(dt_s);
  const auto * arm_data = armRobotData();

  const auto & root_pos_w = robotData()->RootPosW();
  const auto & root_quat_w = robotData()->RootQuatW();
  const auto & projected_gravity = robotData()->ProjectedGravityB();
  const auto & lin_vel_b = robotData()->RootLinVelB();
  const auto & ang_vel_b = robotData()->RootAngVelB();

  const CommandReference & cmd_ref = current_cmd_ref_;

  const Eigen::Vector3f base_lin_vel_cmd_b = cmd_ref.desired_lin_vel_b;
  const Eigen::Vector3f base_ang_vel_cmd_b = cmd_ref.desired_ang_vel_b;

  // --- pos_err_b (3): position error in body frame with per-axis gating ---
  Eigen::Vector3f pos_err_b = Eigen::Vector3f::Zero();
  auto [desired_pos_b, _q_unused] =
    subtract_frame_transforms(root_pos_w, root_quat_w, cmd_ref.desired_pos_w);
  if (desired_pos_b.allFinite()) {
    pos_err_b = desired_pos_b;
  }
  if (cmd_ref.lin_cmd_active[0]) {
    pos_err_b.x() = 0.0f;
  }
  if (cmd_ref.lin_cmd_active[1]) {
    pos_err_b.y() = 0.0f;
  }
  if (cmd_ref.lin_cmd_active[2]) {
    pos_err_b.z() = 0.0f;
  }

  // --- yaw_err_b (4): yaw-gated planar rotation matrix ---
  const float desired_yaw_w = std::get<2>(euler_xyz_from_quat(cmd_ref.desired_quat_w));
  float yaw_err = wrapToPi(desired_yaw_w - robotData()->HeadingW());
  if (cmd_ref.ang_cmd_active[2]) {
    yaw_err = 0.0f;
  }
  const float cos_yaw = std::cos(yaw_err);
  const float sin_yaw = std::sin(yaw_err);

  const Eigen::Vector3f lin_vel_err_b = base_lin_vel_cmd_b - lin_vel_b;
  const Eigen::Vector3f ang_vel_err_b = base_ang_vel_cmd_b - ang_vel_b;
  const std::vector<float> empty_vec;
  const auto & arm_position = arm_data ? arm_data->ArmPosition() : empty_vec;
  const auto action_history = getActionObsBuffer().get_flattened_history();

  const auto append_or_zero_arm_channel =
    [this, &inputs](std::vector<float> & dst, const std::vector<float> & src,
      const char * label) -> bool {
      if (src.empty()) {
        dst.insert(dst.end(), kArmJointObsDim, 0.0f);
        return true;
      }
      if (src.size() != kArmJointObsDim) {
        RCLCPP_ERROR(
          node().get_logger(),
          "Arm observation channel '%s' has size %zu, expected %zu. Skipping inference to avoid misaligned observations.",
          label,
          src.size(),
          kArmJointObsDim);
        inputs.clear();
        return false;
      }
      dst.insert(dst.end(), src.begin(), src.end());
      return true;
    };

  // Build policy observation matching training order:
  // [pos_err_b, yaw_err_b, projected_gravity, lin_vel_err_b, ang_vel_err_b,
  //  servo_position, action_history]
  std::vector<float> obs;
  obs.reserve(kPolicyObsDim);

  // 1) pos_err_b (3)
  obs.push_back(pos_err_b.x());
  obs.push_back(pos_err_b.y());
  obs.push_back(pos_err_b.z());

  // 2) yaw_err_b (4) - flattened 2x2 rotation matrix, row-major
  obs.push_back(cos_yaw);
  obs.push_back(-sin_yaw);
  obs.push_back(sin_yaw);
  obs.push_back(cos_yaw);

  // 3) projected_gravity (3)
  obs.push_back(projected_gravity.x());
  obs.push_back(projected_gravity.y());
  obs.push_back(projected_gravity.z());

  // 4) lin_vel_err_b (3)
  obs.push_back(lin_vel_err_b.x());
  obs.push_back(lin_vel_err_b.y());
  obs.push_back(lin_vel_err_b.z());

  // 5) ang_vel_err_b (3)
  obs.push_back(ang_vel_err_b.x());
  obs.push_back(ang_vel_err_b.y());
  obs.push_back(ang_vel_err_b.z());

  // 6) servo_position
  if (!append_or_zero_arm_channel(obs, arm_position, "servo_position")) {
    return;
  }
  // 7) action_history
  if (action_history.size() != kActionDim) {
    RCLCPP_ERROR(
      node().get_logger(),
      "Action history size %zu does not match expected policy action dimension %zu. Skipping inference to avoid misaligned observations.",
      action_history.size(),
      kActionDim);
    inputs.clear();
    return;
  }
  obs.insert(obs.end(), action_history.begin(), action_history.end());

  if (backend()->hasInput("obs")) {
    const auto shape = backend()->inputShape("obs");
    if (!shape.empty()) {
      const std::size_t expected_obs_dim = static_cast<std::size_t>(shape.back());
      if (obs.size() != expected_obs_dim) {
        RCLCPP_ERROR(
          node().get_logger(),
          "Observation size %zu does not match model input dimension %zu. Skipping inference to avoid silently misaligned observations.",
          obs.size(),
          expected_obs_dim);
        inputs.clear();
        return;
      }
    }
  }

  inputs["obs"] = std::move(obs);
  rnnState().appendInput(inputs);
}

void AmPositionMotorMode::applyAction(const TensorMap & action, float dt_s)
{
  (void)dt_s;

  // Validate model output and extract action vector; ignore on missing or wrong type.
  auto it = action.find("actions");
  if (it == action.end() || !std::holds_alternative<std::vector<float>>(it->second)) {
    return;
  }

  const auto & out_vec = std::get<std::vector<float>>(it->second);
  if (out_vec.size() != kActionDim) {
    RCLCPP_ERROR(
      node().get_logger(),
      "Policy action size %zu does not match expected motor action dimension %zu.",
      out_vec.size(),
      kActionDim);
    return;
  }
  const bool actions_finite = std::all_of(
    out_vec.begin(), out_vec.end(), [](float value) {return std::isfinite(value);});
  if (!actions_finite) {
    RCLCPP_ERROR(node().get_logger(), "Policy produced a non-finite motor action.");
    return;
  }

  std::vector<float> applied_vec(out_vec);
  for (float & value : applied_vec) {
    value = std::clamp(value, 0.0f, 1.0f);
  }

  // The exported Beta-policy mean is already normalized to [0, 1]. Use the
  // applied command for both motor output and the next observation.
  getActionObsBuffer().insert(applied_vec);
  rnnState().updateFromOutput(action);

  // Map normalized policy output to motor channels and keep remaining channels at zero.
  using MotorCommands = Eigen::Matrix<
    float,
    px4_ros2::DirectActuatorsSetpointType::kMaxNumMotors,
    1>;
  MotorCommands motor_commands = MotorCommands::Zero();
  for (std::size_t i = 0; i < kActionDim; ++i) {
    motor_commands[static_cast<int>(i)] = applied_vec[i];
  }

  motor_setpoint_->updateMotors(motor_commands);
}

void AmPositionMotorMode::updateTargets(float dt_s)
{
  (void)dt_s;
  const auto & root_pos_w = robotData()->RootPosW();
  const auto & root_quat_w = robotData()->RootQuatW();
  const float root_yaw_w = robotData()->HeadingW();

  const bool external_valid = hasFreshExternalReference();
  Eigen::Vector3f desired_lin_vel_b = Eigen::Vector3f::Zero();
  Eigen::Vector3f desired_ang_vel_b = Eigen::Vector3f::Zero();
  std::array<bool, 3> lin_active{{false, false, false}};
  std::array<bool, 3> pos_active{{false, false, false}};
  std::array<bool, 3> ang_active{{false, false, false}};

  if (!hover_lock_active_) {
    hover_lock_active_ = true;
    hover_lock_pos_w_ = root_pos_w;
    hover_lock_yaw_w_ = root_yaw_w;
  }

  if (external_valid) {
    const AmPositionOffboardReference reference = buildAmPositionOffboardReference(
      last_offboard_control_mode_msg_,
      last_trajectory_setpoint_msg_,
      hover_lock_pos_w_,
      hover_lock_yaw_w_,
      root_quat_w);
    if (reference.valid) {
      desired_lin_vel_b = reference.desired_lin_vel_b;
      desired_ang_vel_b = reference.desired_ang_vel_b;
      pos_active = reference.position_active;
      lin_active = reference.velocity_active;
      ang_active[2] = reference.has_yaw_rate_cmd && std::abs(desired_ang_vel_b.z()) > kCmdZeroEps;
      if (pos_active[0]) {
        hover_lock_pos_w_.x() = reference.desired_pos_w.x();
      }
      if (pos_active[1]) {
        hover_lock_pos_w_.y() = reference.desired_pos_w.y();
      }
      if (pos_active[2]) {
        hover_lock_pos_w_.z() = reference.desired_pos_w.z();
      }
      if (std::isfinite(last_trajectory_setpoint_msg_.yaw)) {
        hover_lock_yaw_w_ = px4_ros2::yawNedToEnu(last_trajectory_setpoint_msg_.yaw);
      }
    }
  }

  // Maintain per-axis lock reference in position channels.
  auto [lock_pos_b, _q_unused] = subtract_frame_transforms(
    root_pos_w, root_quat_w,
    hover_lock_pos_w_);
  if (lock_pos_b.allFinite()) {
    if (lin_active[0] && !pos_active[0]) {
      lock_pos_b.x() = 0.0f;
    }
    if (lin_active[1] && !pos_active[1]) {
      lock_pos_b.y() = 0.0f;
    }
    if (lin_active[2] && !pos_active[2]) {
      lock_pos_b.z() = 0.0f;
    }
    hover_lock_pos_w_ = root_pos_w + (root_quat_w * lock_pos_b);
  } else {
    hover_lock_pos_w_ = root_pos_w;
  }

  // Maintain per-axis lock reference in yaw channel.
  float yaw_err = wrapToPi(hover_lock_yaw_w_ - root_yaw_w);
  if (ang_active[2]) {
    yaw_err = 0.0f;
  }
  hover_lock_yaw_w_ = wrapToPi(root_yaw_w + yaw_err);

  current_cmd_ref_.desired_lin_vel_b = desired_lin_vel_b;
  current_cmd_ref_.desired_ang_vel_b = desired_ang_vel_b;
  current_cmd_ref_.desired_pos_w = hover_lock_pos_w_;
  current_cmd_ref_.desired_quat_w = yawOnlyQuat(hover_lock_yaw_w_);
  current_cmd_ref_.lin_cmd_active = lin_active;
  current_cmd_ref_.ang_cmd_active = ang_active;
  current_cmd_ref_.has_lin_vel_cmd = anyAxisActive(lin_active);
  current_cmd_ref_.has_ang_vel_cmd = anyAxisActive(ang_active);
}

void AmPositionMotorMode::logTargetValues(float dt_s)
{
  (void)dt_s;

  // Log targets only when logger is available to avoid extra runtime overhead.
  if (!flightLogger() || !flightLogger()->isOpen()) {
    return;
  }

  const auto * arm_data = armRobotData();
  if (!arm_data) {
    return;
  }

  rclcpp::Time now;
  if (!getCurrentModeTime(now)) {
    return;
  }
  const double now_s = now.seconds();

  // Log base velocity targets (linear + angular) for replay-time behavior analysis.
  std_msgs::msg::Float32MultiArray target_twist_msg;
  target_twist_msg.data = {
    current_cmd_ref_.desired_lin_vel_b.x(),
    current_cmd_ref_.desired_lin_vel_b.y(),
    current_cmd_ref_.desired_lin_vel_b.z(),
    current_cmd_ref_.desired_ang_vel_b.x(),
    current_cmd_ref_.desired_ang_vel_b.y(),
    current_cmd_ref_.desired_ang_vel_b.z()
  };
  flightLogger()->log("target_twist_cmd_b", target_twist_msg, now_s);

  // Log full position/attitude reference in world frame.
  std_msgs::msg::Float32MultiArray target_pose_msg;
  target_pose_msg.data = {
    current_cmd_ref_.desired_pos_w.x(),
    current_cmd_ref_.desired_pos_w.y(),
    current_cmd_ref_.desired_pos_w.z(),
    current_cmd_ref_.desired_quat_w.w(),
    current_cmd_ref_.desired_quat_w.x(),
    current_cmd_ref_.desired_quat_w.y(),
    current_cmd_ref_.desired_quat_w.z()
  };
  flightLogger()->log("target_pose_ref_w", target_pose_msg, now_s);

  // Log command validity flags for offline debugging.
  std_msgs::msg::Float32MultiArray target_meta_msg;
  target_meta_msg.data = {
    current_cmd_ref_.has_lin_vel_cmd ? 1.0f : 0.0f,
    current_cmd_ref_.has_ang_vel_cmd ? 1.0f : 0.0f
  };
  flightLogger()->log("target_reference_meta", target_meta_msg, now_s);

}

ArmRobotData * AmPositionMotorMode::armRobotData()
{
  return robotDataAs<ArmRobotData>();
}

const ArmRobotData * AmPositionMotorMode::armRobotData() const
{
  return robotDataAs<ArmRobotData>();
}

ObservationBuffer & AmPositionMotorMode::getActionObsBuffer()
{
  return action_obs_buffer_;
}

void AmPositionMotorMode::offboardControlModeCallback(
  const px4_msgs::msg::OffboardControlMode::SharedPtr msg)
{
  last_offboard_control_mode_msg_ = *msg;
  if (!updateExternalReferenceReceiptTime(last_offboard_control_mode_time_)) {
    has_offboard_control_mode_msg_ = false;
    return;
  }
  has_offboard_control_mode_msg_ = true;
}

void AmPositionMotorMode::trajectorySetpointCallback(
  const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg)
{
  last_trajectory_setpoint_msg_ = *msg;
  if (!updateExternalReferenceReceiptTime(last_trajectory_setpoint_time_)) {
    has_trajectory_setpoint_msg_ = false;
    return;
  }
  has_trajectory_setpoint_msg_ = true;
}

bool AmPositionMotorMode::hasFreshExternalReference()
{
  if (!has_offboard_control_mode_msg_ || !has_trajectory_setpoint_msg_) {
    return false;
  }
  rclcpp::Time now;
  if (!getCurrentModeTime(now)) {
    return false;
  }
  const double control_mode_age_s = (now - last_offboard_control_mode_time_).seconds();
  const double trajectory_age_s = (now - last_trajectory_setpoint_time_).seconds();
  return control_mode_age_s <= offboard_setpoint_timeout_s_ &&
         trajectory_age_s <= offboard_setpoint_timeout_s_;
}

void AmPositionMotorMode::simClockCallback(
  const rosgraph_msgs::msg::Clock::SharedPtr msg)
{
  latest_sim_time_ = rclcpp::Time(msg->clock, RCL_ROS_TIME);
  has_sim_time_ = true;
  warned_waiting_for_sim_time_ = false;

  auto clock = node().get_clock();
  std::lock_guard<std::mutex> clock_lock(clock->get_clock_mutex());
  auto * clock_handle = clock->get_clock_handle();

  if (!clock->ros_time_is_active()) {
    const auto ret = rcl_enable_ros_time_override(clock_handle);
    if (ret != RCL_RET_OK) {
      RCLCPP_ERROR(
        node().get_logger(),
        "Failed to enable ROS time override from '%s': %s",
        sim_clock_topic_.c_str(),
        rcl_get_error_string().str);
      rcl_reset_error();
      return;
    }
  }

  const auto ret = rcl_set_ros_time_override(clock_handle, latest_sim_time_.nanoseconds());
  if (ret != RCL_RET_OK) {
    RCLCPP_ERROR(
      node().get_logger(),
      "Failed to update ROS time from '%s': %s",
      sim_clock_topic_.c_str(),
      rcl_get_error_string().str);
    rcl_reset_error();
  }
}

bool AmPositionMotorMode::getCurrentModeTime(rclcpp::Time & now)
{
  if (!use_sim_time_) {
    now = node().get_clock()->now();
    return true;
  }

  if (!has_sim_time_) {
    return false;
  }

  now = latest_sim_time_;
  return true;
}

bool AmPositionMotorMode::updateExternalReferenceReceiptTime(rclcpp::Time & receipt_time)
{
  if (!getCurrentModeTime(receipt_time)) {
    if (use_sim_time_ && !warned_waiting_for_sim_time_) {
      RCLCPP_WARN(
        node().get_logger(),
        "Ignoring Offboard references until the first message arrives on '%s'.",
        sim_clock_topic_.c_str());
      warned_waiting_for_sim_time_ = true;
    }
    return false;
  }
  return true;
}

void AmPositionMotorMode::configureClockSource(rclcpp::Node & node)
{
  node.get_parameter("use_sim_time", use_sim_time_);

  if (!node.has_parameter("sim_clock_topic")) {
    node.declare_parameter("sim_clock_topic", sim_clock_topic_);
  }
  sim_clock_topic_ = node.get_parameter("sim_clock_topic").as_string();

  if (!use_sim_time_) {
    return;
  }

  sim_clock_sub_ = node.create_subscription<rosgraph_msgs::msg::Clock>(
    sim_clock_topic_,
    rclcpp::ClockQoS(),
    std::bind(&AmPositionMotorMode::simClockCallback, this, std::placeholders::_1));
}
