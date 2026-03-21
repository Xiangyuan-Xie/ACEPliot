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

#include <rl_mc_arm_position_direct_actuators_mode.hpp>
#include <algorithm>
#include <cmath>
#include <functional>
#include <inference/onnx_ort_backend.hpp>
#include <math.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

namespace
{
constexpr std::size_t kActionDim = 4;
constexpr std::size_t kCoreObsDim = 21;  // pos_err_b(3) + att_err_b(9) + gravity(3) + lin_vel_err(3) + ang_vel_err(3)

Eigen::Quaternionf yawOnlyQuat(float yaw_w)
{
  return Eigen::Quaternionf(Eigen::AngleAxisf(yaw_w, Eigen::Vector3f::UnitZ()));
}
}  // namespace

RlMCArmPositionDirectActuatorsMode::RlMCArmPositionDirectActuatorsMode(
  rclcpp::Node & node,
  const std::string & mode_name,
  bool activate_disarmed,
  const std::string & topic_namespace_prefix,
  const std::string & root_dir)
: RLModeBase(
    node,
    std::make_unique<OnnxOrtBackend>(node),
    px4_ros2::ModeBase::Settings(mode_name).activateEvenWhileDisarmed(activate_disarmed),
    topic_namespace_prefix,
    root_dir),
  action_obs_buffer_(static_cast<int>(kActionDim), 1)
{
  // Configure mode requirements and switch to arm-extended robot data.
  modeRequirements().local_position = true;
  setRobotData(std::make_unique<ArmRobotData>(*this));

  // Initialize direct motor output and prefill action history for stable observation shape.
  direct_actuators_ = std::make_shared<px4_ros2::DirectActuatorsSetpointType>(*this);
  for (int i = 0; i < action_obs_buffer_.get_history_length(); ++i) {
    action_obs_buffer_.insert(std::vector<float>(kActionDim, 0.0f));
  }

  // Configure external velocity command interface.
  if (!node.has_parameter("cmd_vel_topic")) {
    node.declare_parameter("cmd_vel_topic", cmd_vel_topic_);
  }
  if (!node.has_parameter("cmd_vel_timeout_s")) {
    node.declare_parameter("cmd_vel_timeout_s", cmd_vel_timeout_s_);
  }

  cmd_vel_topic_ = node.get_parameter("cmd_vel_topic").as_string();
  cmd_vel_timeout_s_ = std::max(0.0, node.get_parameter("cmd_vel_timeout_s").as_double());

  cmd_vel_sub_ = node.create_subscription<geometry_msgs::msg::TwistStamped>(
    cmd_vel_topic_,
    10,
    std::bind(&RlMCArmPositionDirectActuatorsMode::cmdVelCallback, this, std::placeholders::_1));
}

void RlMCArmPositionDirectActuatorsMode::onActivate()
{
  rnnState().initialize(backend());
  has_cmd_vel_msg_ = false;
  hover_lock_active_ = false;
  current_cmd_ref_ = CommandReference{};
}

void RlMCArmPositionDirectActuatorsMode::onDeactivate() {}

void RlMCArmPositionDirectActuatorsMode::getObservation(TensorMap & inputs, float dt_s)
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

  // --- pos_err_b (3): position error in body frame, zeroed when velocity command active ---
  const bool has_lin_vel_cmd = cmd_ref.has_lin_vel_cmd;
  Eigen::Vector3f pos_err_b = Eigen::Vector3f::Zero();
  if (!has_lin_vel_cmd) {
    auto [desired_pos_b, _q_unused] =
      subtract_frame_transforms(root_pos_w, root_quat_w, cmd_ref.desired_pos_w);
    if (desired_pos_b.allFinite()) {
      pos_err_b = desired_pos_b;
    }
  }

  // --- att_err_b (9): attitude error as flattened rotation matrix, identity when ang_vel active ---
  const bool has_ang_vel_cmd = cmd_ref.has_ang_vel_cmd;
  Eigen::Matrix3f rot_matrix = Eigen::Matrix3f::Identity();
  if (!has_ang_vel_cmd) {
    auto [_t_unused, att_err_quat] =
      subtract_frame_transforms(root_pos_w, root_quat_w, root_pos_w, cmd_ref.desired_quat_w);
    auto [roll_err, pitch_err, yaw_err] = euler_xyz_from_quat(att_err_quat);
    Eigen::Quaternionf att_err_no_yaw = quat_from_euler_xyz(roll_err, pitch_err, 0.0f);
    rot_matrix = rotation_matrix_from_quat(att_err_no_yaw);
  }

  const Eigen::Vector3f lin_vel_err_b = base_lin_vel_cmd_b - lin_vel_b;
  const Eigen::Vector3f ang_vel_err_b = base_ang_vel_cmd_b - ang_vel_b;
  const std::vector<float> empty_vec;
  const auto & arm_position = arm_data ? arm_data->ArmPosition() : empty_vec;
  const auto & arm_command = arm_data ? arm_data->ArmCommand() : empty_vec;
  const auto & arm_velocity = arm_data ? arm_data->ArmVelocity() : empty_vec;
  const auto action_history = getActionObsBuffer().get_flattened_history();

  // Build policy observation matching training order:
  // [pos_err_b, att_err_b, projected_gravity, lin_vel_err_b, ang_vel_err_b,
  //  arm_command, servo_position, servo_velocity, action_history]
  std::vector<float> obs;
  obs.reserve(
    kCoreObsDim + arm_command.size() + arm_position.size() + arm_velocity.size() +
    action_history.size());

  // 1) pos_err_b (3)
  obs.push_back(pos_err_b.x());
  obs.push_back(pos_err_b.y());
  obs.push_back(pos_err_b.z());

  // 2) att_err_b (9) — flattened rotation matrix, row-major
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      obs.push_back(rot_matrix(r, c));
    }
  }

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

  // 6) arm_command
  obs.insert(obs.end(), arm_command.begin(), arm_command.end());
  // 7) servo_position
  obs.insert(obs.end(), arm_position.begin(), arm_position.end());
  // 8) servo_velocity
  obs.insert(obs.end(), arm_velocity.begin(), arm_velocity.end());
  // 9) action_history
  obs.insert(obs.end(), action_history.begin(), action_history.end());

  inputs["obs"] = std::move(obs);
  rnnState().appendInput(inputs);
}

void RlMCArmPositionDirectActuatorsMode::applyAction(const TensorMap & action, float dt_s)
{
  (void)dt_s;

  // Validate model output and extract action vector; ignore on missing or wrong type.
  auto it = action.find("actions");
  if (it == action.end() || !std::holds_alternative<std::vector<float>>(it->second)) {
    return;
  }

  // Update action history and RNN state for next-cycle observation.
  const auto & out_vec = std::get<std::vector<float>>(it->second);
  getActionObsBuffer().insert(out_vec);
  rnnState().updateFromOutput(action);
  auto clamped_vec = clamp_vector(out_vec, 0.0f, 1.0f);

  // Map normalized policy output to motor channels and keep remaining channels at zero.
  using MotorCommands = Eigen::Matrix<
    float,
    px4_ros2::DirectActuatorsSetpointType::kMaxNumMotors,
    1>;
  MotorCommands motor_commands = MotorCommands::Zero();
  const size_t max_copy = std::min(clamped_vec.size(), kActionDim);
  for (size_t i = 0; i < max_copy; ++i) {
    motor_commands[static_cast<int>(i)] = clamped_vec[i];
  }

  direct_actuators_->updateMotors(motor_commands);
}

void RlMCArmPositionDirectActuatorsMode::updateTargets(float dt_s)
{
  (void)dt_s;
  const auto & root_pos_w = robotData()->RootPosW();
  const float root_yaw_w = robotData()->HeadingW();

  const bool external_valid = hasFreshExternalCmd();
  Eigen::Vector3f desired_lin_vel_b = Eigen::Vector3f::Zero();
  Eigen::Vector3f desired_ang_vel_b = Eigen::Vector3f::Zero();
  if (external_valid) {
    desired_lin_vel_b.x() = static_cast<float>(last_cmd_vel_msg_.twist.linear.x);
    desired_lin_vel_b.y() = static_cast<float>(last_cmd_vel_msg_.twist.linear.y);
    desired_lin_vel_b.z() = static_cast<float>(last_cmd_vel_msg_.twist.linear.z);
    desired_ang_vel_b.z() = static_cast<float>(last_cmd_vel_msg_.twist.angular.z);
  }
  const bool has_lin_vel_cmd = desired_lin_vel_b.norm() > 1e-6f;
  const bool has_ang_vel_cmd = desired_ang_vel_b.norm() > 1e-6f;
  const bool use_hover_lock = !external_valid || (!has_lin_vel_cmd && !has_ang_vel_cmd);

  if (use_hover_lock) {
    if (!hover_lock_active_) {
      hover_lock_active_ = true;
      hover_lock_pos_w_ = root_pos_w;
      hover_lock_yaw_w_ = root_yaw_w;
    }

    current_cmd_ref_.desired_lin_vel_b.setZero();
    current_cmd_ref_.desired_ang_vel_b.setZero();
    current_cmd_ref_.desired_pos_w = hover_lock_pos_w_;
    current_cmd_ref_.desired_quat_w = yawOnlyQuat(hover_lock_yaw_w_);
    current_cmd_ref_.has_lin_vel_cmd = false;
    current_cmd_ref_.has_ang_vel_cmd = false;
  } else {
    hover_lock_active_ = false;
    current_cmd_ref_.desired_lin_vel_b = desired_lin_vel_b;
    current_cmd_ref_.desired_ang_vel_b = desired_ang_vel_b;
    current_cmd_ref_.desired_pos_w = root_pos_w;
    current_cmd_ref_.desired_quat_w = yawOnlyQuat(root_yaw_w);
    current_cmd_ref_.has_lin_vel_cmd = has_lin_vel_cmd;
    current_cmd_ref_.has_ang_vel_cmd = has_ang_vel_cmd;
  }
}

void RlMCArmPositionDirectActuatorsMode::logTargetValues(float dt_s)
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

  const double now_s = node().get_clock()->now().seconds();

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

  // Log arm command trajectory to inspect coupling with policy outputs.
  std_msgs::msg::Float32MultiArray arm_command_msg;
  arm_command_msg.data = arm_data->ArmCommand();
  flightLogger()->log("arm_command", arm_command_msg, now_s);
}

ArmRobotData * RlMCArmPositionDirectActuatorsMode::armRobotData()
{
  return robotDataAs<ArmRobotData>();
}

const ArmRobotData * RlMCArmPositionDirectActuatorsMode::armRobotData() const
{
  return robotDataAs<ArmRobotData>();
}

ObservationBuffer & RlMCArmPositionDirectActuatorsMode::getActionObsBuffer()
{
  return action_obs_buffer_;
}

void RlMCArmPositionDirectActuatorsMode::cmdVelCallback(
  const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  last_cmd_vel_msg_ = *msg;

  if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
    last_cmd_vel_time_ = node().get_clock()->now();
  } else {
    last_cmd_vel_time_ = rclcpp::Time(msg->header.stamp, node().get_clock()->get_clock_type());
  }

  has_cmd_vel_msg_ = true;
}

bool RlMCArmPositionDirectActuatorsMode::hasFreshExternalCmd()
{
  if (!has_cmd_vel_msg_) {
    return false;
  }
  const double age_s = (node().get_clock()->now() - last_cmd_vel_time_).seconds();
  return age_s <= cmd_vel_timeout_s_;
}
