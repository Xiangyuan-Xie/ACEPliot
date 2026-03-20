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
#include <inference/onnx_ort_backend.hpp>
#include <math.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

namespace
{
constexpr std::size_t kActionDim = 4;
constexpr std::size_t kCoreObsDim = 9;
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
}

void RlMCArmPositionDirectActuatorsMode::onActivate() {}

void RlMCArmPositionDirectActuatorsMode::onDeactivate() {}

void RlMCArmPositionDirectActuatorsMode::getObservation(TensorMap & inputs, float dt_s)
{
  // Refresh targets and logs, then read the latest robot state.
  updateTargets(dt_s);
  logTargetValues(dt_s);
  const auto * arm_data = armRobotData();

  const auto & projected_gravity = robotData()->ProjectedGravityB();
  const auto & lin_vel_b = robotData()->RootLinVelB();
  const auto & ang_vel_b = robotData()->RootAngVelB();
  const Eigen::Vector3f base_lin_vel_cmd_b =
    arm_data ? arm_data->BaseLinVelCmdB() : Eigen::Vector3f::Zero();
  const Eigen::Vector3f base_ang_vel_cmd_b =
    arm_data ? arm_data->BaseAngVelCmdB() : Eigen::Vector3f::Zero();

  const Eigen::Vector3f lin_vel_err_b = base_lin_vel_cmd_b - lin_vel_b;
  const Eigen::Vector3f ang_vel_err_b = base_ang_vel_cmd_b - ang_vel_b;
  const std::vector<float> empty_vec;
  const auto & arm_position = arm_data ? arm_data->ArmPosition() : empty_vec;
  const auto & arm_command = arm_data ? arm_data->ArmCommand() : empty_vec;
  const auto & arm_velocity = arm_data ? arm_data->ArmVelocity() : empty_vec;
  const auto action_history = getActionObsBuffer().get_flattened_history();

  // Build policy observation: base errors + arm state + action history.
  std::vector<float> obs;
  obs.reserve(
    kCoreObsDim + arm_command.size() + arm_position.size() + arm_velocity.size() +
    action_history.size());

  obs.push_back(projected_gravity.x());
  obs.push_back(projected_gravity.y());
  obs.push_back(projected_gravity.z());

  obs.push_back(lin_vel_err_b.x());
  obs.push_back(lin_vel_err_b.y());
  obs.push_back(lin_vel_err_b.z());

  obs.push_back(ang_vel_err_b.x());
  obs.push_back(ang_vel_err_b.y());
  obs.push_back(ang_vel_err_b.z());

  obs.insert(obs.end(), arm_command.begin(), arm_command.end());
  obs.insert(obs.end(), arm_position.begin(), arm_position.end());
  obs.insert(obs.end(), arm_velocity.begin(), arm_velocity.end());
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

  // Log base velocity targets (linear + angular) for replay-time behavior analysis.
  std_msgs::msg::Float32MultiArray target_twist_msg;
  target_twist_msg.data = {
    arm_data->BaseLinVelCmdB().x(),
    arm_data->BaseLinVelCmdB().y(),
    arm_data->BaseLinVelCmdB().z(),
    arm_data->BaseAngVelCmdB().x(),
    arm_data->BaseAngVelCmdB().y(),
    arm_data->BaseAngVelCmdB().z()
  };
  flightLogger()->log(
    "target_twist_cmd_b",
    target_twist_msg,
    node().get_clock()->now().seconds());

  // Log arm command trajectory to inspect coupling with policy outputs.
  std_msgs::msg::Float32MultiArray arm_command_msg;
  arm_command_msg.data = arm_data->ArmCommand();
  flightLogger()->log(
    "arm_command",
    arm_command_msg,
    node().get_clock()->now().seconds());
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
