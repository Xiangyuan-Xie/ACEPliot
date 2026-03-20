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

#include <arm_robot_data.hpp>
#include <cmath>
#include <functional>

ArmRobotData::ArmRobotData(px4_ros2::ModeBase & mode_base)
: RobotData(mode_base)
{
  // Declare and read arm/base-command topic parameters with runtime remapping support.
  auto & node_ref = mode_base.node();
  if (!node_ref.has_parameter("arm_command_topic")) {
    node_ref.declare_parameter("arm_command_topic", "/arm/command");
  }
  if (!node_ref.has_parameter("arm_state_topic")) {
    node_ref.declare_parameter("arm_state_topic", "/arm/state");
  }
  if (!node_ref.has_parameter("base_velocity_cmd_topic")) {
    node_ref.declare_parameter("base_velocity_cmd_topic", "/fmu/out/manual_control_setpoint");
  }

  const auto arm_command_topic = node_ref.get_parameter("arm_command_topic").as_string();
  const auto arm_state_topic = node_ref.get_parameter("arm_state_topic").as_string();
  const auto base_velocity_cmd_topic =
    node_ref.get_parameter("base_velocity_cmd_topic").as_string();

  // Subscribe to arm command, arm state, and base velocity command topics.
  arm_command_sub_ = node_ref.create_subscription<sensor_msgs::msg::JointState>(
    arm_command_topic,
    10,
    std::bind(&ArmRobotData::armCommandCallback, this, std::placeholders::_1));
  arm_state_sub_ = node_ref.create_subscription<sensor_msgs::msg::JointState>(
    arm_state_topic,
    10,
    std::bind(&ArmRobotData::armStateCallback, this, std::placeholders::_1));
  base_velocity_cmd_sub_ = node_ref.create_subscription<px4_msgs::msg::ManualControlSetpoint>(
    base_velocity_cmd_topic,
    rclcpp::QoS(1).best_effort(),
    std::bind(&ArmRobotData::baseVelocityCommandCallback, this, std::placeholders::_1));
}

const std::vector<float> & ArmRobotData::ArmPosition() const
{
  return arm_position_;
}

const std::vector<float> & ArmRobotData::ArmCommand() const
{
  return arm_command_;
}

const std::vector<float> & ArmRobotData::ArmVelocity() const
{
  return arm_velocity_;
}

const Eigen::Vector3f & ArmRobotData::BaseLinVelCmdB() const
{
  return base_lin_vel_cmd_b_;
}

const Eigen::Vector3f & ArmRobotData::BaseAngVelCmdB() const
{
  return base_ang_vel_cmd_b_;
}

void ArmRobotData::armCommandCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // Cache latest joint commands for observation construction and logging.
  arm_command_.assign(msg->position.begin(), msg->position.end());
}

void ArmRobotData::armStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // Cache latest joint states while keeping command and state channels decoupled.
  arm_position_.assign(msg->position.begin(), msg->position.end());
  arm_velocity_.assign(msg->velocity.begin(), msg->velocity.end());
}

void ArmRobotData::baseVelocityCommandCallback(
  const px4_msgs::msg::ManualControlSetpoint::SharedPtr msg)
{
  // Filter NaN/Inf values to prevent invalid inputs from polluting state.
  const auto safe_value = [](float value) {
      return std::isfinite(value) ? value : 0.0f;
    };

  // Use pitch/roll/throttle as body-frame linear velocity commands.
  base_lin_vel_cmd_b_.x() = safe_value(msg->pitch);
  base_lin_vel_cmd_b_.y() = safe_value(msg->roll);
  base_lin_vel_cmd_b_.z() = safe_value(msg->throttle);

  // Use yaw as angular-rate target and keep other angular components zero.
  base_ang_vel_cmd_b_.setZero();
  base_ang_vel_cmd_b_.z() = safe_value(msg->yaw);
}
