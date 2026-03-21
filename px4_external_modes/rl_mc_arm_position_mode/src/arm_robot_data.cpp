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
#include <functional>

ArmRobotData::ArmRobotData(px4_ros2::ModeBase & mode_base)
: RobotData(mode_base)
{
  // Declare and read arm topic parameters with runtime remapping support.
  auto & node_ref = mode_base.node();
  if (!node_ref.has_parameter("arm_command_topic")) {
    node_ref.declare_parameter("arm_command_topic", "/arm/command");
  }
  if (!node_ref.has_parameter("arm_state_topic")) {
    node_ref.declare_parameter("arm_state_topic", "/arm/state");
  }

  const auto arm_command_topic = node_ref.get_parameter("arm_command_topic").as_string();
  const auto arm_state_topic = node_ref.get_parameter("arm_state_topic").as_string();

  // Subscribe to arm command and arm state topics.
  arm_command_sub_ = node_ref.create_subscription<sensor_msgs::msg::JointState>(
    arm_command_topic,
    10,
    std::bind(&ArmRobotData::armCommandCallback, this, std::placeholders::_1));
  arm_state_sub_ = node_ref.create_subscription<sensor_msgs::msg::JointState>(
    arm_state_topic,
    10,
    std::bind(&ArmRobotData::armStateCallback, this, std::placeholders::_1));
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
