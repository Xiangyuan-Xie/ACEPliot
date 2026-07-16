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

namespace
{
constexpr std::size_t kPolicyArmJointCount = 4;
}

ArmRobotData::ArmRobotData(px4_ros2::ModeBase & mode_base)
: RobotData(mode_base)
{
  // Declare and read the arm state topic with runtime remapping support.
  auto & node_ref = mode_base.node();
  if (!node_ref.has_parameter("arm_state_topic")) {
    node_ref.declare_parameter("arm_state_topic", "/arm/state");
  }

  const auto arm_state_topic = node_ref.get_parameter("arm_state_topic").as_string();
  const auto best_effort_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

  // Subscribe only to the arm state topic because policy observations no longer
  // include a separate command channel.
  arm_state_sub_ = node_ref.create_subscription<sensor_msgs::msg::JointState>(
    arm_state_topic,
    best_effort_qos,
    std::bind(&ArmRobotData::armStateCallback, this, std::placeholders::_1));
}

const std::vector<float> & ArmRobotData::ArmPosition() const
{
  return arm_position_;
}

void ArmRobotData::armStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (msg->position.size() < kPolicyArmJointCount) {
    arm_position_.clear();
    return;
  }

  // The gripper remains in JointState for compatibility but is not a policy input.
  arm_position_.assign(
    msg->position.begin(),
    msg->position.begin() + static_cast<std::ptrdiff_t>(kPolicyArmJointCount));
}
