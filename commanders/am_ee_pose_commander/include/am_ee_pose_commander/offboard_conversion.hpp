/****************************************************************************
 * Copyright (c) 2026 Xiangyuan Xie <dragonboat_xxy@163.com>.
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#pragma once

#include <am_ee_pose_commander/policy_contract.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

#include <cstdint>

namespace am_ee_pose_commander
{

px4_msgs::msg::OffboardControlMode makeVelocityOffboardControlMode(uint64_t timestamp_us);

px4_msgs::msg::TrajectorySetpoint makeVelocityTrajectorySetpoint(
  const WorldVelocityCommand & command,
  uint64_t timestamp_us);

}  // namespace am_ee_pose_commander
