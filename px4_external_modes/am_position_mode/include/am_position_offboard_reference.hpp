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

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

struct AmPositionOffboardReference
{
  bool valid{false};
  Eigen::Vector3f desired_lin_vel_b{Eigen::Vector3f::Zero()};
  Eigen::Vector3f desired_ang_vel_b{Eigen::Vector3f::Zero()};
  Eigen::Vector3f desired_pos_w{Eigen::Vector3f::Zero()};
  Eigen::Quaternionf desired_quat_w{Eigen::Quaternionf::Identity()};
  std::array<bool, 3> position_active{{false, false, false}};
  std::array<bool, 3> velocity_active{{false, false, false}};
  bool has_position_cmd{false};
  bool has_velocity_cmd{false};
  bool has_yaw_rate_cmd{false};
};

bool isSupportedAmPositionOffboardMode(const px4_msgs::msg::OffboardControlMode & mode);

AmPositionOffboardReference buildAmPositionOffboardReference(
  const px4_msgs::msg::OffboardControlMode & mode,
  const px4_msgs::msg::TrajectorySetpoint & setpoint,
  const Eigen::Vector3f & fallback_pos_w,
  float fallback_yaw_w,
  const Eigen::Quaternionf & root_quat_w);
