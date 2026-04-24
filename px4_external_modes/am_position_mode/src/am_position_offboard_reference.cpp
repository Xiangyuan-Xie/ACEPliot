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

#include <am_position_offboard_reference.hpp>

#include <cmath>

#include <px4_ros2/utils/frame_conversion.hpp>

namespace
{
bool isFinite(float value)
{
  return std::isfinite(value);
}

bool anyAxisActive(const std::array<bool, 3> & active)
{
  return active[0] || active[1] || active[2];
}
}  // namespace

bool isSupportedAmPositionOffboardMode(const px4_msgs::msg::OffboardControlMode & mode)
{
  if (mode.attitude || mode.body_rate || mode.thrust_and_torque || mode.direct_actuator) {
    return false;
  }

  return mode.position || mode.velocity || mode.acceleration;
}

AmPositionOffboardReference buildAmPositionOffboardReference(
  const px4_msgs::msg::OffboardControlMode & mode,
  const px4_msgs::msg::TrajectorySetpoint & setpoint,
  const Eigen::Vector3f & fallback_pos_w,
  float fallback_yaw_w,
  const Eigen::Quaternionf & root_quat_w)
{
  AmPositionOffboardReference reference;
  reference.desired_pos_w = fallback_pos_w;
  reference.desired_quat_w = Eigen::Quaternionf(
    Eigen::AngleAxisf(fallback_yaw_w, Eigen::Vector3f::UnitZ()));

  if (!isSupportedAmPositionOffboardMode(mode)) {
    return reference;
  }

  const Eigen::Vector3f pos_w = px4_ros2::positionNedToEnu(
    Eigen::Vector3f(
      setpoint.position[0], setpoint.position[1], setpoint.position[2]));
  const Eigen::Vector3f vel_w = px4_ros2::positionNedToEnu(
    Eigen::Vector3f(
      setpoint.velocity[0], setpoint.velocity[1], setpoint.velocity[2]));
  const Eigen::Vector3f vel_b = root_quat_w.inverse() * vel_w;

  reference.position_active = {{
    mode.position && isFinite(setpoint.position[0]),
    mode.position && isFinite(setpoint.position[1]),
    mode.position && isFinite(setpoint.position[2]),
  }};
  reference.velocity_active = {{
    mode.velocity && isFinite(setpoint.velocity[0]),
    mode.velocity && isFinite(setpoint.velocity[1]),
    mode.velocity && isFinite(setpoint.velocity[2]),
  }};

  if (reference.position_active[0]) {
    reference.desired_pos_w.x() = pos_w.x();
  }
  if (reference.position_active[1]) {
    reference.desired_pos_w.y() = pos_w.y();
  }
  if (reference.position_active[2]) {
    reference.desired_pos_w.z() = pos_w.z();
  }

  if (reference.velocity_active[0]) {
    reference.desired_lin_vel_b.x() = vel_b.x();
  }
  if (reference.velocity_active[1]) {
    reference.desired_lin_vel_b.y() = vel_b.y();
  }
  if (reference.velocity_active[2]) {
    reference.desired_lin_vel_b.z() = vel_b.z();
  }

  if (isFinite(setpoint.yaw)) {
    reference.desired_quat_w = Eigen::Quaternionf(
      Eigen::AngleAxisf(px4_ros2::yawNedToEnu(setpoint.yaw), Eigen::Vector3f::UnitZ()));
  }
  if (isFinite(setpoint.yawspeed)) {
    reference.desired_ang_vel_b.z() = px4_ros2::yawRateNedToEnu(setpoint.yawspeed);
    reference.has_yaw_rate_cmd = true;
  }

  reference.has_position_cmd = anyAxisActive(reference.position_active);
  reference.has_velocity_cmd = anyAxisActive(reference.velocity_active);
  reference.valid =
    reference.has_position_cmd || reference.has_velocity_cmd || reference.has_yaw_rate_cmd;
  return reference;
}
