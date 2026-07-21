/****************************************************************************
 * Copyright (c) 2026 Xiangyuan Xie <dragonboat_xxy@163.com>.
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <am_ee_pose_commander/offboard_conversion.hpp>

#include <px4_ros2/utils/frame_conversion.hpp>

#include <cmath>

namespace am_ee_pose_commander
{
namespace
{

void fillNan(std::array<float, 3> & values)
{
  values.fill(NAN);
}

}  // namespace

px4_msgs::msg::OffboardControlMode makeVelocityOffboardControlMode(uint64_t timestamp_us)
{
  px4_msgs::msg::OffboardControlMode message{};
  message.timestamp = timestamp_us;
  message.velocity = true;
  return message;
}

px4_msgs::msg::TrajectorySetpoint makeVelocityTrajectorySetpoint(
  const WorldVelocityCommand & command,
  uint64_t timestamp_us)
{
  px4_msgs::msg::TrajectorySetpoint message{};
  message.timestamp = timestamp_us;
  fillNan(message.position);
  fillNan(message.acceleration);
  fillNan(message.jerk);

  const Eigen::Vector3f velocity_ned =
    px4_ros2::positionEnuToNed(command.linear_velocity_enu_m_s);
  message.velocity[0] = velocity_ned.x();
  message.velocity[1] = velocity_ned.y();
  message.velocity[2] = velocity_ned.z();
  message.yaw = NAN;
  message.yawspeed = px4_ros2::yawRateEnuToNed(command.yaw_rate_enu_rad_s);
  return message;
}

}  // namespace am_ee_pose_commander
