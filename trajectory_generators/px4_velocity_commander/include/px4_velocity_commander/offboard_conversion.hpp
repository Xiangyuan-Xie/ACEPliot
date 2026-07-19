#pragma once

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

#include <px4_velocity_commander/velocity_profile.hpp>

px4_msgs::msg::OffboardControlMode makeVelocityOffboardControlMode(uint64_t timestamp = 0);

px4_msgs::msg::TrajectorySetpoint makeVelocityTrajectorySetpoint(
  const VelocityCommand & command,
  uint64_t timestamp = 0);
