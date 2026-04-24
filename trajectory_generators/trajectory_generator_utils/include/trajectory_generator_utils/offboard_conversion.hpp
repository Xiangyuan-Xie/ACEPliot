#pragma once

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

#include <trajectory_generator_utils/generator.hpp>

px4_msgs::msg::OffboardControlMode makeOffboardControlMode(
  const TrajectorySample & sample,
  uint64_t timestamp = 0);

px4_msgs::msg::TrajectorySetpoint makeTrajectorySetpoint(
  const TrajectorySample & sample,
  uint64_t timestamp = 0);
