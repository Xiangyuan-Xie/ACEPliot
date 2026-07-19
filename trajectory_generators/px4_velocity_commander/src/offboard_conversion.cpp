#include <px4_velocity_commander/offboard_conversion.hpp>

#include <cmath>

namespace
{
void fillNanVector(std::array<float, 3> & values)
{
  values[0] = values[1] = values[2] = NAN;
}
}  // namespace

px4_msgs::msg::OffboardControlMode makeVelocityOffboardControlMode(uint64_t timestamp)
{
  px4_msgs::msg::OffboardControlMode mode{};
  mode.timestamp = timestamp;
  mode.position = false;
  mode.velocity = true;
  mode.acceleration = false;
  mode.attitude = false;
  mode.body_rate = false;
  mode.thrust_and_torque = false;
  mode.direct_actuator = false;
  return mode;
}

px4_msgs::msg::TrajectorySetpoint makeVelocityTrajectorySetpoint(
  const VelocityCommand & command,
  uint64_t timestamp)
{
  px4_msgs::msg::TrajectorySetpoint setpoint{};
  setpoint.timestamp = timestamp;
  fillNanVector(setpoint.position);
  fillNanVector(setpoint.acceleration);
  fillNanVector(setpoint.jerk);

  setpoint.velocity[0] = static_cast<float>(command.velocity_enu_m_s[1]);
  setpoint.velocity[1] = static_cast<float>(command.velocity_enu_m_s[0]);
  setpoint.velocity[2] = static_cast<float>(-command.velocity_enu_m_s[2]);
  setpoint.yaw = NAN;
  setpoint.yawspeed = static_cast<float>(-command.yaw_rate_enu_rad_s);
  return setpoint;
}
