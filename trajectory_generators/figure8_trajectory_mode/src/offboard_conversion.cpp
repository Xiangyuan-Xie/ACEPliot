#include <figure8_trajectory_mode/offboard_conversion.hpp>

#include <cmath>

#include <px4_ros2/utils/frame_conversion.hpp>

namespace
{
void fillVector(const std::optional<Eigen::Vector3f> & source, std::array<float, 3> & dst)
{
  if (!source.has_value()) {
    dst[0] = dst[1] = dst[2] = NAN;
    return;
  }

  const Eigen::Vector3f ned = px4_ros2::positionEnuToNed(source.value());
  dst[0] = ned.x();
  dst[1] = ned.y();
  dst[2] = ned.z();
}
}  // namespace

px4_msgs::msg::OffboardControlMode makeOffboardControlMode(
  const TrajectorySample & sample,
  uint64_t timestamp)
{
  px4_msgs::msg::OffboardControlMode mode{};
  mode.timestamp = timestamp;
  mode.position = sample.position.has_value();
  mode.velocity = sample.velocity.has_value();
  mode.acceleration = sample.acceleration.has_value();
  mode.attitude = false;
  mode.body_rate = false;
  mode.thrust_and_torque = false;
  mode.direct_actuator = false;
  return mode;
}

px4_msgs::msg::TrajectorySetpoint makeTrajectorySetpoint(
  const TrajectorySample & sample,
  uint64_t timestamp)
{
  px4_msgs::msg::TrajectorySetpoint setpoint{};
  setpoint.timestamp = timestamp;
  fillVector(sample.position, setpoint.position);
  fillVector(sample.velocity, setpoint.velocity);
  fillVector(sample.acceleration, setpoint.acceleration);
  setpoint.jerk[0] = setpoint.jerk[1] = setpoint.jerk[2] = NAN;
  setpoint.yaw = sample.yaw.has_value() ? px4_ros2::yawEnuToNed(sample.yaw.value()) : NAN;
  setpoint.yawspeed =
    sample.yaw_rate.has_value() ? px4_ros2::yawRateEnuToNed(sample.yaw_rate.value()) : NAN;
  return setpoint;
}
