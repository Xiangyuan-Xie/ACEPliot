#include <flying_hand_control_common/wrench_setpoint_type.hpp>

#include <px4_ros2/utils/message_version.hpp>

#include <exception>

namespace flying_hand_control_common
{

WrenchSetpointType::WrenchSetpointType(px4_ros2::Context & context)
: SetpointBase(context)
{
  thrust_topic_ = context.topicNamespacePrefix() + "fmu/in/vehicle_thrust_setpoint" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleThrustSetpoint>();
  torque_topic_ = context.topicNamespacePrefix() + "fmu/in/vehicle_torque_setpoint" +
    px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleTorqueSetpoint>();
  thrust_publisher_ = context.node().create_publisher<px4_msgs::msg::VehicleThrustSetpoint>(
    thrust_topic_, 1);
  torque_publisher_ = context.node().create_publisher<px4_msgs::msg::VehicleTorqueSetpoint>(
    torque_topic_, 1);
  thrust_topic_ = thrust_publisher_->get_topic_name();
  torque_topic_ = torque_publisher_->get_topic_name();
}

px4_ros2::SetpointBase::Configuration WrenchSetpointType::getConfiguration()
{
  Configuration config{};
  config.control_allocation_enabled = true;
  config.rates_enabled = false;
  config.attitude_enabled = false;
  config.altitude_enabled = false;
  config.climb_rate_enabled = false;
  config.acceleration_enabled = false;
  config.velocity_enabled = false;
  config.position_enabled = false;
  return config;
}

float WrenchSetpointType::desiredUpdateRateHz()
{
  return 200.0F;
}

bool WrenchSetpointType::update(
  const Eigen::Vector3f & normalized_thrust_frd,
  const Eigen::Vector3f & normalized_torque_frd,
  std::uint64_t sample_timestamp_us)
{
  if (!normalized_thrust_frd.allFinite() || !normalized_torque_frd.allFinite() ||
    (normalized_thrust_frd.array().abs() > 1.0F).any() ||
    (normalized_torque_frd.array().abs() > 1.0F).any())
  {
    return false;
  }

  try {
    onUpdate();

    px4_msgs::msg::VehicleThrustSetpoint thrust{};
    thrust.xyz[0] = normalized_thrust_frd.x();
    thrust.xyz[1] = normalized_thrust_frd.y();
    thrust.xyz[2] = normalized_thrust_frd.z();
    thrust.timestamp = 0;
    thrust.timestamp_sample = sample_timestamp_us;
    thrust_publisher_->publish(thrust);

    // PX4's allocator callback is driven by torque and reads the latest thrust.
    px4_msgs::msg::VehicleTorqueSetpoint torque{};
    torque.xyz[0] = normalized_torque_frd.x();
    torque.xyz[1] = normalized_torque_frd.y();
    torque.xyz[2] = normalized_torque_frd.z();
    torque.timestamp = 0;
    torque.timestamp_sample = sample_timestamp_us;
    torque_publisher_->publish(torque);
  } catch (const std::exception &) {
    return false;
  }
  return true;
}

const std::string & WrenchSetpointType::thrustTopic() const noexcept
{
  return thrust_topic_;
}

const std::string & WrenchSetpointType::torqueTopic() const noexcept
{
  return torque_topic_;
}

}  // namespace flying_hand_control_common
