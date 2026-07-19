#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <memory>
#include <string>

#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>
#include <px4_ros2/common/setpoint_base.hpp>
#include <rclcpp/rclcpp.hpp>

namespace flying_hand_control_common
{

class WrenchSetpointType : public px4_ros2::SetpointBase
{
public:
  explicit WrenchSetpointType(px4_ros2::Context & context);

  Configuration getConfiguration() override;
  float desiredUpdateRateHz() override;

  bool update(
    const Eigen::Vector3f & normalized_thrust_frd,
    const Eigen::Vector3f & normalized_torque_frd,
    std::uint64_t sample_timestamp_us);

  const std::string & thrustTopic() const noexcept;
  const std::string & torqueTopic() const noexcept;

private:
  std::string thrust_topic_;
  std::string torque_topic_;
  rclcpp::Publisher<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr thrust_publisher_;
  rclcpp::Publisher<px4_msgs::msg::VehicleTorqueSetpoint>::SharedPtr torque_publisher_;
};

}  // namespace flying_hand_control_common
