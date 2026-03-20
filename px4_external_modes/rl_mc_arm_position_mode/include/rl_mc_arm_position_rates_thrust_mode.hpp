#pragma once

#include <px4_ros2/control/setpoint_types/experimental/rates.hpp>
#include <rl_mc_arm_position_direct_actuators_mode.hpp>
#include <memory>
#include <string>

/// @brief Default mode name for the rates-plus-thrust arm-position variant.
static constexpr char kArmPositionRatesThrustModeName[] = "RL Arm Position Rates Thrust";

/**
 * @class RlMCArmPositionRatesThrustMode
 * @brief Arm-position variant that outputs body rates and collective thrust.
 *
 * This class reuses observation and target logic from
 * RlMCArmPositionDirectActuatorsMode and only overrides action application.
 */
class RlMCArmPositionRatesThrustMode : public RlMCArmPositionDirectActuatorsMode
{
public:
  /**
   * @brief Construct a rates-plus-thrust arm-position mode instance.
   * @param node ROS2 node handle.
   * @param mode_name PX4 external mode display name.
   * @param activate_disarmed Whether this mode can activate while disarmed.
   * @param topic_namespace_prefix PX4 topic namespace prefix.
   * @param root_dir Mode root directory for resolving resources.
   */
  explicit RlMCArmPositionRatesThrustMode(
    rclcpp::Node & node,
    const std::string & mode_name = kArmPositionRatesThrustModeName,
    bool activate_disarmed = kArmPositionActivateEvenWhileDisarmed,
    const std::string & topic_namespace_prefix = "",
    const std::string & root_dir = ROOT_DIR);

  /// @brief Default destructor.
  ~RlMCArmPositionRatesThrustMode() override = default;

  /// @brief Applies policy outputs as body-rate and collective-thrust commands.
  void applyAction(const TensorMap & action, float dt_s) override;

private:
  /// @brief Rates+thrust output interface.
  std::shared_ptr<px4_ros2::RatesSetpointType> rates_setpoint_;
  /// @brief Max absolute body-rate command used to scale normalized outputs.
  float max_body_rate_rad_s_{6.0f};
  /// @brief Max collective thrust used to scale normalized outputs.
  float max_collective_thrust_{1.0f};
};
