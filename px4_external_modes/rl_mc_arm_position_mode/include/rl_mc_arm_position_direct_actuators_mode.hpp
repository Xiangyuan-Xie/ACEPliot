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
#include <memory>
#include <string>
#include <vector>
#include <arm_robot_data.hpp>
#include <observation_buffer.hpp>
#include <px4_ros2/control/setpoint_types/direct_actuators.hpp>
#include <rl_mode.hpp>

/// @brief Default mode name for the direct-actuator arm-position variant.
static constexpr char kArmPositionDirectActuatorsModeName[] = "RL Arm Position Direct Actuators";
/// @brief Default activation behavior while disarmed.
static constexpr bool kArmPositionActivateEvenWhileDisarmed = true;

/**
 * @class RlMCArmPositionDirectActuatorsMode
 * @brief Reinforcement-learning multirotor arm-positioning mode with direct motor outputs.
 *
 * This mode builds observations, runs policy inference, and forwards normalized
 * policy actions to PX4 direct actuator channels.
 */
class RlMCArmPositionDirectActuatorsMode : public RLModeBase
{
public:
  /**
   * @brief Constructs the direct-actuator arm-positioning mode instance.
   * @param node ROS2 node handle.
   * @param mode_name PX4 external mode display name.
   * @param activate_disarmed Whether this mode can activate while disarmed.
   * @param topic_namespace_prefix PX4 topic namespace prefix.
   * @param root_dir Mode root directory for resolving resources.
   */
  explicit RlMCArmPositionDirectActuatorsMode(
    rclcpp::Node & node,
    const std::string & mode_name = kArmPositionDirectActuatorsModeName,
    bool activate_disarmed = kArmPositionActivateEvenWhileDisarmed,
    const std::string & topic_namespace_prefix = "",
    const std::string & root_dir = ROOT_DIR);

  /// @brief Default destructor.
  ~RlMCArmPositionDirectActuatorsMode() override = default;

  /// @brief Called when the mode is activated.
  void onActivate() override;
  /// @brief Called when the mode is deactivated.
  void onDeactivate() override;
  /// @brief Builds policy-network observations.
  void getObservation(TensorMap & inputs, float dt_s) override;
  /// @brief Applies policy actions as direct motor commands.
  void applyAction(const TensorMap & action, float dt_s) override;

protected:
  /// @brief Updates task targets before observation construction.
  virtual void updateTargets(float dt_s);
  /// @brief Logs target values for debugging and replay analysis.
  virtual void logTargetValues(float dt_s);
  /// @brief Returns the action observation history buffer.
  ObservationBuffer & getActionObsBuffer();

private:
  /**
   * @brief Returns writable robot data with arm extensions.
   * @return Arm data pointer, or nullptr when type conversion fails.
   */
  ArmRobotData * armRobotData();
  /**
   * @brief Returns read-only robot data with arm extensions.
   * @return Arm data pointer, or nullptr when type conversion fails.
   */
  const ArmRobotData * armRobotData() const;

  /// @brief Direct motor output interface.
  std::shared_ptr<px4_ros2::DirectActuatorsSetpointType> direct_actuators_;
  /// @brief Action history buffer used for time-context observations.
  ObservationBuffer action_obs_buffer_;
};
