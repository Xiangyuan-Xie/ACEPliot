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
#include <Eigen/Geometry>
#include <array>
#include <memory>
#include <string>
#include <vector>
#include <am_position_offboard_reference.hpp>
#include <arm_robot_data.hpp>
#include <observation_buffer.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_ros2/control/setpoint_types/direct_actuators.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <rl_mode.hpp>

/// @brief Default mode name for the motor-output arm-position variant.
static constexpr char kAmPositionMotorModeName[] = "AM Position Motor";
/// @brief Default activation behavior while disarmed.
static constexpr bool kAmPositionActivateEvenWhileDisarmed = true;

/**
 * @class AmPositionMotorMode
 * @brief Reinforcement-learning multirotor arm-positioning mode with motor outputs.
 *
 * This mode builds observations, runs policy inference, and forwards normalized
 * policy actions to PX4 direct actuator channels. Base commands are sourced
 * from OffboardControlMode and TrajectorySetpoint messages with hover-lock
 * fallback when external references time out.
 */
class AmPositionMotorMode : public RLModeBase
{
public:
  /**
   * @brief Constructs the motor-output arm-positioning mode instance.
   * @param node ROS2 node handle.
   * @param mode_name PX4 external mode display name.
   * @param activate_disarmed Whether this mode can activate while disarmed.
   * @param topic_namespace_prefix PX4 topic namespace prefix.
   * @param root_dir Mode root directory for resolving resources.
   */
  explicit AmPositionMotorMode(
    rclcpp::Node & node,
    const std::string & mode_name = kAmPositionMotorModeName,
    bool activate_disarmed = kAmPositionActivateEvenWhileDisarmed,
    const std::string & topic_namespace_prefix = "",
    const std::string & root_dir = ROOT_DIR);

  /// @brief Default destructor.
  ~AmPositionMotorMode() override = default;

  /// @brief Called when the mode is activated.
  void onActivate() override;
  /// @brief Called when the mode is deactivated.
  void onDeactivate() override;
  /// @brief Builds policy-network observations.
  void getObservation(TensorMap & inputs, float dt_s) override;
  /// @brief Applies policy actions as motor commands.
  void applyAction(const TensorMap & action, float dt_s) override;

protected:
  /// @brief Updates task targets before observation construction.
  virtual void updateTargets(float dt_s);
  /// @brief Logs target values for debugging and replay analysis.
  virtual void logTargetValues(float dt_s);
  /// @brief Returns the action observation history buffer.
  ObservationBuffer & getActionObsBuffer();

private:
  struct CommandReference
  {
    Eigen::Vector3f desired_lin_vel_b{Eigen::Vector3f::Zero()};
    Eigen::Vector3f desired_ang_vel_b{Eigen::Vector3f::Zero()};
    Eigen::Vector3f desired_pos_w{Eigen::Vector3f::Zero()};
    Eigen::Quaternionf desired_quat_w{Eigen::Quaternionf::Identity()};
    std::array<bool, 3> lin_cmd_active{{false, false, false}};
    std::array<bool, 3> ang_cmd_active{{false, false, false}};
    bool has_lin_vel_cmd{false};
    bool has_ang_vel_cmd{false};
  };

  /// @brief Callback for Offboard control-mode input.
  void offboardControlModeCallback(const px4_msgs::msg::OffboardControlMode::SharedPtr msg);
  /// @brief Callback for trajectory setpoint input.
  void trajectorySetpointCallback(const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg);
  /// @brief Callback for the simulation clock input.
  void simClockCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg);
  /// @brief Returns whether the latest Offboard reference is still valid.
  bool hasFreshExternalReference();
  /// @brief Returns the active time source for mode logic.
  bool getCurrentModeTime(rclcpp::Time & now);
  /// @brief Configures node clock behavior according to `use_sim_time`.
  void configureClockSource(rclcpp::Node & node);
  /// @brief Caches the local receive time for the latest Offboard message.
  bool updateExternalReferenceReceiptTime(rclcpp::Time & receipt_time);

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

  /// @brief Motor output interface.
  std::shared_ptr<px4_ros2::DirectActuatorsSetpointType> motor_setpoint_;
  /// @brief Action history buffer used for time-context observations.
  ObservationBuffer action_obs_buffer_;

  /// @brief Offboard input topics and freshness timeout.
  std::string offboard_control_mode_topic_{"/fmu/in/offboard_control_mode"};
  std::string trajectory_setpoint_topic_{"/fmu/in/trajectory_setpoint"};
  double offboard_setpoint_timeout_s_{0.5};
  /// @brief Offboard reference subscriptions and latest cached samples.
  rclcpp::Subscription<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_sub_;
  rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_sub_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr sim_clock_sub_;
  px4_msgs::msg::OffboardControlMode last_offboard_control_mode_msg_{};
  px4_msgs::msg::TrajectorySetpoint last_trajectory_setpoint_msg_{};
  rclcpp::Time last_offboard_control_mode_time_{0};
  rclcpp::Time last_trajectory_setpoint_time_{0};
  bool has_offboard_control_mode_msg_{false};
  bool has_trajectory_setpoint_msg_{false};
  /// @brief Simulation clock configuration and latest sample cache.
  bool use_sim_time_{false};
  bool has_sim_time_{false};
  std::string sim_clock_topic_{"/acesim/clock"};
  rclcpp::Time latest_sim_time_{0, 0, RCL_ROS_TIME};
  bool warned_waiting_for_sim_time_{false};

  /// @brief Position+yaw hold fallback used when no command source is valid.
  bool hover_lock_active_{false};
  Eigen::Vector3f hover_lock_pos_w_{Eigen::Vector3f::Zero()};
  float hover_lock_yaw_w_{0.0f};
  /// @brief Cached command reference updated each cycle in updateTargets().
  CommandReference current_cmd_ref_{};
};
