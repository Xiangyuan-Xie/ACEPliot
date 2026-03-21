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

#include <vector>
#include <robot_data.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

/**
 * @class ArmRobotData
 * @brief Robot data extension for arm-positioning mode.
 *
 * This class extends RobotData with arm command/state channels
 * used during observation construction.
 */
class ArmRobotData : public RobotData
{
public:
  /**
   * @brief Constructs the arm-extended robot data provider.
   * @param mode_base Current mode instance.
   */
  explicit ArmRobotData(px4_ros2::ModeBase & mode_base);

  /**
   * @brief Returns latest arm joint positions.
   * @return Arm joint position vector.
   */
  const std::vector<float> & ArmPosition() const;

  /**
   * @brief Returns latest arm joint commands.
   * @return Arm joint command vector.
   */
  const std::vector<float> & ArmCommand() const;

  /**
   * @brief Returns latest arm joint velocities.
   * @return Arm joint velocity vector.
   */
  const std::vector<float> & ArmVelocity() const;

private:
  /**
   * @brief Callback for arm command topic.
   * @param msg JointState message.
   */
  void armCommandCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

  /**
   * @brief Callback for arm state topic.
   * @param msg JointState message.
   */
  void armStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
    arm_command_sub_;  ///< Arm command subscription.
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
    arm_state_sub_;  ///< Arm state subscription.
  std::vector<float> arm_position_;  ///< Arm joint position cache.
  std::vector<float> arm_command_;   ///< Arm joint command cache.
  std::vector<float> arm_velocity_;  ///< Arm joint velocity cache.
};
