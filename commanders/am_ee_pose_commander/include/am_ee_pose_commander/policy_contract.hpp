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
#include <vector>

namespace am_ee_pose_commander
{

constexpr std::size_t kPreviewPoseCount = 5;
constexpr std::size_t kArmJointCount = 4;
constexpr std::size_t kUpperObservationDim = 63;
constexpr std::size_t kUpperActionDim = 8;
constexpr float kUpperControlPeriodS = 0.02F;
constexpr float kVelocityDeadband = 0.05F;
constexpr float kArmDeltaScaleRad = 0.02F;

using ArmVector = Eigen::Matrix<float, static_cast<int>(kArmJointCount), 1>;
using UpperActionVector = Eigen::Matrix<float, static_cast<int>(kUpperActionDim), 1>;

struct Pose
{
  Eigen::Vector3f position_w{Eigen::Vector3f::Zero()};
  Eigen::Matrix3f rotation_w{Eigen::Matrix3f::Identity()};

  bool allFinite() const noexcept;
};

using TargetPreview = std::array<Pose, kPreviewPoseCount>;

TargetPreview makeConstantTargetPreview(const Pose & target);

struct ProcessedUpperAction
{
  UpperActionVector raw_action{UpperActionVector::Zero()};
  Eigen::Vector4f heading_velocity_command{Eigen::Vector4f::Zero()};
  ArmVector arm_position_target{ArmVector::Zero()};
  ArmVector arm_velocity_target{ArmVector::Zero()};
};

struct WorldVelocityCommand
{
  Eigen::Vector3f linear_velocity_enu_m_s{Eigen::Vector3f::Zero()};
  float yaw_rate_enu_rad_s{0.0F};
};

class PolicyContract
{
public:
  static std::vector<float> buildObservation(
    const TargetPreview & target_preview,
    const Pose & current_ee_pose,
    const ArmVector & servo_position,
    const Eigen::Vector3f & root_linear_velocity_b,
    const Eigen::Vector3f & root_angular_velocity_b,
    const UpperActionVector & last_action);

  static ProcessedUpperAction processAction(
    const std::vector<float> & model_action,
    const ArmVector & previous_arm_target,
    const ArmVector & joint_lower_limit,
    const ArmVector & joint_upper_limit);
};

WorldVelocityCommand headingVelocityToWorld(
  const Eigen::Vector4f & heading_velocity_command,
  const Eigen::Quaternionf & root_attitude_enu);

}  // namespace am_ee_pose_commander
