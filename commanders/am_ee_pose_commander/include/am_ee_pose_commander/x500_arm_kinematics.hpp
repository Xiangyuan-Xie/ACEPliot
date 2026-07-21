/****************************************************************************
 * Copyright (c) 2026 Xiangyuan Xie <dragonboat_xxy@163.com>.
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#pragma once

#include <am_ee_pose_commander/policy_contract.hpp>

#include <Eigen/Geometry>

namespace am_ee_pose_commander
{

class X500ArmKinematics
{
public:
  static float gripperPublicToJoint5(float public_position) noexcept;

  Eigen::Isometry3f link5PoseFlu(
    const ArmVector & arm_position_rad,
    float gripper_public_position) const noexcept;
};

}  // namespace am_ee_pose_commander
