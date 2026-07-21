/****************************************************************************
 * Copyright (c) 2026 Xiangyuan Xie <dragonboat_xxy@163.com>.
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <am_ee_pose_commander/x500_arm_kinematics.hpp>

#include <algorithm>
#include <array>

namespace am_ee_pose_commander
{
namespace
{

struct JointOrigin
{
  Eigen::Vector3f xyz_m;
  Eigen::Vector3f rpy_rad;
};

const std::array<JointOrigin, kArmJointCount> kJointOrigins{{
  {{0.0F, 0.0002F, -0.1503F}, {-1.5708F, 0.0F, 3.1416F}},
  {{-0.04123F, 0.2314F, -0.0005F}, {0.0F, 0.0F, 3.1416F}},
  {{-0.04098F, -0.2686F, 0.0F}, {0.0F, 0.0F, 0.0F}},
  {{0.0F, -0.05189F, 0.0F}, {1.5708F, 1.5708F, 0.0F}},
}};

const Eigen::Vector3f kJoint5OriginM{0.0F, 0.0F, 0.037F};
constexpr float kJoint5ClosedRad = -1.723F;

Eigen::Matrix3f rotationFromRpy(const Eigen::Vector3f & rpy) noexcept
{
  return (
    Eigen::AngleAxisf(rpy.z(), Eigen::Vector3f::UnitZ()) *
    Eigen::AngleAxisf(rpy.y(), Eigen::Vector3f::UnitY()) *
    Eigen::AngleAxisf(rpy.x(), Eigen::Vector3f::UnitX())).toRotationMatrix();
}

Eigen::Isometry3f originTransform(const JointOrigin & origin) noexcept
{
  Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();
  transform.linear() = rotationFromRpy(origin.rpy_rad);
  transform.translation() = origin.xyz_m;
  return transform;
}

Eigen::Isometry3f jointRotation(float position_rad) noexcept
{
  Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();
  transform.linear() =
    Eigen::AngleAxisf(position_rad, Eigen::Vector3f::UnitZ()).toRotationMatrix();
  return transform;
}

}  // namespace

float X500ArmKinematics::gripperPublicToJoint5(float public_position) noexcept
{
  return kJoint5ClosedRad * (1.0F - std::clamp(public_position, 0.0F, 1.0F));
}

Eigen::Isometry3f X500ArmKinematics::link5PoseFlu(
  const ArmVector & arm_position_rad,
  float gripper_public_position) const noexcept
{
  Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();
  for (int joint = 0; joint < static_cast<int>(kArmJointCount); ++joint) {
    transform = transform * originTransform(kJointOrigins[static_cast<std::size_t>(joint)]) *
      jointRotation(arm_position_rad[joint]);
  }
  transform = transform * Eigen::Translation3f(kJoint5OriginM) *
    jointRotation(gripperPublicToJoint5(gripper_public_position));
  return transform;
}

}  // namespace am_ee_pose_commander
