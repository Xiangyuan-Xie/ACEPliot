/****************************************************************************
 * Copyright (c) 2026 Xiangyuan Xie <dragonboat_xxy@163.com>.
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <am_ee_pose_commander/policy_contract.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace am_ee_pose_commander
{
namespace
{

constexpr float kQuaternionNormEpsilon = 1.0e-6F;
const Eigen::Vector4f kVelocityScales{1.5F, 1.5F, 1.0F, 1.0F};

void validateFinite(const Eigen::MatrixXf & value, const char * label)
{
  if (!value.allFinite()) {
    throw std::invalid_argument(std::string(label) + " contains non-finite values");
  }
}

float headingFromQuaternion(const Eigen::Quaternionf & attitude) noexcept
{
  const Eigen::Quaternionf normalized = attitude.normalized();
  return std::atan2(
    2.0F * (normalized.w() * normalized.z() + normalized.x() * normalized.y()),
    1.0F - 2.0F *
    (normalized.y() * normalized.y() + normalized.z() * normalized.z()));
}

}  // namespace

bool Pose::allFinite() const noexcept
{
  return position_w.allFinite() && rotation_w.allFinite();
}

TargetPreview makeConstantTargetPreview(const Pose & target)
{
  if (!target.allFinite()) {
    throw std::invalid_argument("constant target EE pose contains non-finite values");
  }
  TargetPreview preview;
  preview.fill(target);
  return preview;
}

std::vector<float> PolicyContract::buildObservation(
  const TargetPreview & target_preview,
  const Pose & current_ee_pose,
  const ArmVector & servo_position,
  const Eigen::Vector3f & root_linear_velocity_b,
  const Eigen::Vector3f & root_angular_velocity_b,
  const UpperActionVector & last_action)
{
  if (!current_ee_pose.allFinite()) {
    throw std::invalid_argument("current EE pose contains non-finite values");
  }
  validateFinite(servo_position, "servo position");
  validateFinite(root_linear_velocity_b, "root linear velocity");
  validateFinite(root_angular_velocity_b, "root angular velocity");
  validateFinite(last_action, "last upper action");

  const Eigen::Matrix3f world_to_ee = current_ee_pose.rotation_w.transpose();
  std::vector<float> observation;
  observation.reserve(kUpperObservationDim);

  for (const Pose & target : target_preview) {
    if (!target.allFinite()) {
      throw std::invalid_argument("target EE preview contains non-finite values");
    }
    const Eigen::Vector3f relative_position_ee =
      world_to_ee * (target.position_w - current_ee_pose.position_w);
    const Eigen::Matrix3f relative_rotation_ee = world_to_ee * target.rotation_w;

    observation.push_back(relative_position_ee.x());
    observation.push_back(relative_position_ee.y());
    observation.push_back(relative_position_ee.z());
    for (int row = 0; row < 2; ++row) {
      for (int column = 0; column < 3; ++column) {
        observation.push_back(relative_rotation_ee(row, column));
      }
    }
  }

  observation.insert(observation.end(), servo_position.data(), servo_position.data() + 4);
  observation.insert(
    observation.end(), root_linear_velocity_b.data(), root_linear_velocity_b.data() + 3);
  observation.insert(
    observation.end(), root_angular_velocity_b.data(), root_angular_velocity_b.data() + 3);
  observation.insert(observation.end(), last_action.data(), last_action.data() + 8);

  if (observation.size() != kUpperObservationDim) {
    throw std::logic_error("AM EE Pose observation dimension is not 63");
  }
  return observation;
}

ProcessedUpperAction PolicyContract::processAction(
  const std::vector<float> & model_action,
  const ArmVector & previous_arm_target,
  const ArmVector & joint_lower_limit,
  const ArmVector & joint_upper_limit)
{
  if (model_action.size() != kUpperActionDim) {
    throw std::invalid_argument("AM EE Pose upper policy must produce 8 actions");
  }
  validateFinite(previous_arm_target, "previous arm target");
  validateFinite(joint_lower_limit, "joint lower limit");
  validateFinite(joint_upper_limit, "joint upper limit");
  if ((joint_lower_limit.array() > joint_upper_limit.array()).any()) {
    throw std::invalid_argument("joint lower limit exceeds upper limit");
  }

  ProcessedUpperAction processed;
  for (std::size_t index = 0; index < kUpperActionDim; ++index) {
    if (!std::isfinite(model_action[index])) {
      throw std::invalid_argument("AM EE Pose upper policy produced a non-finite action");
    }
    processed.raw_action[static_cast<int>(index)] =
      std::clamp(model_action[index], -1.0F, 1.0F);
  }

  processed.heading_velocity_command = processed.raw_action.head<4>();
  for (int axis = 0; axis < 4; ++axis) {
    if (std::abs(processed.heading_velocity_command[axis]) < kVelocityDeadband) {
      processed.heading_velocity_command[axis] = 0.0F;
    }
  }
  processed.heading_velocity_command.array() *= kVelocityScales.array();

  processed.arm_position_target =
    previous_arm_target + kArmDeltaScaleRad * processed.raw_action.tail<4>();
  for (int joint = 0; joint < static_cast<int>(kArmJointCount); ++joint) {
    processed.arm_position_target[joint] = std::clamp(
      processed.arm_position_target[joint], joint_lower_limit[joint], joint_upper_limit[joint]);
  }
  processed.arm_velocity_target =
    (processed.arm_position_target - previous_arm_target) / kUpperControlPeriodS;
  return processed;
}

WorldVelocityCommand headingVelocityToWorld(
  const Eigen::Vector4f & heading_velocity_command,
  const Eigen::Quaternionf & root_attitude_enu)
{
  if (!heading_velocity_command.allFinite() || !root_attitude_enu.coeffs().allFinite() ||
    root_attitude_enu.norm() < kQuaternionNormEpsilon)
  {
    throw std::invalid_argument("cannot convert an invalid heading velocity command");
  }

  const float heading = headingFromQuaternion(root_attitude_enu);
  const Eigen::Quaternionf heading_rotation(
    Eigen::AngleAxisf(heading, Eigen::Vector3f::UnitZ()));
  WorldVelocityCommand command;
  command.linear_velocity_enu_m_s = heading_rotation * heading_velocity_command.head<3>();
  command.yaw_rate_enu_rad_s = heading_velocity_command[3];
  return command;
}

}  // namespace am_ee_pose_commander
