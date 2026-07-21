#include <flying_hand_mode/fully_actuated/dh_kinematics.hpp>

#include <cmath>
#include <stdexcept>
#include <utility>

namespace flying_hand_mode::fully_actuated
{

bool DhKinematicsConfig::allFinite() const noexcept
{
  return d_m.allFinite() && a_m.allFinite() && alpha_rad.allFinite() &&
         theta_offset_rad.allFinite() && body_from_arm_base_flu.matrix().allFinite();
}

DhKinematics::DhKinematics(DhKinematicsConfig config)
: config_(std::move(config))
{
  if (!config_.allFinite()) {
    throw std::invalid_argument("Invalid Flying Hand DH kinematics configuration");
  }
}

Eigen::Isometry3d DhKinematics::endEffectorPoseFlu(
  const JointVector & joint_position_rad) const noexcept
{
  if (!joint_position_rad.allFinite()) {
    return Eigen::Isometry3d::Identity();
  }
  Eigen::Isometry3d transform = config_.body_from_arm_base_flu;
  for (int joint = 0; joint < flying_hand_mode::runtime::kArmJointCount; ++joint) {
    const double theta = joint_position_rad[joint] + config_.theta_offset_rad[joint];
    const double alpha = config_.alpha_rad[joint];
    Eigen::Matrix4d dh = Eigen::Matrix4d::Identity();
    dh(0, 0) = std::cos(theta);
    dh(0, 1) = -std::sin(theta) * std::cos(alpha);
    dh(0, 2) = std::sin(theta) * std::sin(alpha);
    dh(0, 3) = config_.a_m[joint] * std::cos(theta);
    dh(1, 0) = std::sin(theta);
    dh(1, 1) = std::cos(theta) * std::cos(alpha);
    dh(1, 2) = -std::cos(theta) * std::sin(alpha);
    dh(1, 3) = config_.a_m[joint] * std::sin(theta);
    dh(2, 1) = std::sin(alpha);
    dh(2, 2) = std::cos(alpha);
    dh(2, 3) = config_.d_m[joint];
    transform.matrix() *= dh;
  }
  return transform;
}

const DhKinematicsConfig & DhKinematics::config() const noexcept
{
  return config_;
}

}  // namespace flying_hand_mode::fully_actuated
