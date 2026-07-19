#pragma once

#include <flying_hand_control_common/controller_types.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace flying_hand_fully_actuated_mode
{

using flying_hand_control_common::JointVector;

struct DhKinematicsConfig
{
  JointVector d_m{JointVector::Zero()};
  JointVector a_m{JointVector::Zero()};
  JointVector alpha_rad{JointVector::Zero()};
  JointVector theta_offset_rad{JointVector::Zero()};
  Eigen::Isometry3d body_from_arm_base_flu{Eigen::Isometry3d::Identity()};

  bool allFinite() const noexcept;
};

class DhKinematics
{
public:
  explicit DhKinematics(DhKinematicsConfig config);

  Eigen::Isometry3d endEffectorPoseFlu(const JointVector & joint_position_rad) const noexcept;
  const DhKinematicsConfig & config() const noexcept;

private:
  DhKinematicsConfig config_;
};

}  // namespace flying_hand_fully_actuated_mode
