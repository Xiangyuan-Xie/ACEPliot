#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <flying_hand_quadrotor_mode/controller_core.hpp>

namespace flying_hand_quadrotor_mode
{

struct VehicleMassProperties
{
  double mass_kg{0.0};
  Eigen::Vector3d center_of_mass_flu_m{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d inertia_com_flu_kg_m2{Eigen::Matrix3d::Zero()};
  Eigen::Matrix<double, 3, kArmJointCount> com_jacobian_flu_m_rad{
    Eigen::Matrix<double, 3, kArmJointCount>::Zero()};

  bool allFinite() const noexcept;
};

class ArmKinematics
{
public:
  Eigen::Isometry3d endEffectorPoseFlu(const JointVector & joint_position_rad) const noexcept;
  Eigen::Matrix<double, 6, kArmJointCount> endEffectorJacobianFlu(
    const JointVector & joint_position_rad) const noexcept;
  VehicleMassProperties massPropertiesFlu(
    const JointVector & joint_position_rad) const noexcept;
};

}  // namespace flying_hand_quadrotor_mode
