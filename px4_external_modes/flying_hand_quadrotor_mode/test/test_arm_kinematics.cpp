#include <gtest/gtest.h>

#include <flying_hand_quadrotor_mode/arm_kinematics.hpp>

namespace flying_hand_quadrotor_mode
{
namespace
{

TEST(ArmKinematicsTest, MatchesAuditedHomePose)
{
  ArmKinematics kinematics;
  JointVector home;
  home << -1.5708, 3.1416, 0.0, 0.0;

  const Eigen::Isometry3d pose = kinematics.endEffectorPoseFlu(home);

  EXPECT_NEAR(pose.translation().x(), 0.153390235794, 1.0e-9);
  EXPECT_NEAR(pose.translation().y(), 0.000701428823, 1.0e-9);
  EXPECT_NEAR(pose.translation().z(), -0.224250144130, 1.0e-9);
  EXPECT_TRUE(pose.matrix().allFinite());
}

TEST(ArmKinematicsTest, AnalyticJacobianMatchesFiniteDifference)
{
  ArmKinematics kinematics;
  JointVector position;
  position << -1.0, 2.2, 0.4, -0.7;
  const Eigen::Matrix<double, 6, kArmJointCount> jacobian =
    kinematics.endEffectorJacobianFlu(position);
  constexpr double kStep = 1.0e-7;

  for (int joint = 0; joint < kArmJointCount; ++joint) {
    JointVector plus = position;
    JointVector minus = position;
    plus[joint] += kStep;
    minus[joint] -= kStep;
    const Eigen::Isometry3d pose = kinematics.endEffectorPoseFlu(position);
    const Eigen::Isometry3d pose_plus = kinematics.endEffectorPoseFlu(plus);
    const Eigen::Isometry3d pose_minus = kinematics.endEffectorPoseFlu(minus);
    const Eigen::Vector3d finite_position =
      (pose_plus.translation() - pose_minus.translation()) / (2.0 * kStep);
    const Eigen::Matrix3d rotation_delta =
      pose_minus.linear().transpose() * pose_plus.linear();
    const Eigen::AngleAxisd angle_axis(rotation_delta);
    const Eigen::Vector3d finite_angular_world =
      pose.linear() * angle_axis.axis() * angle_axis.angle() / (2.0 * kStep);

    EXPECT_TRUE((jacobian.block<3, 1>(0, joint).isApprox(finite_position, 1.0e-7)));
    EXPECT_TRUE((jacobian.block<3, 1>(3, joint).isApprox(finite_angular_world, 1.0e-7)));
  }
}

TEST(ArmKinematicsTest, MatchesPinocchioMassPropertiesAcrossArmMotion)
{
  ArmKinematics kinematics;
  JointVector home;
  home << -1.5708, 3.1415, 0.0, 0.0;
  const VehicleMassProperties home_properties = kinematics.massPropertiesFlu(home);

  EXPECT_NEAR(home_properties.mass_kg, 2.35584, 1.0e-12);
  const Eigen::Vector3d expected_home_com(-0.01394428, 0.00035525, -0.09972404);
  for (int axis = 0; axis < 3; ++axis) {
    EXPECT_NEAR(
      home_properties.center_of_mass_flu_m[axis], expected_home_com[axis], 1.0e-8);
  }
  const Eigen::Matrix3d expected_home_inertia = (
    Eigen::Matrix3d() <<
      0.0288593790, 0.0000672021461, 0.00205091665,
      0.0000672021461, 0.0416254445, 0.00000854745664,
      0.00205091665, 0.00000854745664, 0.0388321837).finished();
  EXPECT_TRUE(home_properties.inertia_com_flu_kg_m2.isApprox(expected_home_inertia, 1.0e-9));

  JointVector displaced;
  displaced << 1.3656, 0.3908, -0.2196, 2.1205;
  const VehicleMassProperties displaced_properties =
    kinematics.massPropertiesFlu(displaced);
  const Eigen::Vector3d expected_displaced_com(0.08094972, 0.00043896, -0.08675997);
  for (int axis = 0; axis < 3; ++axis) {
    EXPECT_NEAR(
      displaced_properties.center_of_mass_flu_m[axis], expected_displaced_com[axis],
      1.0e-8);
  }
  const Eigen::Matrix3d expected_displaced_inertia = (
    Eigen::Matrix3d() <<
      0.0229164075, -0.000185899747, 0.0156941165,
      -0.000185899747, 0.117652117, 0.0000972928855,
      0.0156941165, 0.0000972928855, 0.120471007).finished();
  EXPECT_TRUE(
    displaced_properties.inertia_com_flu_kg_m2.isApprox(expected_displaced_inertia, 1.0e-8));
}

TEST(ArmKinematicsTest, CenterOfMassJacobianMatchesFiniteDifference)
{
  ArmKinematics kinematics;
  JointVector position;
  position << -1.0, 2.2, 0.4, -0.7;
  const VehicleMassProperties properties = kinematics.massPropertiesFlu(position);
  constexpr double kStep = 1.0e-7;

  for (int joint = 0; joint < kArmJointCount; ++joint) {
    JointVector plus = position;
    JointVector minus = position;
    plus[joint] += kStep;
    minus[joint] -= kStep;
    const Eigen::Vector3d finite_difference =
      (kinematics.massPropertiesFlu(plus).center_of_mass_flu_m -
      kinematics.massPropertiesFlu(minus).center_of_mass_flu_m) / (2.0 * kStep);
    for (int axis = 0; axis < 3; ++axis) {
      EXPECT_NEAR(
        properties.com_jacobian_flu_m_rad(axis, joint), finite_difference[axis], 1.0e-8);
    }
  }
}

}  // namespace
}  // namespace flying_hand_quadrotor_mode
