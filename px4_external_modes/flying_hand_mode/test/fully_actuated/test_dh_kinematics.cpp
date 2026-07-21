#include <flying_hand_mode/fully_actuated/dh_kinematics.hpp>

#include <gtest/gtest.h>

namespace flying_hand_mode::fully_actuated
{
namespace
{

TEST(DhKinematics, UsesPaperStandardDhChain)
{
  DhKinematicsConfig config;
  config.d_m << 0.0, 0.050, 0.0, 0.076;
  config.a_m << 0.363, 0.441, 0.007, 0.200;
  config.alpha_rad << 0.10, -0.10, -1.578, 0.0;
  const DhKinematics kinematics(config);
  const Eigen::Isometry3d pose = kinematics.endEffectorPoseFlu(JointVector::Zero());

  EXPECT_TRUE(pose.matrix().allFinite());
  EXPECT_NEAR(pose.linear().determinant(), 1.0, 1.0e-10);
  EXPECT_GT(pose.translation().norm(), 0.5);
}

TEST(DhKinematics, AppliesConfiguredBodyToArmBaseTransform)
{
  DhKinematicsConfig config;
  config.body_from_arm_base_flu.translation() = Eigen::Vector3d(0.1, -0.2, 0.3);
  const DhKinematics kinematics(config);
  const Eigen::Isometry3d pose = kinematics.endEffectorPoseFlu(JointVector::Zero());
  EXPECT_TRUE(pose.translation().isApprox(Eigen::Vector3d(0.1, -0.2, 0.3)));
}

}  // namespace
}  // namespace flying_hand_mode::fully_actuated
