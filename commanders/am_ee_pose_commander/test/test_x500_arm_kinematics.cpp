#include <am_ee_pose_commander/x500_arm_kinematics.hpp>

#include <gtest/gtest.h>

namespace am_ee_pose_commander
{
namespace
{

TEST(X500ArmKinematics, ConvertsAcetelePublicGripperPosition)
{
  EXPECT_FLOAT_EQ(X500ArmKinematics::gripperPublicToJoint5(1.0F), 0.0F);
  EXPECT_FLOAT_EQ(X500ArmKinematics::gripperPublicToJoint5(0.0F), -1.723F);
  EXPECT_FLOAT_EQ(X500ArmKinematics::gripperPublicToJoint5(2.0F), 0.0F);
}

TEST(X500ArmKinematics, Joint5ChangesOrientationWithoutMovingLink5Origin)
{
  X500ArmKinematics kinematics;
  const ArmVector arm = ArmVector::Zero();
  const Eigen::Isometry3f open_pose = kinematics.link5PoseFlu(arm, 1.0F);
  const Eigen::Isometry3f closed_pose = kinematics.link5PoseFlu(arm, 0.0F);

  EXPECT_TRUE(open_pose.matrix().allFinite());
  EXPECT_TRUE(closed_pose.matrix().allFinite());
  EXPECT_TRUE(open_pose.translation().isApprox(closed_pose.translation(), 1.0e-6F));
  EXPECT_FALSE(open_pose.linear().isApprox(closed_pose.linear(), 1.0e-3F));
}

}  // namespace
}  // namespace am_ee_pose_commander
