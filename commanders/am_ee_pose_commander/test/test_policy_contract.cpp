#include <am_ee_pose_commander/offboard_conversion.hpp>
#include <am_ee_pose_commander/policy_contract.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace am_ee_pose_commander
{
namespace
{

constexpr float kHalfPi = 1.57079632679F;

TEST(PolicyContract, BuildsConstantPreviewForStartupHold)
{
  Pose target;
  target.position_w = Eigen::Vector3f(1.0F, -2.0F, 3.0F);
  target.rotation_w = Eigen::AngleAxisf(kHalfPi, Eigen::Vector3f::UnitZ()).toRotationMatrix();

  const TargetPreview preview = makeConstantTargetPreview(target);

  for (const Pose & sample : preview) {
    EXPECT_TRUE(sample.position_w.isApprox(target.position_w));
    EXPECT_TRUE(sample.rotation_w.isApprox(target.rotation_w));
  }
}

TEST(PolicyContract, BuildsAcelabObservationInExactOrder)
{
  Pose current;
  current.position_w = Eigen::Vector3f(1.0F, 2.0F, 3.0F);
  current.rotation_w = Eigen::Matrix3f::Identity();

  TargetPreview preview;
  for (std::size_t index = 0; index < kPreviewPoseCount; ++index) {
    preview[index].position_w = current.position_w +
      Eigen::Vector3f(static_cast<float>(index + 1), 0.0F, 0.0F);
    preview[index].rotation_w = Eigen::Matrix3f::Identity();
  }
  preview[0].rotation_w =
    Eigen::AngleAxisf(kHalfPi, Eigen::Vector3f::UnitZ()).toRotationMatrix();

  ArmVector arm;
  arm << 0.1F, 0.2F, 0.3F, 0.4F;
  UpperActionVector last_action;
  last_action << -0.4F, -0.3F, -0.2F, -0.1F, 0.1F, 0.2F, 0.3F, 0.4F;
  const auto observation = PolicyContract::buildObservation(
    preview, current, arm, Eigen::Vector3f(1.0F, 2.0F, 3.0F),
    Eigen::Vector3f(4.0F, 5.0F, 6.0F), last_action);

  ASSERT_EQ(observation.size(), kUpperObservationDim);
  EXPECT_FLOAT_EQ(observation[0], 1.0F);
  EXPECT_FLOAT_EQ(observation[45], 0.1F);
  EXPECT_FLOAT_EQ(observation[48], 0.4F);
  EXPECT_FLOAT_EQ(observation[49], 1.0F);
  EXPECT_FLOAT_EQ(observation[54], 6.0F);
  EXPECT_FLOAT_EQ(observation[55], -0.4F);
  EXPECT_FLOAT_EQ(observation[62], 0.4F);
}

TEST(PolicyContract, AppliesTrainingActionSemanticsAndJointLimits)
{
  const std::vector<float> model_action{
    0.04F, 0.05F, -2.0F, 1.0F,
    1.0F, -1.0F, 0.5F, 1.0F};
  ArmVector previous;
  previous << 2.64F, 0.01F, 0.0F, 3.14F;
  ArmVector lower;
  lower << -2.6485F, 0.0F, -2.6485F, -3.1415F;
  ArmVector upper;
  upper << 2.6485F, 3.1415F, 2.6485F, 3.1415F;

  const ProcessedUpperAction action =
    PolicyContract::processAction(model_action, previous, lower, upper);

  EXPECT_FLOAT_EQ(action.heading_velocity_command[0], 0.0F);
  EXPECT_FLOAT_EQ(action.heading_velocity_command[1], 0.075F);
  EXPECT_FLOAT_EQ(action.heading_velocity_command[2], -1.0F);
  EXPECT_FLOAT_EQ(action.heading_velocity_command[3], 1.0F);
  EXPECT_FLOAT_EQ(action.arm_position_target[0], 2.6485F);
  EXPECT_FLOAT_EQ(action.arm_position_target[1], 0.0F);
  EXPECT_FLOAT_EQ(action.arm_position_target[2], 0.01F);
  EXPECT_FLOAT_EQ(action.arm_position_target[3], 3.1415F);
}

TEST(PolicyContract, ConvertsHeadingVelocityToWorldEnu)
{
  const Eigen::Quaternionf yaw_90(
    Eigen::AngleAxisf(kHalfPi, Eigen::Vector3f::UnitZ()));
  const WorldVelocityCommand command = headingVelocityToWorld(
    Eigen::Vector4f(1.0F, 0.0F, 0.5F, 0.25F), yaw_90);

  EXPECT_NEAR(command.linear_velocity_enu_m_s.x(), 0.0F, 1.0e-6F);
  EXPECT_NEAR(command.linear_velocity_enu_m_s.y(), 1.0F, 1.0e-6F);
  EXPECT_FLOAT_EQ(command.linear_velocity_enu_m_s.z(), 0.5F);
  EXPECT_FLOAT_EQ(command.yaw_rate_enu_rad_s, 0.25F);
}

TEST(OffboardConversion, PublishesVelocityOnlyForPx4AmOffboard)
{
  WorldVelocityCommand command;
  command.linear_velocity_enu_m_s = Eigen::Vector3f(1.0F, 2.0F, 3.0F);
  command.yaw_rate_enu_rad_s = 0.4F;

  const auto mode = makeVelocityOffboardControlMode(42);
  const auto setpoint = makeVelocityTrajectorySetpoint(command, 42);

  EXPECT_TRUE(mode.velocity);
  EXPECT_FALSE(mode.position);
  EXPECT_EQ(mode.timestamp, 42U);
  EXPECT_FLOAT_EQ(setpoint.velocity[0], 2.0F);
  EXPECT_FLOAT_EQ(setpoint.velocity[1], 1.0F);
  EXPECT_FLOAT_EQ(setpoint.velocity[2], -3.0F);
  EXPECT_FLOAT_EQ(setpoint.yawspeed, -0.4F);
  EXPECT_TRUE(std::isnan(setpoint.position[0]));
  EXPECT_TRUE(std::isnan(setpoint.yaw));
}

TEST(PolicyContract, RejectsWrongActionDimension)
{
  EXPECT_THROW(
    PolicyContract::processAction(
      std::vector<float>(7, 0.0F), ArmVector::Zero(),
      ArmVector::Constant(-1.0F), ArmVector::Constant(1.0F)),
    std::invalid_argument);
}

}  // namespace
}  // namespace am_ee_pose_commander
