#include <flying_hand_mode/fully_actuated/paper_l1_adaptive.hpp>

#include <gtest/gtest.h>

namespace flying_hand_mode::fully_actuated
{
namespace
{

TEST(PaperL1Adaptive, CompensatesAllSixWrenchChannels)
{
  PaperL1AdaptiveConfig config;
  config.uav_low_pass_cutoff_hz.setConstant(1000.0);
  config.uav_correction_limit.setConstant(100.0);
  PaperL1AdaptiveController controller(config);

  PaperL1AdaptiveInput input;
  input.generalized_mass_diag << 5.0, 5.0, 5.0, 0.1, 0.1, 0.2;
  ASSERT_TRUE(controller.update(input, 0.01).valid);

  input.measured_body_velocity.setConstant(0.1);
  const PaperL1AdaptiveOutput output = controller.update(input, 0.01);
  ASSERT_TRUE(output.valid);
  EXPECT_TRUE((output.wrench_correction_frd.array().abs() > 0.0).all());
  EXPECT_TRUE(output.wrench_command_frd.isApprox(output.wrench_correction_frd));
}

TEST(PaperL1Adaptive, AddsArmDelayEstimateToCommand)
{
  PaperL1AdaptiveConfig config;
  config.arm_low_pass_cutoff_hz.setConstant(1000.0);
  PaperL1AdaptiveController controller(config);
  PaperL1AdaptiveInput input;
  ASSERT_TRUE(controller.update(input, 0.01).valid);
  input.measured_arm_position_rad.setConstant(0.05);
  input.nominal_arm_position_command_rad.setConstant(0.2);
  const PaperL1AdaptiveOutput output = controller.update(input, 0.01);
  ASSERT_TRUE(output.valid);
  EXPECT_TRUE(output.arm_position_command_rad.allFinite());
  EXPECT_FALSE(output.arm_position_correction_rad.isZero(0.0));
}

TEST(PaperL1Adaptive, InvalidTimingResetsController)
{
  PaperL1AdaptiveController controller;
  PaperL1AdaptiveInput input;
  EXPECT_FALSE(controller.update(input, 0.0).valid);
  EXPECT_TRUE(controller.update(input, 0.01).valid);
}

}  // namespace
}  // namespace flying_hand_mode::fully_actuated
