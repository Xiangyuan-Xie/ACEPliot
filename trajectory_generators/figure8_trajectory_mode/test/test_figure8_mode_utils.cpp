#include <gtest/gtest.h>

#include "figure8_mode_utils.hpp"

TEST(Figure8ModeUtils, PositionModeDropsVelocityAndYawRate)
{
  const TrajectorySample input{
    Eigen::Vector3f{1.0f, 2.0f, 3.0f},
    Eigen::Vector3f{0.1f, 0.2f, 0.3f},
    Eigen::Vector3f{0.4f, 0.5f, 0.6f},
    0.7f,
    0.8f
  };

  const TrajectorySample output = makePositionModeSample(input);

  ASSERT_TRUE(output.position.has_value());
  EXPECT_TRUE(output.yaw.has_value());
  EXPECT_FALSE(output.velocity.has_value());
  EXPECT_FALSE(output.acceleration.has_value());
  EXPECT_FALSE(output.yaw_rate.has_value());
}

TEST(Figure8ModeUtils, VelocityModeDropsPositionAndYaw)
{
  const TrajectorySample input{
    Eigen::Vector3f{1.0f, 2.0f, 3.0f},
    Eigen::Vector3f{0.1f, 0.2f, 0.3f},
    Eigen::Vector3f{0.4f, 0.5f, 0.6f},
    0.7f,
    0.8f
  };

  const TrajectorySample output = makeVelocityModeSample(input);

  EXPECT_FALSE(output.position.has_value());
  ASSERT_TRUE(output.velocity.has_value());
  EXPECT_FALSE(output.acceleration.has_value());
  EXPECT_FALSE(output.yaw.has_value());
  EXPECT_TRUE(output.yaw_rate.has_value());
}
