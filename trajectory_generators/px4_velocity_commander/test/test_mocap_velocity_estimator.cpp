#include <gtest/gtest.h>

#include <px4_velocity_commander/mocap_velocity_estimator.hpp>

TEST(MocapVelocityEstimator, ComputesVelocityFromPoseDifferences)
{
  MocapVelocityEstimatorConfig config;
  config.min_dt_s = 0.01;
  config.max_dt_s = 0.5;
  config.low_pass_alpha = 1.0;
  MocapVelocityEstimator estimator(config);

  EXPECT_FALSE(estimator.update(10.0, {0.0, 0.0, 0.0}).has_value());

  const auto velocity = estimator.update(10.1, {1.0, 0.2, -0.1});
  ASSERT_TRUE(velocity.has_value());
  EXPECT_NEAR(velocity->linear_enu_m_s[0], 10.0, 1e-9);
  EXPECT_NEAR(velocity->linear_enu_m_s[1], 2.0, 1e-9);
  EXPECT_NEAR(velocity->linear_enu_m_s[2], -1.0, 1e-9);
}

TEST(MocapVelocityEstimator, ResetsOnInvalidTimeStep)
{
  MocapVelocityEstimatorConfig config;
  config.min_dt_s = 0.01;
  config.max_dt_s = 0.5;
  config.low_pass_alpha = 1.0;
  MocapVelocityEstimator estimator(config);

  EXPECT_FALSE(estimator.update(1.0, {0.0, 0.0, 0.0}).has_value());
  EXPECT_FALSE(estimator.update(2.0, {10.0, 0.0, 0.0}).has_value());

  const auto velocity = estimator.update(2.1, {11.0, 0.0, 0.0});
  ASSERT_TRUE(velocity.has_value());
  EXPECT_NEAR(velocity->linear_enu_m_s[0], 10.0, 1e-9);
}

TEST(MocapVelocityEstimator, AppliesLowPassFilter)
{
  MocapVelocityEstimatorConfig config;
  config.min_dt_s = 0.01;
  config.max_dt_s = 0.5;
  config.low_pass_alpha = 0.5;
  MocapVelocityEstimator estimator(config);

  EXPECT_FALSE(estimator.update(0.0, {0.0, 0.0, 0.0}).has_value());
  const auto first = estimator.update(0.1, {1.0, 0.0, 0.0});
  ASSERT_TRUE(first.has_value());
  EXPECT_NEAR(first->linear_enu_m_s[0], 10.0, 1e-9);

  const auto second = estimator.update(0.2, {3.0, 0.0, 0.0});
  ASSERT_TRUE(second.has_value());
  EXPECT_NEAR(second->linear_enu_m_s[0], 15.0, 1e-9);
}
