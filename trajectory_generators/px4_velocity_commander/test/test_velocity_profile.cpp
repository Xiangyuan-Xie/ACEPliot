#include <gtest/gtest.h>

#include <px4_velocity_commander/velocity_profile.hpp>

#include <stdexcept>

namespace
{
VelocityProfileConfig makeConfig()
{
  VelocityProfileConfig config;
  config.durations_s = {1.0, 2.0};
  config.vx_m_s = {1.0, -0.5};
  config.vy_m_s = {2.0, 0.0};
  config.vz_m_s = {-0.25, 0.5};
  config.yaw_rate_rad_s = {0.1, -0.2};
  config.loop = false;
  config.max_linear_speed_m_s = 3.0;
  config.max_yaw_rate_rad_s = 0.5;
  return config;
}
}  // namespace

TEST(VelocityProfile, SamplesConfiguredPhasesAndStopsAfterEnd)
{
  const VelocityProfile profile(makeConfig());

  const VelocityCommand first = profile.sample(0.5);
  EXPECT_DOUBLE_EQ(first.velocity_enu_m_s[0], 1.0);
  EXPECT_DOUBLE_EQ(first.velocity_enu_m_s[1], 2.0);
  EXPECT_DOUBLE_EQ(first.velocity_enu_m_s[2], -0.25);
  EXPECT_DOUBLE_EQ(first.yaw_rate_enu_rad_s, 0.1);
  EXPECT_FALSE(first.finished);

  const VelocityCommand second = profile.sample(1.25);
  EXPECT_DOUBLE_EQ(second.velocity_enu_m_s[0], -0.5);
  EXPECT_DOUBLE_EQ(second.velocity_enu_m_s[1], 0.0);
  EXPECT_DOUBLE_EQ(second.velocity_enu_m_s[2], 0.5);
  EXPECT_DOUBLE_EQ(second.yaw_rate_enu_rad_s, -0.2);
  EXPECT_FALSE(second.finished);

  const VelocityCommand stopped = profile.sample(3.1);
  EXPECT_DOUBLE_EQ(stopped.velocity_enu_m_s[0], 0.0);
  EXPECT_DOUBLE_EQ(stopped.velocity_enu_m_s[1], 0.0);
  EXPECT_DOUBLE_EQ(stopped.velocity_enu_m_s[2], 0.0);
  EXPECT_DOUBLE_EQ(stopped.yaw_rate_enu_rad_s, 0.0);
  EXPECT_TRUE(stopped.finished);
}

TEST(VelocityProfile, LoopsOverTotalDuration)
{
  auto config = makeConfig();
  config.loop = true;
  const VelocityProfile profile(config);

  const VelocityCommand wrapped = profile.sample(3.5);
  EXPECT_DOUBLE_EQ(wrapped.velocity_enu_m_s[0], 1.0);
  EXPECT_DOUBLE_EQ(wrapped.velocity_enu_m_s[1], 2.0);
  EXPECT_DOUBLE_EQ(wrapped.velocity_enu_m_s[2], -0.25);
  EXPECT_DOUBLE_EQ(wrapped.yaw_rate_enu_rad_s, 0.1);
  EXPECT_FALSE(wrapped.finished);
}

TEST(VelocityProfile, RejectsMismatchedLengthsAndUnsafeSpeeds)
{
  auto mismatched = makeConfig();
  mismatched.vx_m_s.pop_back();
  EXPECT_THROW(VelocityProfile{mismatched}, std::invalid_argument);

  auto too_fast = makeConfig();
  too_fast.max_linear_speed_m_s = 1.0;
  EXPECT_THROW(VelocityProfile{too_fast}, std::invalid_argument);

  auto too_much_yaw = makeConfig();
  too_much_yaw.max_yaw_rate_rad_s = 0.05;
  EXPECT_THROW(VelocityProfile{too_much_yaw}, std::invalid_argument);
}
