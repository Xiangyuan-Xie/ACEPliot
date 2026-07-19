#include <gtest/gtest.h>

#include <px4_velocity_commander/offboard_conversion.hpp>

#include <cmath>

TEST(OffboardConversion, BuildsVelocityOnlyPx4Messages)
{
  VelocityCommand command;
  command.velocity_enu_m_s = {1.0, 2.0, -0.5};
  command.yaw_rate_enu_rad_s = 0.25;

  const auto mode = makeVelocityOffboardControlMode(123);
  EXPECT_EQ(mode.timestamp, 123u);
  EXPECT_FALSE(mode.position);
  EXPECT_TRUE(mode.velocity);
  EXPECT_FALSE(mode.acceleration);
  EXPECT_FALSE(mode.attitude);
  EXPECT_FALSE(mode.body_rate);
  EXPECT_FALSE(mode.thrust_and_torque);
  EXPECT_FALSE(mode.direct_actuator);

  const auto setpoint = makeVelocityTrajectorySetpoint(command, 456);
  EXPECT_EQ(setpoint.timestamp, 456u);
  EXPECT_TRUE(std::isnan(setpoint.position[0]));
  EXPECT_TRUE(std::isnan(setpoint.acceleration[0]));
  EXPECT_TRUE(std::isnan(setpoint.jerk[0]));
  EXPECT_FLOAT_EQ(setpoint.velocity[0], 2.0f);
  EXPECT_FLOAT_EQ(setpoint.velocity[1], 1.0f);
  EXPECT_FLOAT_EQ(setpoint.velocity[2], 0.5f);
  EXPECT_TRUE(std::isnan(setpoint.yaw));
  EXPECT_FLOAT_EQ(setpoint.yawspeed, -0.25f);
}
