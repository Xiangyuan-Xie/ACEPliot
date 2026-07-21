#include <am_ee_pose_commander/vehicle_state.hpp>

#include <gtest/gtest.h>

#include <string>

namespace am_ee_pose_commander
{
namespace
{

px4_msgs::msg::VehicleOdometry validOdometry()
{
  px4_msgs::msg::VehicleOdometry message{};
  message.timestamp = 100;
  message.timestamp_sample = 90;
  message.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
  message.position = {1.0F, 2.0F, -3.0F};
  message.q = {1.0F, 0.0F, 0.0F, 0.0F};
  message.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD;
  message.velocity = {1.0F, 2.0F, 3.0F};
  message.angular_velocity = {0.1F, 0.2F, 0.3F};
  return message;
}

TEST(VehicleState, ConvertsPx4NedFrdStateToEnuFlu)
{
  const auto state = vehicleStateFromPx4Odometry(validOdometry());

  ASSERT_TRUE(state.has_value());
  EXPECT_FLOAT_EQ(state->position_enu_m.x(), 2.0F);
  EXPECT_FLOAT_EQ(state->position_enu_m.y(), 1.0F);
  EXPECT_FLOAT_EQ(state->position_enu_m.z(), 3.0F);
  EXPECT_FLOAT_EQ(state->linear_velocity_flu_m_s.x(), 1.0F);
  EXPECT_FLOAT_EQ(state->linear_velocity_flu_m_s.y(), -2.0F);
  EXPECT_FLOAT_EQ(state->linear_velocity_flu_m_s.z(), -3.0F);
  EXPECT_FLOAT_EQ(state->angular_velocity_flu_rad_s.x(), 0.1F);
  EXPECT_FLOAT_EQ(state->angular_velocity_flu_rad_s.y(), -0.2F);
  EXPECT_FLOAT_EQ(state->angular_velocity_flu_rad_s.z(), -0.3F);
  EXPECT_EQ(state->timestamp_sample_us, 90U);
}

TEST(VehicleState, RejectsUnsupportedPoseFrame)
{
  auto message = validOdometry();
  message.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD;
  std::string error;

  EXPECT_FALSE(vehicleStateFromPx4Odometry(message, &error).has_value());
  EXPECT_NE(error.find("pose_frame"), std::string::npos);
}

TEST(VehicleState, RejectsUnsupportedVelocityFrame)
{
  auto message = validOdometry();
  message.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_FRD;

  EXPECT_FALSE(vehicleStateFromPx4Odometry(message).has_value());
}

}  // namespace
}  // namespace am_ee_pose_commander
