#include <gtest/gtest.h>

#include <limits>

#include <am_position_offboard_reference.hpp>
#include <px4_ros2/utils/frame_conversion.hpp>

namespace
{
float quietNaN()
{
  return std::numeric_limits<float>::quiet_NaN();
}
}  // namespace

TEST(AmPositionOffboardReference, RejectsUnsupportedOffboardFlags)
{
  px4_msgs::msg::OffboardControlMode mode{};
  mode.position = true;
  mode.body_rate = true;

  EXPECT_FALSE(isSupportedAmPositionOffboardMode(mode));
}

TEST(AmPositionOffboardReference, ConvertsPositionAndYawFromNedToEnu)
{
  px4_msgs::msg::OffboardControlMode mode{};
  mode.position = true;

  px4_msgs::msg::TrajectorySetpoint setpoint{};
  setpoint.position[0] = 1.0f;
  setpoint.position[1] = -2.0f;
  setpoint.position[2] = -3.0f;
  setpoint.velocity[0] = quietNaN();
  setpoint.velocity[1] = quietNaN();
  setpoint.velocity[2] = quietNaN();
  setpoint.acceleration[0] = quietNaN();
  setpoint.acceleration[1] = quietNaN();
  setpoint.acceleration[2] = quietNaN();
  setpoint.yaw = 0.4f;
  setpoint.yawspeed = quietNaN();

  const Eigen::Vector3f fallback_pos_w{5.0f, 6.0f, 7.0f};
  const float fallback_yaw_w = -0.2f;
  const Eigen::Quaternionf root_quat_w = Eigen::Quaternionf::Identity();

  const AmPositionOffboardReference reference = buildAmPositionOffboardReference(
    mode,
    setpoint,
    fallback_pos_w,
    fallback_yaw_w,
    root_quat_w);

  const Eigen::Vector3f expected_pos_w = px4_ros2::positionNedToEnu(
    Eigen::Vector3f(
      setpoint.position[0], setpoint.position[1], setpoint.position[2]));
  const Eigen::Quaternionf expected_quat(
    Eigen::AngleAxisf(px4_ros2::yawNedToEnu(setpoint.yaw), Eigen::Vector3f::UnitZ()));
  EXPECT_TRUE(reference.valid);
  EXPECT_TRUE(reference.position_active[0]);
  EXPECT_TRUE(reference.position_active[1]);
  EXPECT_TRUE(reference.position_active[2]);
  EXPECT_FLOAT_EQ(reference.desired_pos_w.x(), expected_pos_w.x());
  EXPECT_FLOAT_EQ(reference.desired_pos_w.y(), expected_pos_w.y());
  EXPECT_FLOAT_EQ(reference.desired_pos_w.z(), expected_pos_w.z());
  EXPECT_FLOAT_EQ(reference.desired_lin_vel_b.norm(), 0.0f);
  EXPECT_FLOAT_EQ(reference.desired_quat_w.angularDistance(expected_quat), 0.0f);
}

TEST(AmPositionOffboardReference, ConvertsVelocityAndYawRateIntoBodyFrame)
{
  px4_msgs::msg::OffboardControlMode mode{};
  mode.velocity = true;

  px4_msgs::msg::TrajectorySetpoint setpoint{};
  setpoint.position[0] = quietNaN();
  setpoint.position[1] = quietNaN();
  setpoint.position[2] = quietNaN();
  setpoint.velocity[0] = 2.0f;
  setpoint.velocity[1] = 1.0f;
  setpoint.velocity[2] = -0.5f;
  setpoint.acceleration[0] = quietNaN();
  setpoint.acceleration[1] = quietNaN();
  setpoint.acceleration[2] = quietNaN();
  setpoint.yaw = quietNaN();
  setpoint.yawspeed = 0.3f;

  const Eigen::Vector3f fallback_pos_w{5.0f, 6.0f, 7.0f};
  const float fallback_yaw_w = -0.6f;
  const Eigen::Quaternionf root_quat_w(
    Eigen::AngleAxisf(static_cast<float>(M_PI) / 2.0f, Eigen::Vector3f::UnitZ()));

  const AmPositionOffboardReference reference = buildAmPositionOffboardReference(
    mode,
    setpoint,
    fallback_pos_w,
    fallback_yaw_w,
    root_quat_w);

  const Eigen::Vector3f vel_w = px4_ros2::positionNedToEnu(
    Eigen::Vector3f(
      setpoint.velocity[0], setpoint.velocity[1], setpoint.velocity[2]));
  const Eigen::Vector3f expected_vel_b = root_quat_w.inverse() * vel_w;
  const Eigen::Quaternionf expected_quat(
    Eigen::AngleAxisf(fallback_yaw_w, Eigen::Vector3f::UnitZ()));

  EXPECT_TRUE(reference.valid);
  EXPECT_TRUE(reference.velocity_active[0]);
  EXPECT_TRUE(reference.velocity_active[1]);
  EXPECT_TRUE(reference.velocity_active[2]);
  EXPECT_FLOAT_EQ(reference.desired_pos_w.x(), fallback_pos_w.x());
  EXPECT_FLOAT_EQ(reference.desired_pos_w.y(), fallback_pos_w.y());
  EXPECT_FLOAT_EQ(reference.desired_pos_w.z(), fallback_pos_w.z());
  EXPECT_NEAR(reference.desired_lin_vel_b.x(), expected_vel_b.x(), 1.0e-6f);
  EXPECT_NEAR(reference.desired_lin_vel_b.y(), expected_vel_b.y(), 1.0e-6f);
  EXPECT_NEAR(reference.desired_lin_vel_b.z(), expected_vel_b.z(), 1.0e-6f);
  EXPECT_NEAR(
    reference.desired_ang_vel_b.z(), px4_ros2::yawRateNedToEnu(setpoint.yawspeed),
    1.0e-6f);
  EXPECT_FLOAT_EQ(reference.desired_quat_w.angularDistance(expected_quat), 0.0f);
}
