#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include <figure8_trajectory_mode/generator.hpp>
#include <figure8_trajectory_mode/offboard_conversion.hpp>

#include <px4_ros2/utils/frame_conversion.hpp>

TEST(TrajectoryGeneratorUtilsOffboardConversion, BuildsPositionVelocityOffboardMode)
{
  const TrajectorySample sample{
    Eigen::Vector3f{1.0f, 2.0f, 3.0f},
    Eigen::Vector3f{0.1f, 0.2f, 0.3f},
    std::nullopt,
    0.4f,
    0.5f
  };

  const px4_msgs::msg::OffboardControlMode mode = makeOffboardControlMode(sample);

  EXPECT_TRUE(mode.position);
  EXPECT_TRUE(mode.velocity);
  EXPECT_FALSE(mode.acceleration);
  EXPECT_FALSE(mode.body_rate);
  EXPECT_FALSE(mode.direct_actuator);
}

TEST(TrajectoryGeneratorUtilsOffboardConversion, ConvertsTrajectorySampleFromEnuToNed)
{
  const TrajectorySample sample{
    Eigen::Vector3f{1.0f, 2.0f, 3.0f},
    Eigen::Vector3f{0.1f, 0.2f, 0.3f},
    Eigen::Vector3f{0.4f, 0.5f, 0.6f},
    0.7f,
    0.8f
  };

  const px4_msgs::msg::TrajectorySetpoint setpoint = makeTrajectorySetpoint(sample);

  const Eigen::Vector3f expected_pos_ned = px4_ros2::positionEnuToNed(sample.position.value());
  const Eigen::Vector3f expected_vel_ned = px4_ros2::positionEnuToNed(sample.velocity.value());
  const Eigen::Vector3f expected_acc_ned = px4_ros2::positionEnuToNed(sample.acceleration.value());

  EXPECT_FLOAT_EQ(setpoint.position[0], expected_pos_ned.x());
  EXPECT_FLOAT_EQ(setpoint.position[1], expected_pos_ned.y());
  EXPECT_FLOAT_EQ(setpoint.position[2], expected_pos_ned.z());
  EXPECT_FLOAT_EQ(setpoint.velocity[0], expected_vel_ned.x());
  EXPECT_FLOAT_EQ(setpoint.velocity[1], expected_vel_ned.y());
  EXPECT_FLOAT_EQ(setpoint.velocity[2], expected_vel_ned.z());
  EXPECT_FLOAT_EQ(setpoint.acceleration[0], expected_acc_ned.x());
  EXPECT_FLOAT_EQ(setpoint.acceleration[1], expected_acc_ned.y());
  EXPECT_FLOAT_EQ(setpoint.acceleration[2], expected_acc_ned.z());
  EXPECT_FLOAT_EQ(setpoint.yaw, px4_ros2::yawEnuToNed(sample.yaw.value()));
  EXPECT_FLOAT_EQ(setpoint.yawspeed, px4_ros2::yawRateEnuToNed(sample.yaw_rate.value()));
}

TEST(TrajectoryGeneratorUtilsOffboardConversion, MissingFieldsBecomeNan)
{
  const TrajectorySample sample{
    std::nullopt,
    Eigen::Vector3f{0.1f, 0.2f, 0.3f},
    std::nullopt,
    std::nullopt,
    std::nullopt
  };

  const px4_msgs::msg::TrajectorySetpoint setpoint = makeTrajectorySetpoint(sample);

  EXPECT_TRUE(std::isnan(setpoint.position[0]));
  EXPECT_TRUE(std::isnan(setpoint.position[1]));
  EXPECT_TRUE(std::isnan(setpoint.position[2]));
  EXPECT_FALSE(std::isnan(setpoint.velocity[0]));
  EXPECT_TRUE(std::isnan(setpoint.acceleration[0]));
  EXPECT_TRUE(std::isnan(setpoint.yaw));
  EXPECT_TRUE(std::isnan(setpoint.yawspeed));
}

TEST(TrajectoryGeneratorUtilsOffboardConversion, PositionOnlySampleEnablesOnlyPositionControl)
{
  const TrajectorySample sample{
    Eigen::Vector3f{1.0f, 2.0f, 3.0f},
    std::nullopt,
    std::nullopt,
    0.4f,
    std::nullopt
  };

  const px4_msgs::msg::OffboardControlMode mode = makeOffboardControlMode(sample);
  const px4_msgs::msg::TrajectorySetpoint setpoint = makeTrajectorySetpoint(sample);

  EXPECT_TRUE(mode.position);
  EXPECT_FALSE(mode.velocity);
  EXPECT_FALSE(mode.acceleration);
  EXPECT_FALSE(std::isnan(setpoint.position[0]));
  EXPECT_TRUE(std::isnan(setpoint.velocity[0]));
  EXPECT_TRUE(std::isnan(setpoint.acceleration[0]));
  EXPECT_FALSE(std::isnan(setpoint.yaw));
  EXPECT_TRUE(std::isnan(setpoint.yawspeed));
}

TEST(TrajectoryGeneratorUtilsOffboardConversion, VelocityOnlySampleEnablesOnlyVelocityControl)
{
  const TrajectorySample sample{
    std::nullopt,
    Eigen::Vector3f{0.1f, 0.2f, 0.3f},
    std::nullopt,
    std::nullopt,
    0.8f
  };

  const px4_msgs::msg::OffboardControlMode mode = makeOffboardControlMode(sample);
  const px4_msgs::msg::TrajectorySetpoint setpoint = makeTrajectorySetpoint(sample);

  EXPECT_FALSE(mode.position);
  EXPECT_TRUE(mode.velocity);
  EXPECT_FALSE(mode.acceleration);
  EXPECT_TRUE(std::isnan(setpoint.position[0]));
  EXPECT_FALSE(std::isnan(setpoint.velocity[0]));
  EXPECT_TRUE(std::isnan(setpoint.acceleration[0]));
  EXPECT_TRUE(std::isnan(setpoint.yaw));
  EXPECT_FALSE(std::isnan(setpoint.yawspeed));
}
