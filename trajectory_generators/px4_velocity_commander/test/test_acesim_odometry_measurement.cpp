#include <gtest/gtest.h>

#include <px4_velocity_commander/acesim_odometry_measurement.hpp>

#include <nav_msgs/msg/odometry.hpp>

TEST(AcesimOdometryMeasurement, ConvertsNwuPositionToInternalEnu)
{
  nav_msgs::msg::Odometry odometry;
  odometry.pose.pose.position.x = 1.0;
  odometry.pose.pose.position.y = 2.0;
  odometry.pose.pose.position.z = 3.0;

  const auto position = acesimOdometryPositionToEnu(odometry);

  EXPECT_DOUBLE_EQ(position[0], -2.0);
  EXPECT_DOUBLE_EQ(position[1], 1.0);
  EXPECT_DOUBLE_EQ(position[2], 3.0);
}

TEST(AcesimOdometryMeasurement, ParsesSupportedMeasurementSources)
{
  EXPECT_EQ(parseMeasurementSource("pose_stamped"), MeasurementSource::PoseStamped);
  EXPECT_EQ(parseMeasurementSource("odometry_pose"), MeasurementSource::OdometryPose);
}

TEST(AcesimOdometryMeasurement, RejectsUnknownMeasurementSource)
{
  EXPECT_THROW(parseMeasurementSource("twist"), std::invalid_argument);
}
