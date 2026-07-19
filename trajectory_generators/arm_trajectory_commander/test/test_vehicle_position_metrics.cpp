#include <gtest/gtest.h>

#include <arm_trajectory_commander/vehicle_position_metrics.hpp>

#include <array>
#include <cmath>
#include <stdexcept>

namespace
{

HoverPositionMetrics makeMetrics(
  const std::array<double, 3> & position_enu_m,
  const std::array<double, 3> & drift_enu_m,
  double sample_time_s)
{
  HoverPositionMetrics metrics;
  metrics.position_enu_m = position_enu_m;
  metrics.drift_enu_m = drift_enu_m;
  metrics.drift_xy_m = std::hypot(drift_enu_m[0], drift_enu_m[1]);
  metrics.drift_xyz_m = std::sqrt(
    drift_enu_m[0] * drift_enu_m[0] +
    drift_enu_m[1] * drift_enu_m[1] +
    drift_enu_m[2] * drift_enu_m[2]);
  metrics.sample_time_s = sample_time_s;
  return metrics;
}

}  // namespace

TEST(VehiclePositionMetrics, LocksFirstSampleAsHoverReference)
{
  VehiclePositionMetricsTracker tracker;

  const HoverPositionMetrics first = tracker.update(
    VehiclePositionSample{
    std::array<double, 3>{1.0, 2.0, 3.0}, 10.0});

  EXPECT_DOUBLE_EQ(first.position_enu_m[0], 1.0);
  EXPECT_DOUBLE_EQ(first.position_enu_m[1], 2.0);
  EXPECT_DOUBLE_EQ(first.position_enu_m[2], 3.0);
  EXPECT_DOUBLE_EQ(first.drift_enu_m[0], 0.0);
  EXPECT_DOUBLE_EQ(first.drift_enu_m[1], 0.0);
  EXPECT_DOUBLE_EQ(first.drift_enu_m[2], 0.0);
  EXPECT_DOUBLE_EQ(first.drift_xy_m, 0.0);
  EXPECT_DOUBLE_EQ(first.drift_xyz_m, 0.0);
}

TEST(VehiclePositionMetrics, ComputesDriftFromHoverReference)
{
  VehiclePositionMetricsTracker tracker;
  tracker.update(VehiclePositionSample{std::array<double, 3>{1.0, 2.0, 3.0}, 10.0});

  const HoverPositionMetrics metrics = tracker.update(
    VehiclePositionSample{
    std::array<double, 3>{4.0, 6.0, 15.0}, 11.0});

  EXPECT_DOUBLE_EQ(metrics.drift_enu_m[0], 3.0);
  EXPECT_DOUBLE_EQ(metrics.drift_enu_m[1], 4.0);
  EXPECT_DOUBLE_EQ(metrics.drift_enu_m[2], 12.0);
  EXPECT_DOUBLE_EQ(metrics.drift_xy_m, 5.0);
  EXPECT_DOUBLE_EQ(metrics.drift_xyz_m, 13.0);
  EXPECT_DOUBLE_EQ(metrics.sample_time_s, 11.0);
}

TEST(VehiclePositionMetrics, ConvertsAcesimNwuPositionToEnu)
{
  const std::array<double, 3> enu = acesimNwuPositionToEnu(
    std::array<double, 3>{2.0, -3.0, 4.0});

  EXPECT_DOUBLE_EQ(enu[0], 3.0);
  EXPECT_DOUBLE_EQ(enu[1], 2.0);
  EXPECT_DOUBLE_EQ(enu[2], 4.0);
}

TEST(VehiclePositionMetrics, ParsesPositionSources)
{
  EXPECT_EQ(parseVehiclePositionSource("pose_stamped"), VehiclePositionSource::PoseStamped);
  EXPECT_EQ(parseVehiclePositionSource("odometry_pose"), VehiclePositionSource::OdometryPose);
  EXPECT_THROW(parseVehiclePositionSource("bad_source"), std::invalid_argument);
}

TEST(VehiclePositionMetrics, ComputesTimeWeightedHoverSummary)
{
  VehiclePositionSummaryAccumulator accumulator;

  accumulator.update(
    makeMetrics(
      std::array<double, 3>{0.0, 0.0, 0.0},
      std::array<double, 3>{0.0, 0.0, 0.0},
      10.0));
  accumulator.update(
    makeMetrics(
      std::array<double, 3>{1.0, 2.0, 3.0},
      std::array<double, 3>{1.0, 0.0, 0.0},
      11.0));
  accumulator.update(
    makeMetrics(
      std::array<double, 3>{4.0, 5.0, 6.0},
      std::array<double, 3>{0.0, 2.0, 0.0},
      13.0));

  const std::optional<HoverPositionSummary> summary = accumulator.summary();
  ASSERT_TRUE(summary.has_value());
  EXPECT_DOUBLE_EQ(summary->duration_s, 3.0);
  EXPECT_EQ(summary->sample_count, 3U);
  EXPECT_DOUBLE_EQ(summary->mean_position_enu_m[0], 3.0);
  EXPECT_DOUBLE_EQ(summary->mean_position_enu_m[1], 4.0);
  EXPECT_DOUBLE_EQ(summary->mean_position_enu_m[2], 5.0);
  EXPECT_DOUBLE_EQ(summary->mean_drift_enu_m[0], 1.0 / 3.0);
  EXPECT_DOUBLE_EQ(summary->mean_drift_enu_m[1], 4.0 / 3.0);
  EXPECT_DOUBLE_EQ(summary->mean_drift_enu_m[2], 0.0);
  EXPECT_DOUBLE_EQ(summary->mean_drift_xy_m, 5.0 / 3.0);
  EXPECT_DOUBLE_EQ(summary->mean_drift_xyz_m, 5.0 / 3.0);
}

TEST(VehiclePositionMetrics, IgnoresInvalidSummaryTimeSteps)
{
  VehiclePositionSummaryAccumulator accumulator;

  accumulator.update(
    makeMetrics(
      std::array<double, 3>{0.0, 0.0, 0.0},
      std::array<double, 3>{0.0, 0.0, 0.0},
      10.0));
  accumulator.update(
    makeMetrics(
      std::array<double, 3>{100.0, 100.0, 100.0},
      std::array<double, 3>{100.0, 100.0, 100.0},
      10.0));
  accumulator.update(
    makeMetrics(
      std::array<double, 3>{2.0, 4.0, 6.0},
      std::array<double, 3>{0.0, 3.0, 4.0},
      11.0));

  const std::optional<HoverPositionSummary> summary = accumulator.summary();
  ASSERT_TRUE(summary.has_value());
  EXPECT_DOUBLE_EQ(summary->duration_s, 1.0);
  EXPECT_EQ(summary->sample_count, 3U);
  EXPECT_DOUBLE_EQ(summary->mean_position_enu_m[0], 2.0);
  EXPECT_DOUBLE_EQ(summary->mean_position_enu_m[1], 4.0);
  EXPECT_DOUBLE_EQ(summary->mean_position_enu_m[2], 6.0);
  EXPECT_DOUBLE_EQ(summary->mean_drift_xy_m, 3.0);
  EXPECT_DOUBLE_EQ(summary->mean_drift_xyz_m, 5.0);
}

TEST(VehiclePositionMetrics, SummaryUnavailableWithoutValidTrackingDuration)
{
  VehiclePositionSummaryAccumulator accumulator;

  EXPECT_FALSE(accumulator.summary().has_value());

  accumulator.update(
    makeMetrics(
      std::array<double, 3>{1.0, 2.0, 3.0},
      std::array<double, 3>{0.0, 0.0, 0.0},
      10.0));

  EXPECT_FALSE(accumulator.summary().has_value());
}
