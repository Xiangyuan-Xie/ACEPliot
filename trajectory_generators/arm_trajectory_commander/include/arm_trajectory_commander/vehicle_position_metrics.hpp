#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <string>

enum class VehiclePositionSource
{
  PoseStamped,
  OdometryPose,
};

struct VehiclePositionSample
{
  std::array<double, 3> position_enu_m{0.0, 0.0, 0.0};
  double sample_time_s{0.0};
};

struct HoverPositionMetrics
{
  std::array<double, 3> position_enu_m{0.0, 0.0, 0.0};
  std::array<double, 3> drift_enu_m{0.0, 0.0, 0.0};
  double drift_xy_m{0.0};
  double drift_xyz_m{0.0};
  double sample_time_s{0.0};
};

struct HoverPositionSummary
{
  std::array<double, 3> mean_position_enu_m{0.0, 0.0, 0.0};
  std::array<double, 3> mean_drift_enu_m{0.0, 0.0, 0.0};
  double mean_drift_xy_m{0.0};
  double mean_drift_xyz_m{0.0};
  double duration_s{0.0};
  std::size_t sample_count{0};
};

VehiclePositionSource parseVehiclePositionSource(const std::string & value);

std::array<double, 3> acesimNwuPositionToEnu(const std::array<double, 3> & position_nwu_m);

class VehiclePositionMetricsTracker
{
public:
  HoverPositionMetrics update(const VehiclePositionSample & sample);
  bool hasReference() const;

private:
  std::optional<std::array<double, 3>> hover_reference_enu_m_;
};

class VehiclePositionSummaryAccumulator
{
public:
  void update(const HoverPositionMetrics & metrics);
  void endSegment();
  std::optional<HoverPositionSummary> summary() const;

private:
  std::optional<HoverPositionMetrics> previous_metrics_;
  std::array<double, 3> weighted_position_sum_{0.0, 0.0, 0.0};
  std::array<double, 3> weighted_drift_sum_{0.0, 0.0, 0.0};
  double weighted_drift_xy_sum_{0.0};
  double weighted_drift_xyz_sum_{0.0};
  double duration_s_{0.0};
  std::size_t sample_count_{0};
};
