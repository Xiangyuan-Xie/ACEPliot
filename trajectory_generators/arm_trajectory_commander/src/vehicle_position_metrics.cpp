#include <arm_trajectory_commander/vehicle_position_metrics.hpp>

#include <cmath>
#include <stdexcept>
#include <string>

VehiclePositionSource parseVehiclePositionSource(const std::string & value)
{
  if (value == "pose_stamped") {
    return VehiclePositionSource::PoseStamped;
  }
  if (value == "odometry_pose") {
    return VehiclePositionSource::OdometryPose;
  }
  throw std::invalid_argument(
          "vehicle_position_source must be 'pose_stamped' or 'odometry_pose'");
}

std::array<double, 3> acesimNwuPositionToEnu(const std::array<double, 3> & position_nwu_m)
{
  return std::array<double, 3>{-position_nwu_m[1], position_nwu_m[0], position_nwu_m[2]};
}

HoverPositionMetrics VehiclePositionMetricsTracker::update(const VehiclePositionSample & sample)
{
  if (!hover_reference_enu_m_.has_value()) {
    hover_reference_enu_m_ = sample.position_enu_m;
  }

  HoverPositionMetrics metrics;
  metrics.position_enu_m = sample.position_enu_m;
  metrics.sample_time_s = sample.sample_time_s;
  for (std::size_t i = 0; i < metrics.drift_enu_m.size(); ++i) {
    metrics.drift_enu_m[i] = sample.position_enu_m[i] - hover_reference_enu_m_.value()[i];
  }
  metrics.drift_xy_m = std::hypot(metrics.drift_enu_m[0], metrics.drift_enu_m[1]);
  metrics.drift_xyz_m = std::sqrt(
    metrics.drift_enu_m[0] * metrics.drift_enu_m[0] +
    metrics.drift_enu_m[1] * metrics.drift_enu_m[1] +
    metrics.drift_enu_m[2] * metrics.drift_enu_m[2]);
  return metrics;
}

bool VehiclePositionMetricsTracker::hasReference() const
{
  return hover_reference_enu_m_.has_value();
}

void VehiclePositionSummaryAccumulator::update(const HoverPositionMetrics & metrics)
{
  ++sample_count_;

  if (previous_metrics_.has_value()) {
    const double dt_s = metrics.sample_time_s - previous_metrics_->sample_time_s;
    if (dt_s > 0.0) {
      for (std::size_t i = 0; i < weighted_position_sum_.size(); ++i) {
        weighted_position_sum_[i] += metrics.position_enu_m[i] * dt_s;
        weighted_drift_sum_[i] += metrics.drift_enu_m[i] * dt_s;
      }
      weighted_drift_xy_sum_ += metrics.drift_xy_m * dt_s;
      weighted_drift_xyz_sum_ += metrics.drift_xyz_m * dt_s;
      duration_s_ += dt_s;
    }
  }

  previous_metrics_ = metrics;
}

void VehiclePositionSummaryAccumulator::endSegment()
{
  previous_metrics_.reset();
}

std::optional<HoverPositionSummary> VehiclePositionSummaryAccumulator::summary() const
{
  if (duration_s_ <= 0.0) {
    return std::nullopt;
  }

  HoverPositionSummary result;
  result.duration_s = duration_s_;
  result.sample_count = sample_count_;
  for (std::size_t i = 0; i < result.mean_position_enu_m.size(); ++i) {
    result.mean_position_enu_m[i] = weighted_position_sum_[i] / duration_s_;
    result.mean_drift_enu_m[i] = weighted_drift_sum_[i] / duration_s_;
  }
  result.mean_drift_xy_m = weighted_drift_xy_sum_ / duration_s_;
  result.mean_drift_xyz_m = weighted_drift_xyz_sum_ / duration_s_;
  return result;
}
