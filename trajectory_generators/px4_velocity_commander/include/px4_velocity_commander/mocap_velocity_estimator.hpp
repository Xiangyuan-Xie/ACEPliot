#pragma once

#include <array>
#include <optional>

struct MocapVelocityEstimatorConfig
{
  double min_dt_s{0.002};
  double max_dt_s{0.5};
  double low_pass_alpha{0.5};
};

struct MocapVelocitySample
{
  std::array<double, 3> linear_enu_m_s{0.0, 0.0, 0.0};
};

class MocapVelocityEstimator
{
public:
  explicit MocapVelocityEstimator(MocapVelocityEstimatorConfig config);

  std::optional<MocapVelocitySample> update(
    double stamp_s,
    const std::array<double, 3> & position_enu_m);

  void reset();

private:
  MocapVelocityEstimatorConfig config_;
  std::optional<double> last_stamp_s_;
  std::optional<std::array<double, 3>> last_position_enu_m_;
  std::optional<std::array<double, 3>> filtered_velocity_enu_m_s_;
};
