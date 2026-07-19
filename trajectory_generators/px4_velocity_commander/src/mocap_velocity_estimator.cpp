#include <px4_velocity_commander/mocap_velocity_estimator.hpp>

#include <stdexcept>

MocapVelocityEstimator::MocapVelocityEstimator(MocapVelocityEstimatorConfig config)
: config_(config)
{
  if (config_.min_dt_s <= 0.0) {
    throw std::invalid_argument("velocity_estimator.min_dt_s must be positive");
  }
  if (config_.max_dt_s < config_.min_dt_s) {
    throw std::invalid_argument("velocity_estimator.max_dt_s must be >= min_dt_s");
  }
  if (config_.low_pass_alpha <= 0.0 || config_.low_pass_alpha > 1.0) {
    throw std::invalid_argument("velocity_estimator.low_pass_alpha must be in (0, 1]");
  }
}

std::optional<MocapVelocitySample> MocapVelocityEstimator::update(
  double stamp_s,
  const std::array<double, 3> & position_enu_m)
{
  if (!last_stamp_s_.has_value() || !last_position_enu_m_.has_value()) {
    last_stamp_s_ = stamp_s;
    last_position_enu_m_ = position_enu_m;
    filtered_velocity_enu_m_s_.reset();
    return std::nullopt;
  }

  const double dt_s = stamp_s - last_stamp_s_.value();
  if (dt_s < config_.min_dt_s || dt_s > config_.max_dt_s) {
    last_stamp_s_ = stamp_s;
    last_position_enu_m_ = position_enu_m;
    filtered_velocity_enu_m_s_.reset();
    return std::nullopt;
  }

  std::array<double, 3> raw_velocity{};
  for (std::size_t i = 0; i < raw_velocity.size(); ++i) {
    raw_velocity[i] = (position_enu_m[i] - last_position_enu_m_.value()[i]) / dt_s;
  }

  std::array<double, 3> filtered_velocity = raw_velocity;
  if (filtered_velocity_enu_m_s_.has_value()) {
    for (std::size_t i = 0; i < filtered_velocity.size(); ++i) {
      filtered_velocity[i] =
        config_.low_pass_alpha * raw_velocity[i] +
        (1.0 - config_.low_pass_alpha) * filtered_velocity_enu_m_s_.value()[i];
    }
  }

  last_stamp_s_ = stamp_s;
  last_position_enu_m_ = position_enu_m;
  filtered_velocity_enu_m_s_ = filtered_velocity;

  MocapVelocitySample sample;
  sample.linear_enu_m_s = filtered_velocity;
  return sample;
}

void MocapVelocityEstimator::reset()
{
  last_stamp_s_.reset();
  last_position_enu_m_.reset();
  filtered_velocity_enu_m_s_.reset();
}
