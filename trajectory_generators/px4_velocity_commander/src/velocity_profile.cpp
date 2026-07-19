#include <px4_velocity_commander/velocity_profile.hpp>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>
#include <string>
#include <utility>

namespace
{
void requireSameSize(
  const std::vector<double> & values,
  std::size_t expected,
  const char * name)
{
  if (values.size() != expected) {
    throw std::invalid_argument(std::string(name) + " length must match profile.durations_s");
  }
}
}  // namespace

VelocityProfile::VelocityProfile(VelocityProfileConfig config)
: config_(std::move(config))
{
  const std::size_t count = config_.durations_s.size();
  if (count == 0) {
    throw std::invalid_argument("profile.durations_s must not be empty");
  }
  requireSameSize(config_.vx_m_s, count, "profile.vx_m_s");
  requireSameSize(config_.vy_m_s, count, "profile.vy_m_s");
  requireSameSize(config_.vz_m_s, count, "profile.vz_m_s");
  requireSameSize(config_.yaw_rate_rad_s, count, "profile.yaw_rate_rad_s");

  if (config_.max_linear_speed_m_s < 0.0 || config_.max_yaw_rate_rad_s < 0.0) {
    throw std::invalid_argument("profile speed limits must be non-negative");
  }

  for (std::size_t i = 0; i < count; ++i) {
    if (config_.durations_s[i] <= 0.0) {
      throw std::invalid_argument("profile durations must be positive");
    }

    const double linear_speed = std::sqrt(
      config_.vx_m_s[i] * config_.vx_m_s[i] +
      config_.vy_m_s[i] * config_.vy_m_s[i] +
      config_.vz_m_s[i] * config_.vz_m_s[i]);
    if (linear_speed > config_.max_linear_speed_m_s) {
      throw std::invalid_argument("profile linear speed exceeds max_linear_speed_m_s");
    }
    if (std::abs(config_.yaw_rate_rad_s[i]) > config_.max_yaw_rate_rad_s) {
      throw std::invalid_argument("profile yaw rate exceeds max_yaw_rate_rad_s");
    }
  }

  total_duration_s_ = std::accumulate(config_.durations_s.begin(), config_.durations_s.end(), 0.0);
}

VelocityCommand VelocityProfile::sample(double elapsed_s) const
{
  VelocityCommand command;
  if (elapsed_s < 0.0) {
    elapsed_s = 0.0;
  }

  if (config_.loop && total_duration_s_ > 0.0) {
    elapsed_s = std::fmod(elapsed_s, total_duration_s_);
  } else if (elapsed_s >= total_duration_s_) {
    command.finished = true;
    return command;
  }

  double phase_end_s = 0.0;
  for (std::size_t i = 0; i < config_.durations_s.size(); ++i) {
    phase_end_s += config_.durations_s[i];
    if (elapsed_s < phase_end_s) {
      command.velocity_enu_m_s = {
        config_.vx_m_s[i],
        config_.vy_m_s[i],
        config_.vz_m_s[i],
      };
      command.yaw_rate_enu_rad_s = config_.yaw_rate_rad_s[i];
      return command;
    }
  }

  command.finished = true;
  return command;
}

double VelocityProfile::totalDurationS() const
{
  return total_duration_s_;
}
