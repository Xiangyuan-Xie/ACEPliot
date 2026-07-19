#pragma once

#include <array>
#include <vector>

struct VelocityProfileConfig
{
  std::vector<double> durations_s;
  std::vector<double> vx_m_s;
  std::vector<double> vy_m_s;
  std::vector<double> vz_m_s;
  std::vector<double> yaw_rate_rad_s;
  bool loop{false};
  double max_linear_speed_m_s{2.0};
  double max_yaw_rate_rad_s{1.0};
};

struct VelocityCommand
{
  std::array<double, 3> velocity_enu_m_s{0.0, 0.0, 0.0};
  double yaw_rate_enu_rad_s{0.0};
  bool finished{false};
};

class VelocityProfile
{
public:
  explicit VelocityProfile(VelocityProfileConfig config);

  VelocityCommand sample(double elapsed_s) const;
  double totalDurationS() const;

private:
  VelocityProfileConfig config_;
  double total_duration_s_{0.0};
};
