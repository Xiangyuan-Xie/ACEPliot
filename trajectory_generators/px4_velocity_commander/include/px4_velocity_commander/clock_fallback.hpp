#pragma once

class ClockFallback
{
public:
  explicit ClockFallback(bool use_sim_time, bool allow_wall_time_without_clock);

  double effectiveNowS(double ros_now_s, double steady_now_s);
  bool usingFallback() const;
  bool hasRosClock() const;

private:
  bool use_sim_time_{false};
  bool allow_wall_time_without_clock_{true};
  bool has_ros_clock_{false};
  bool fallback_active_{false};
  double first_steady_time_s_{0.0};
  bool has_first_steady_time_{false};
};
