#include <arm_trajectory_commander/clock_fallback.hpp>

#include <algorithm>

ClockFallback::ClockFallback(bool use_sim_time, bool allow_wall_time_without_clock)
: use_sim_time_(use_sim_time),
  allow_wall_time_without_clock_(allow_wall_time_without_clock),
  has_ros_clock_(!use_sim_time)
{
}

double ClockFallback::effectiveNowS(double ros_now_s, double steady_now_s)
{
  if (!use_sim_time_) {
    has_ros_clock_ = true;
    fallback_active_ = false;
    return ros_now_s;
  }

  if (has_ros_clock_) {
    fallback_active_ = false;
    return ros_now_s;
  }

  if (ros_now_s > 0.0) {
    has_ros_clock_ = true;
    fallback_active_ = false;
    return ros_now_s;
  }

  if (!allow_wall_time_without_clock_) {
    fallback_active_ = false;
    return ros_now_s;
  }

  if (!has_first_steady_time_) {
    first_steady_time_s_ = steady_now_s;
    has_first_steady_time_ = true;
  }

  fallback_active_ = true;
  return std::max(0.0, steady_now_s - first_steady_time_s_);
}

bool ClockFallback::usingFallback() const
{
  return fallback_active_;
}

bool ClockFallback::hasRosClock() const
{
  return has_ros_clock_;
}
