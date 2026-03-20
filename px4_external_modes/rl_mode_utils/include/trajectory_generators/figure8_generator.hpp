#pragma once

#include <cmath>
#include <Eigen/Dense>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include "generator.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * @brief Generates a smooth figure-8 trajectory with:
 *  - CLIMBING: ascend to target height with accel limit
 *  - TRANSITION: ramp up the figure-8 phase-rate from zero (smooth5)
 *  - FIGURE8: steady 8 with speed/acc limits via time-scaling
 *  - DECEL: ramp down phase-rate to zero after N loops (smooth5)
 *  - DESCEND: vertical landing back to start height with accel limit
 *
 * Key idea: keep position and velocity consistent by parameterizing the path by
 * phase ψ and using time-scaling ψ̇ = f(t) subject to v_max and a_max (via curvature).
 * An additional rate limit on ψ̇ provides near-jerk smoothing.
 */
class Figure8Generator final : public ITrajectoryGenerator
{
public:
  explicit Figure8Generator() = default;
  void reset(const RobotData & robot_data) override;
  TrajectorySample step(float dt, const RobotData & robot_data) override;

  /**
   * @brief Parameters for configuring the figure-8 trajectory.
   */
  struct Figure8Params
  {
    // Geometry / timing
    float period_s = 10.0f;         ///< Nominal figure-8 period [s]
    float A = 1.0f;                ///< X amplitude [m]
    float B = 1.0f;                ///< Y amplitude factor (y = 0.5*B*sin(2ψ)) [m]
    float yaw_rate = 0.0f;         ///< Yaw rate [rad/s]
    float target_height = 1.0f;    ///< Height above start position [m]

    // Limits (horizontal path)
    float max_speed = 2.0f;        ///< Max horizontal speed along path [m/s], 0 = unlimited
    float max_acceleration = 2.0f; ///< Max allowable acceleration [m/s^2] enforced via curvature

    // Phase-rate shaping (smooth start/stop)
    float transition_time = 1.0f;  ///< Ramp-up time for ψ̇ from 0 to nominal [s]
    float decel_time = 1.0f;       ///< Ramp-down time for ψ̇ to 0 [s]
    int loops_to_run = 5;          ///< Number of loops to execute; 0 = infinite

    // Phase-rate change limiter (near-jerk smoothing in phase space)
    float max_phase_accel = 20.0f; ///< Limit on d(ψ̇)/dt [rad/s^2]; 0 = disabled

    // Vertical motion (climb / land)
    float climb_speed = 0.5f;      ///< Max climb speed [m/s]
    float climb_accel = 1.0f;      ///< Max climb acceleration [m/s^2]
    float land_speed = 0.5f;       ///< Max descend speed [m/s]
    float land_accel = 1.0f;       ///< Max descend acceleration [m/s^2]
  };

  void setParams(const Figure8Params & params);
  const Figure8Params & getParams() const;
  void setMaxSpeed(float max_speed_mps);

  /**
   * @brief Theoretical max speed of the nominal parametric path (ignores limits).
   */
  float getTheoreticalMaxSpeed() const;

  /**
   * @brief Discretized path for visualization (XY figure-8 at cruise height).
   */
  nav_msgs::msg::Path getFullPathForVisualization(
    const rclcpp::Time time_stamp,
    int num_points = 200) const;

private:
  /// Generator state machine.
  enum class State { CLIMBING, TRANSITION, FIGURE8, DECEL, DESCEND };

  Figure8Params p_;
  State state_{State::CLIMBING};

  float t_{0.f};          ///< Local time within the current state [s]
  float yaw_{0.f};        ///< Current yaw [rad]

  // Phase dynamics for the figure-8
  float phase_{0.f};      ///< Phase ψ
  float psi_dot_{0.f};    ///< Current phase-rate ψ̇ [rad/s] (after limiting)
  double phase_sum_{0.0}; ///< Accumulated ∫ψ̇ dt (for loop counting) [rad]
  float decel_t_{0.f};    ///< Local time within DECEL [s]

  // Reference positions
  Eigen::Vector3f start_pos_{0.f, 0.f, 0.f}; ///< Start position at reset()
  Eigen::Vector2f origin_{0.f, 0.f};         ///< XY origin of figure-8 center

  // For vertical accel limiting and landing hold
  Eigen::Vector3f prev_velocity_{0.f, 0.f, 0.f}; ///< Previous commanded velocity
  Eigen::Vector2f hold_xy_{0.f, 0.f};            ///< XY hold point when stopping
};
