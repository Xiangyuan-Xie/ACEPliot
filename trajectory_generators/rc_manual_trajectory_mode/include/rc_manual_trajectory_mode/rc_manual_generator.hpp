#pragma once

#include <memory>
#include <limits>
#include <Eigen/Dense>
#include <px4_ros2/common/context.hpp>
#include <px4_ros2/components/manual_control_input.hpp>
#include <trajectory_generator_utils/generator.hpp>


/**
 * @class RcManualGenerator
 * @brief RC manual control trajectory generator (PX4-like position & altitude behavior)
 *
 * Conventions:
 * - XY velocity in BODY frame (X=forward, Y=left)
 * - Z velocity in ENU (Up positive)
 * - Yaw: ENU CCW positive
 * - Position output in WORLD frame (used for hold/lock on axes)
 *
 * Behavior highlights (PX4-like):
 * - Unit circle limit on XY stick vector (no diagonal overspeed)
 * - Directional caps: side/back scaling relative to base v_xy
 * - Estimator cap on v_xy with a reserved min speed (0.3 m/s) for reposition
 * - XY lock: sticks released + body XY speed below hold_max_xy -> lock XY position
 * - Z hold: throttle near zero + body Z speed below hold_max_z -> lock world Z
 */
class RcManualGenerator final : public ITrajectoryGenerator
{

public:
  explicit RcManualGenerator(px4_ros2::Context & context);

  void reset(const TrajectoryGeneratorState & state) override;

  TrajectorySample step(float dt, const TrajectoryGeneratorState & state) override;

public:
  struct RcManualParams
  {
    // Base limits
    float v_xy = 2.0f;          // [m/s] base horizontal speed
    float v_up = 1.0f;          // [m/s] climb speed
    float v_down = 0.5f;        // [m/s] descent speed
    float acc_xy = 4.0f;        // [m/s^2] XY accel limit

    // Hold thresholds
    float hold_max_xy = 0.3f;   // [m/s] XY stop threshold to lock
    float hold_max_z = 0.1f;    // [m/s] Z  stop threshold to lock

    // Stick shaping
    float yaw_rate_max_deg = 45.0f;
    float expo = 5.0f;
    float deadzone = 0.1f;

    // Directional scaling (relative to v_xy), negative to disable
    float v_xy_side_k = 0.9f;   // side speed ratio
    float v_xy_back_k = 1.2f;   // backward speed ratio

    // PX4-like extras
    float v_xy_est_max = std::numeric_limits<float>::infinity();   // estimator-provided cap
    float v_xy_repos_min = 0.3f;   // keep >=0.3 m/s for reposition
    float stick_brake_eps = 0.02f; // near-zero stick to consider "apply brake"
    float reset_jump_thresh = 0.50f; // [m] world pose jump to re-anchor XY lock
  };

  void setParams(const RcManualParams & params);
  const RcManualParams & getParams() const;

private:
  // ---------------- helpers ----------------
  static float deadzone(float input, float dz);
  static float expo(float input, float expo_factor);
  static void limitUnitCircle(Eigen::Vector2f & v);

private:
  RcManualParams p_{};

  // Manual control input
  std::shared_ptr<px4_ros2::ManualControlInput> manual_control_input_;

  float yaw_{0.0f};

  bool locked_xy_{false};
  bool locked_z_{false};

  Eigen::Vector2f lock_xy_{Eigen::Vector2f::Zero()};
  float lock_z_{std::numeric_limits<float>::quiet_NaN()};

  Eigen::Vector2f v_last_{Eigen::Vector2f::Zero()};      // last XY body velocity sp (for accel limiting)
  Eigen::Vector2f last_pos_w_xy_{Eigen::Vector2f::Zero()}; // for jump detection
};
