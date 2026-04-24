/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rc_manual_trajectory_mode/rc_manual_generator.hpp>
#include <px4_ros2/utils/frame_conversion.hpp>
#include <algorithm>
#include <cmath>

RcManualGenerator::RcManualGenerator(px4_ros2::Context & context)
: manual_control_input_(std::make_shared<px4_ros2::ManualControlInput>(context, true)) {}

void RcManualGenerator::setParams(const RcManualParams & params)
{
  p_ = params;
}

const RcManualGenerator::RcManualParams & RcManualGenerator::getParams() const
{
  return p_;
}

void RcManualGenerator::reset(const TrajectoryGeneratorState & state)
{
  yaw_ = state.heading_w;
  v_last_.setZero();
  locked_xy_ = false;
  locked_z_ = false;
  lock_xy_.setZero();
  lock_z_ = std::numeric_limits<float>::quiet_NaN();
  last_pos_w_xy_ = state.root_pos_w.head<2>().cast<float>();
}

TrajectorySample RcManualGenerator::step(float dt, const TrajectoryGeneratorState & state)
{
  const auto & rc_input = manual_control_input_;
  TrajectorySample out;

  if (!rc_input || !rc_input->isValid()) {
    out.velocity = Eigen::Vector3f::Zero();
    out.position = Eigen::Vector3f(
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::quiet_NaN());
    out.yaw = yaw_;
    out.yaw_rate = 0.f;
    return out;
  }

  // ---------------- 1) Sticks with deadzone + expo ----------------
  // Sign mapping:
  //  - roll: left(+)  -> body +Y, so negate raw to match convention
  //  - pitch: fwd(+)  -> body +X
  //  - throttle: up(+)-> ENU +Z
  //  - yaw: left(+)   -> CCW(+), negate raw to match
  float roll = -expo(deadzone(rc_input->roll(), p_.deadzone), p_.expo);
  float pitch = expo(deadzone(rc_input->pitch(), p_.deadzone), p_.expo);
  float throttle = expo(deadzone(rc_input->throttle(), p_.deadzone), p_.expo);
  float yaw_stk = -expo(deadzone(rc_input->yaw(), p_.deadzone), p_.expo);

  // ---------------- 2) Yaw update ----------------
  float yaw_rate = yaw_stk * p_.yaw_rate_max_deg * M_PI / 180.f;
  yaw_ = px4_ros2::wrapPi(yaw_ + yaw_rate * dt);

  // ---------------- 3) XY stick shaping -> unit circle -> dir scaling -> speed scale ----------------
  Eigen::Vector2f stick_xy(pitch, roll); // body: (X=forward, Y=left)

  // 3.1 unit circle limit (PX4: Sticks::limitStickUnitLengthXY)
  limitUnitCircle(stick_xy);

  // 3.2 directional scaling (PX4: mpc_vel_man_side/back relative to manual)
  if (p_.v_xy_side_k >= 0.f) {
    stick_xy.y() *= p_.v_xy_side_k;
  }
  if (p_.v_xy_back_k >= 0.f && stick_xy.x() < 0.f) {
    stick_xy.x() *= p_.v_xy_back_k;
  }

  // 3.3 base speed + estimator cap (reserve 0.3 m/s for reposition)
  float velocity_scale = p_.v_xy;
  if (std::isfinite(p_.v_xy_est_max)) {
    velocity_scale = std::clamp(velocity_scale, p_.v_xy_repos_min, p_.v_xy_est_max);
  }
  Eigen::Vector2f v_sp_xy = stick_xy * velocity_scale; // body-frame setpoint

  // 3.4 accel limiting in XY body frame
  if (p_.acc_xy > 0.f) {
    const Eigen::Vector2f dv = v_sp_xy - v_last_;
    const float max_step = p_.acc_xy * dt;
    const float n = dv.norm();
    if (n > max_step && n > 1e-6f) {
      v_sp_xy = v_last_ + dv * (max_step / n);
    }
  }
  v_last_ = v_sp_xy;

  // ---------------- 4) Vertical velocity (ENU Up +) ----------------
  float vz = 0.f;
  if (throttle > 0.f) {
    vz = throttle * p_.v_up;
  } else if (throttle < 0.f) {
    vz = -(-throttle) * p_.v_down;
  }

  // ---------------- 5) PX4-like lock logic ----------------
  // 5.1 XY lock: use stick near zero (apply_brake) AND vehicle stopped in body XY
  const bool apply_brake_xy =
    (std::fabs(pitch) <= p_.stick_brake_eps) && (std::fabs(roll) <= p_.stick_brake_eps);
  const float v_xy_body = state.root_lin_vel_b.head<2>().norm();
  const bool stopped_xy = (p_.hold_max_xy < 1e-6f) || (v_xy_body < p_.hold_max_xy);

  const Eigen::Vector2f pos_w_xy = state.root_pos_w.head<2>().cast<float>();
  if (apply_brake_xy && stopped_xy) {
    if (!locked_xy_) {
      lock_xy_ = pos_w_xy;     // enter lock
      locked_xy_ = true;
    } else {
      // emulate PX4 xy_reset_counter: if world pose jumps, shift lock to current
      const float jump = (pos_w_xy - last_pos_w_xy_).norm();
      if (jump > p_.reset_jump_thresh) {
        lock_xy_ = pos_w_xy;
      }
    }
  } else {
    locked_xy_ = false;
  }

  // 5.2 Z hold: throttle near zero (apply_brake) AND vehicle stopped in body Z
  const bool apply_brake_z = (std::fabs(throttle) <= p_.stick_brake_eps);
  const float vz_body = state.root_lin_vel_b.z();
  const bool stopped_z = (p_.hold_max_z < 0.3f) || (std::fabs(vz_body) < p_.hold_max_z);

  const float z_w = state.root_pos_w.z();
  if (apply_brake_z && stopped_z) {
    if (!locked_z_) {
      lock_z_ = z_w;   // enter Z lock
      locked_z_ = true;
    }
  } else {
    locked_z_ = false;
    lock_z_ = std::numeric_limits<float>::quiet_NaN();
  }

  // ---------------- 6) Compose output (four combinations) ----------------
  out.yaw = yaw_;
  out.yaw_rate = yaw_rate;

  if (locked_xy_ && locked_z_) {
    // Full position hold (XYZ)
    out.position = Eigen::Vector3f(lock_xy_.x(), lock_xy_.y(), lock_z_);
    out.velocity = Eigen::Vector3f(0.f, 0.f, 0.f);

  } else if (locked_xy_ && !locked_z_) {
    // XY hold + Z velocity
    out.position = Eigen::Vector3f(
      lock_xy_.x(), lock_xy_.y(),
      std::numeric_limits<float>::quiet_NaN());
    out.velocity = Eigen::Vector3f(0.f, 0.f, vz);

  } else if (!locked_xy_ && locked_z_) {
    // XY velocity + Z hold
    out.position = Eigen::Vector3f(
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::quiet_NaN(),
      lock_z_);
    out.velocity = Eigen::Vector3f(v_sp_xy.x(), v_sp_xy.y(), 0.f);

  } else {
    // All velocity mode
    out.position = Eigen::Vector3f(
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::quiet_NaN());
    out.velocity = Eigen::Vector3f(v_sp_xy.x(), v_sp_xy.y(), vz);
  }

  // ---------------- 7) Book-keeping ----------------
  last_pos_w_xy_ = pos_w_xy;
  return out;
}

// ---------------- Private helper functions ----------------
float RcManualGenerator::deadzone(float input, float dz)
{
  if (std::fabs(input) < dz) {return 0.0f;}
  return (input - std::copysign(dz, input)) / (1.0f - dz);
}

float RcManualGenerator::expo(float input, float expo_factor)
{
  if (expo_factor < 1e-6f) {return input;}
  return (std::exp(expo_factor * std::fabs(input)) - 1.0f) /
         (std::exp(expo_factor) - 1.0f) * std::copysign(1.0f, input);
}

void RcManualGenerator::limitUnitCircle(Eigen::Vector2f & v)
{
  const float n = v.norm();
  if (n > 1.f && n > 1e-6f) {v /= n;}
}
