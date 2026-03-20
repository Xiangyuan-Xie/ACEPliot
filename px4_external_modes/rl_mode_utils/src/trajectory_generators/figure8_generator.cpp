#include <trajectory_generators/figure8_generator.hpp>
#include "../../../rl_base_mode/include/robot_data.hpp"
#include <px4_ros2/utils/frame_conversion.hpp>
#include <algorithm>
#include <cmath>
#include <rclcpp/node.hpp>

// 5th-order smoothstep (minimum-jerk) for 0->1 transitions with zero end slopes
static inline float smooth5(float u)
{
  u = std::clamp(u, 0.f, 1.f);
  return 10.f * u * u * u - 15.f * u * u * u * u + 6.f * u * u * u * u * u;
}

// || dX/dψ || used to map ψ̇ to arc-length speed v = ||X'(ψ)|| ψ̇
static inline float dpos_dphase_norm(float A, float B, float phase)
{
  // x = A sinψ      -> x' = A cosψ
  // y = 0.5 B sin2ψ -> y' = B cos2ψ
  const float c1 = std::cos(phase);
  const float c2 = std::cos(2.f * phase);
  const float dx = A * c1;
  const float dy = B * c2;
  return std::sqrt(dx * dx + dy * dy);
}

// Path curvature κ(ψ) = |x'y'' - y'x''| / (x'^2 + y'^2)^(3/2)
// Used to convert an acceleration bound into a bound on ψ̇.
static inline float curvature(float A, float B, float phase)
{
  const float c1 = std::cos(phase);
  const float c2 = std::cos(2.f * phase);
  const float s1 = std::sin(phase);
  const float s2 = std::sin(2.f * phase);

  const float x1 = A * c1;        // x'
  const float y1 = B * c2;        // y'
  const float x2 = -A * s1;       // x''
  const float y2 = -2.f * B * s2; // y''

  const float num = std::fabs(x1 * y2 - y1 * x2);
  const float den = std::pow(x1 * x1 + y1 * y1, 1.5f);
  return num / (den + 1e-6f);     // avoid divide-by-zero
}

void Figure8Generator::reset(const RobotData & robot_data)
{
  // Record starting pose & heading
  start_pos_ = robot_data.RootPosW();
  origin_ = start_pos_.head<2>();
  yaw_ = robot_data.HeadingW();

  // Reset state/time/phase and counters
  state_ = State::CLIMBING;
  t_ = 0.f;
  phase_ = 0.f;
  psi_dot_ = 0.f;
  phase_sum_ = 0.0;
  decel_t_ = 0.f;

  prev_velocity_.setZero();
  hold_xy_.setZero();
}

void Figure8Generator::setParams(const Figure8Params & params)
{
  p_ = params;
}

const Figure8Generator::Figure8Params & Figure8Generator::getParams() const
{
  return p_;
}

void Figure8Generator::setMaxSpeed(float max_speed_mps)
{
  p_.max_speed = max_speed_mps;
}

float Figure8Generator::getTheoreticalMaxSpeed() const
{
  const float w = 2.f * M_PI / p_.period_s;
  const float max_vx = p_.A * w;
  const float max_vy = p_.B * w;
  return std::sqrt(max_vx * max_vx + max_vy * max_vy);
}

TrajectorySample Figure8Generator::step(float dt, const RobotData & robot_data)
{
  t_ += dt;
  TrajectorySample sample;

  // Update yaw with wrapping to (-π, π]
  yaw_ = px4_ros2::wrapPi(yaw_ + p_.yaw_rate * dt);
  sample.yaw = yaw_;
  sample.yaw_rate = p_.yaw_rate;

  // Absolute target heights
  const float z_target = start_pos_.z() + p_.target_height;  // cruise altitude
  const float z_land = start_pos_.z();                       // landing altitude

  // Helper lambda: integrate the parametric figure-8 with a given psi_dot (already limited)
  auto computePoseVel = [&](float psi_dot_limited) -> std::pair<Eigen::Vector3f, Eigen::Vector3f> {
      phase_ += psi_dot_limited * dt;
      phase_sum_ += psi_dot_limited * dt;

      const float s1 = std::sin(phase_), c1 = std::cos(phase_);
      const float s2 = std::sin(2.f * phase_), c2 = std::cos(2.f * phase_);

      const float x = origin_.x() + p_.A * s1;
      const float y = origin_.y() + 0.5f * p_.B * s2;
      const float z = z_target;

      const float vx = p_.A * c1 * psi_dot_limited;
      const float vy = p_.B * c2 * psi_dot_limited;

      return {Eigen::Vector3f(x, y, z), Eigen::Vector3f(vx, vy, 0.f)};
    };

  // Helper lambda: compute ψ̇ command from envelopes and constraints, then apply d(ψ̇)/dt limit
  auto updatePsiDotLimited = [&](float psi_dot_nominal) {
      const float dpos = std::max(1e-4f, dpos_dphase_norm(p_.A, p_.B, phase_));
      const float kapp = std::max(1e-6f, curvature(p_.A, p_.B, phase_));

      // Convert v_max and a_max into ψ̇ limits
      float psi_dot_max_speed = std::numeric_limits<float>::infinity();
      if (p_.max_speed > 0.f) {
        psi_dot_max_speed = p_.max_speed / dpos;
      }

      float psi_dot_max_acc = std::numeric_limits<float>::infinity();
      if (p_.max_acceleration > 0.f) {
        // a_n = v^2 κ = (||X'(ψ)|| ψ̇)^2 κ(ψ) <= a_max  ->  ψ̇ <= sqrt(a_max/κ)/||X'(ψ)||
        psi_dot_max_acc = std::sqrt(p_.max_acceleration / kapp) / dpos;
      }

      float psi_dot_cmd = psi_dot_nominal;
      psi_dot_cmd = std::min(psi_dot_cmd, psi_dot_max_speed);
      psi_dot_cmd = std::min(psi_dot_cmd, psi_dot_max_acc);

      // Near-jerk smoothing in phase domain: limit d(ψ̇)/dt
      if (p_.max_phase_accel > 0.f) {
        const float dpsi_dot = std::clamp(
          psi_dot_cmd - psi_dot_,
          -p_.max_phase_accel * dt,
          p_.max_phase_accel * dt);
        psi_dot_ += dpsi_dot;
      } else {
        psi_dot_ = psi_dot_cmd;
      }
    };

  switch (state_) {

    // ---------------- CLIMBING: reach cruise height with vertical accel limiting ----------------
    case State::CLIMBING: {
        const float z_now = robot_data.RootPosW().z();
        const float err = z_target - z_now;

        if (std::fabs(err) < 0.05f) {
          // Transition into phase ramp-up
          state_ = State::TRANSITION;
          t_ = 0.f;
          phase_ = 0.f;
          psi_dot_ = 0.f;
          prev_velocity_.setZero();

          sample.position = Eigen::Vector3f(origin_.x(), origin_.y(), z_target);
          sample.velocity = Eigen::Vector3f::Zero();
          break;
        }

        const float vz_des = std::copysign(p_.climb_speed, err);
        const float dv_max = p_.climb_accel * dt;
        const float dv = std::clamp(vz_des - prev_velocity_.z(), -dv_max, dv_max);
        float vz = prev_velocity_.z() + dv;

        float z_next = z_now + vz * dt;

        // Prevent overshoot
        if ((err > 0.f && z_next > z_target) || (err < 0.f && z_next < z_target)) {
          z_next = z_target;
          vz = 0.f;
        }

        sample.position = Eigen::Vector3f(origin_.x(), origin_.y(), z_next);
        sample.velocity = Eigen::Vector3f(0.f, 0.f, vz);
        break;
      }

    // ---------------- TRANSITION: ramp ψ̇ up from 0->nominal via smooth5 ----------------
    case State::TRANSITION: {
        const float w_nom = 2.f * M_PI / std::max(1e-3f, p_.period_s);
        const float T = std::max(1e-3f, p_.transition_time);
        const float env = smooth5(t_ / T);           // 0 -> 1

        // Nominal ψ̇ under envelope (limits applied in updatePsiDotLimited)
        updatePsiDotLimited(w_nom * env);

        auto [pos, vel] = computePoseVel(psi_dot_);
        sample.position = pos;
        sample.velocity = vel;

        if (t_ >= p_.transition_time) {
          state_ = State::FIGURE8;
          t_ = 0.f;
        }
        break;
      }

    // ---------------- FIGURE8: steady tracking with speed/acc constraints via ψ̇ ----------------
    case State::FIGURE8: {
        if (p_.period_s < 1e-3f) {
          sample.position = Eigen::Vector3f(origin_.x(), origin_.y(), z_target);
          sample.velocity = Eigen::Vector3f::Zero();
          break;
        }

        const float w_nom = 2.f * M_PI / p_.period_s;
        updatePsiDotLimited(w_nom); // env = 1

        auto [pos, vel] = computePoseVel(psi_dot_);
        sample.position = pos;
        sample.velocity = vel;

        // Start smooth deceleration after N loops (if enabled)
        if (p_.loops_to_run > 0) {
          const double loops_done = phase_sum_ / (2.0 * M_PI);
          if (loops_done >= static_cast<double>(p_.loops_to_run)) {
            state_ = State::DECEL;
            t_ = 0.f;
            decel_t_ = 0.f;
            hold_xy_ = pos.head<2>();
          }
        }
        break;
      }

    // ---------------- DECEL: ramp ψ̇ down 1->0 via smooth5, then freeze XY and land ----------------
    case State::DECEL: {
        const float w_nom = 2.f * M_PI / std::max(1e-3f, p_.period_s);
        const float T = std::max(1e-3f, p_.decel_time);
        decel_t_ += dt;

        const float tau = std::clamp(decel_t_ / T, 0.f, 1.f);
        const float env = smooth5(1.f - tau);        // 1 -> 0

        updatePsiDotLimited(w_nom * env);

        auto [pos, vel] = computePoseVel(psi_dot_);
        sample.position = pos;
        sample.velocity = vel;

        if (decel_t_ >= p_.decel_time) {
          // Freeze XY at stop and switch to vertical descent
          state_ = State::DESCEND;
          t_ = 0.f;
          prev_velocity_.setZero();
          hold_xy_ = pos.head<2>();
        }
        break;
      }

    // ---------------- DESCEND: hold XY, descend to start height with accel limiting ----------------
    case State::DESCEND: {
        const float z_now = robot_data.RootPosW().z();
        const float err = z_land - z_now;

        if (std::fabs(err) < 0.03f) {
          sample.position = Eigen::Vector3f(hold_xy_.x(), hold_xy_.y(), z_land);
          sample.velocity = Eigen::Vector3f::Zero();
          break;
        }

        const float vz_des = std::clamp(err, -p_.land_speed, p_.land_speed);
        const float dv_max = p_.land_accel * dt;
        const float dv = std::clamp(vz_des - prev_velocity_.z(), -dv_max, dv_max);
        const float vz = prev_velocity_.z() + dv;

        float z_next = z_now + vz * dt;
        if ((err > 0.f && z_next > z_land) || (err < 0.f && z_next < z_land)) {
          z_next = z_land;
        }

        sample.position = Eigen::Vector3f(hold_xy_.x(), hold_xy_.y(), z_next);
        sample.velocity = Eigen::Vector3f(0.f, 0.f, (z_next - z_now) / dt);
        break;
      }
  }

  // Store commanded velocity for accel-limited vertical motion
  if (sample.velocity.has_value()) {
    prev_velocity_ = sample.velocity.value();
  }

  return sample;
}

nav_msgs::msg::Path Figure8Generator::getFullPathForVisualization(
  const rclcpp::Time time_stamp,
  int num_points) const
{
  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = "world";
  path_msg.header.stamp = time_stamp;

  if (p_.period_s < 1e-6f || num_points <= 1) {
    return path_msg;
  }

  const float w = 2.f * M_PI / p_.period_s;
  const float dt_traj = p_.period_s / num_points;
  const float target_z = start_pos_.z() + p_.target_height;

  for (int i = 0; i <= num_points; ++i) {
    const float t = i * dt_traj;
    const float wt = w * t;

    const float x = origin_.x() + p_.A * std::sin(wt);
    const float y = origin_.y() + 0.5f * p_.B * std::sin(2.f * wt);

    geometry_msgs::msg::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = target_z;
    pose.pose.orientation.w = 1.0; // no yaw for visualization
    path_msg.poses.push_back(pose);
  }

  return path_msg;
}
