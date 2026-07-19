#include <flying_hand_quadrotor_mode/controller_core.hpp>

#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <Eigen/QR>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace flying_hand_quadrotor_mode
{
namespace
{

constexpr int kActiveSetCount = 81;
constexpr double kTwoPi = 6.28318530717958647692;

bool isValidRotorModel(const RotorAllocationModel & model) noexcept
{
  return model.allocation_flu.allFinite() && model.minimum_thrust_n.allFinite() &&
         model.maximum_thrust_n.allFinite() && model.actuation_weights.allFinite() &&
         (model.minimum_thrust_n.array() <= model.maximum_thrust_n.array()).all() &&
         (model.actuation_weights.array() > 0.0).all();
}

template<int FreeCount>
void solveFreeVariables(
  const RotorAllocationMatrix & weighted_allocation,
  const QuadrotorActuationVector & right_hand_side,
  const std::array<int, kRotorCount> & free_indices,
  RotorVector & candidate)
{
  Eigen::Matrix<double, kQuadrotorActuationDimension, FreeCount> free_allocation;
  for (int column = 0; column < FreeCount; ++column) {
    free_allocation.col(column) = weighted_allocation.col(free_indices[column]);
  }

  const Eigen::ColPivHouseholderQR<
    Eigen::Matrix<double, kQuadrotorActuationDimension, FreeCount>> decomposition(
    free_allocation);
  const Eigen::Matrix<double, FreeCount, 1> solution = decomposition.solve(right_hand_side);
  for (int column = 0; column < FreeCount; ++column) {
    candidate[free_indices[column]] = solution[column];
  }
}

bool enforceBounds(
  const RotorAllocationModel & model,
  const std::array<RotorConstraint, kRotorCount> & constraints,
  RotorVector & candidate) noexcept
{
  for (int rotor = 0; rotor < kRotorCount; ++rotor) {
    const double lower = model.minimum_thrust_n[rotor];
    const double upper = model.maximum_thrust_n[rotor];
    const double tolerance = 1.0e-10 * std::max({1.0, std::abs(lower), std::abs(upper)});
    if (!std::isfinite(candidate[rotor]) || candidate[rotor] < lower - tolerance ||
      candidate[rotor] > upper + tolerance)
    {
      return false;
    }

    if (constraints[rotor] == RotorConstraint::kFree) {
      candidate[rotor] = std::clamp(candidate[rotor], lower, upper);
    }
  }
  return true;
}

template<int Size>
Eigen::Matrix<double, Size, 1> clampSymmetric(
  const Eigen::Matrix<double, Size, 1> & value,
  const Eigen::Matrix<double, Size, 1> & limit) noexcept
{
  return value.cwiseMin(limit).cwiseMax(-limit);
}

template<int Size>
bool isNonnegativeFinite(const Eigen::Matrix<double, Size, 1> & value) noexcept
{
  return value.allFinite() && (value.array() >= 0.0).all();
}

template<int Size>
bool isPositiveFinite(const Eigen::Matrix<double, Size, 1> & value) noexcept
{
  return value.allFinite() && (value.array() > 0.0).all();
}

}  // namespace

QuadrotorActuationVector QuadrotorActuation::vector() const noexcept
{
  QuadrotorActuationVector value;
  value[0] = thrust_up_n;
  value.tail<3>() = moment_b_nm;
  return value;
}

PhysicalWrench QuadrotorActuation::physicalWrench() const noexcept
{
  PhysicalWrench wrench;
  wrench.force_b_n.z() = thrust_up_n;
  wrench.moment_b_nm = moment_b_nm;
  return wrench;
}

QuadrotorActuation QuadrotorActuation::fromVector(
  const QuadrotorActuationVector & value) noexcept
{
  QuadrotorActuation actuation;
  actuation.thrust_up_n = value[0];
  actuation.moment_b_nm = value.tail<3>();
  return actuation;
}

bool QuadrotorActuation::allFinite() const noexcept
{
  return std::isfinite(thrust_up_n) && moment_b_nm.allFinite();
}

WrenchProjectionResult projectPhysicalWrench(
  const QuadrotorActuation & desired_actuation,
  const RotorAllocationModel & model) noexcept
{
  WrenchProjectionResult result;
  if (!desired_actuation.allFinite() || !isValidRotorModel(model)) {
    return result;
  }

  const QuadrotorActuationVector desired = desired_actuation.vector();
  const RotorAllocationMatrix weighted_allocation =
    model.actuation_weights.asDiagonal() * model.allocation_flu;
  const QuadrotorActuationVector weighted_desired =
    model.actuation_weights.cwiseProduct(desired);
  double best_cost = std::numeric_limits<double>::infinity();

  for (int active_set = 0; active_set < kActiveSetCount; ++active_set) {
    int encoded_constraints = active_set;
    RotorVector candidate = RotorVector::Zero();
    std::array<RotorConstraint, kRotorCount> constraints{};
    std::array<int, kRotorCount> free_indices{};
    int free_count = 0;

    for (int rotor = 0; rotor < kRotorCount; ++rotor) {
      const int constraint = encoded_constraints % 3;
      encoded_constraints /= 3;
      if (constraint == 0) {
        constraints[rotor] = RotorConstraint::kLower;
        candidate[rotor] = model.minimum_thrust_n[rotor];
      } else if (constraint == 1) {
        constraints[rotor] = RotorConstraint::kFree;
        free_indices[free_count] = rotor;
        ++free_count;
      } else {
        constraints[rotor] = RotorConstraint::kUpper;
        candidate[rotor] = model.maximum_thrust_n[rotor];
      }
    }

    const QuadrotorActuationVector right_hand_side =
      weighted_desired - weighted_allocation * candidate;
    switch (free_count) {
      case 0:
        break;
      case 1:
        solveFreeVariables<1>(
          weighted_allocation, right_hand_side, free_indices, candidate);
        break;
      case 2:
        solveFreeVariables<2>(
          weighted_allocation, right_hand_side, free_indices, candidate);
        break;
      case 3:
        solveFreeVariables<3>(
          weighted_allocation, right_hand_side, free_indices, candidate);
        break;
      case 4:
        solveFreeVariables<4>(
          weighted_allocation, right_hand_side, free_indices, candidate);
        break;
      default:
        continue;
    }

    if (!enforceBounds(model, constraints, candidate)) {
      continue;
    }

    const QuadrotorActuationVector weighted_error =
      model.actuation_weights.cwiseProduct(model.allocation_flu * candidate - desired);
    const double cost = weighted_error.squaredNorm();
    if (std::isfinite(cost) && cost < best_cost) {
      best_cost = cost;
      result.rotor_thrust_n = candidate;
      result.constraints = constraints;
      result.weighted_error_squared = cost;
      result.valid = true;
    }
  }

  if (result.valid) {
    result.projected_actuation =
      QuadrotorActuation::fromVector(model.allocation_flu * result.rotor_thrust_n);
  }
  return result;
}

Px4AllocatorModel buildPx4NormalizedAllocator(
  const Px4AllocatorGeometry & geometry) noexcept
{
  Px4AllocatorModel result;
  if (!geometry.rotor_position_xy_frd.allFinite() || !geometry.moment_ratio.allFinite()) {
    return result;
  }

  RotorAllocationMatrix effectiveness;
  for (int rotor = 0; rotor < kRotorCount; ++rotor) {
    effectiveness(0, rotor) = -geometry.rotor_position_xy_frd(1, rotor);
    effectiveness(1, rotor) = geometry.rotor_position_xy_frd(0, rotor);
    effectiveness(2, rotor) = geometry.moment_ratio[rotor];
    effectiveness(3, rotor) = -1.0;
  }

  const Eigen::FullPivLU<RotorAllocationMatrix> effectiveness_lu(effectiveness);
  if (!effectiveness_lu.isInvertible()) {
    return result;
  }
  RotorAllocationMatrix mixer = effectiveness_lu.inverse();

  int nonzero_roll = 0;
  int nonzero_pitch = 0;
  for (int rotor = 0; rotor < kRotorCount; ++rotor) {
    nonzero_roll += std::abs(mixer(rotor, 0)) > 1.0e-3 ? 1 : 0;
    nonzero_pitch += std::abs(mixer(rotor, 1)) > 1.0e-3 ? 1 : 0;
  }
  if (nonzero_roll == 0 || nonzero_pitch == 0) {
    return result;
  }
  const double roll_scale = std::sqrt(
    mixer.col(0).squaredNorm() / (static_cast<double>(nonzero_roll) / 2.0));
  const double pitch_scale = std::sqrt(
    mixer.col(1).squaredNorm() / (static_cast<double>(nonzero_pitch) / 2.0));
  const double roll_pitch_scale = std::max(roll_scale, pitch_scale);
  const double yaw_scale = mixer.col(2).maxCoeff();

  double thrust_scale = 0.0;
  int nonzero_thrust = 0;
  for (int rotor = 0; rotor < kRotorCount; ++rotor) {
    const double magnitude = std::abs(mixer(rotor, 3));
    thrust_scale += magnitude;
    nonzero_thrust += magnitude > std::numeric_limits<double>::epsilon() ? 1 : 0;
  }
  if (!std::isfinite(roll_pitch_scale) || roll_pitch_scale <= 0.0 ||
    !std::isfinite(yaw_scale) || yaw_scale <= 0.0 || nonzero_thrust == 0)
  {
    return result;
  }
  thrust_scale /= static_cast<double>(nonzero_thrust);
  if (!std::isfinite(thrust_scale) || thrust_scale <= 0.0) {
    return result;
  }

  mixer.col(0) /= roll_pitch_scale;
  mixer.col(1) /= roll_pitch_scale;
  mixer.col(2) /= yaw_scale;
  mixer.col(3) /= thrust_scale;
  const Eigen::FullPivLU<RotorAllocationMatrix> mixer_lu(mixer);
  if (!mixer.allFinite() || !mixer_lu.isInvertible()) {
    return result;
  }

  result.actuator_from_control = mixer;
  result.control_from_actuator = mixer_lu.inverse();
  result.valid = result.control_from_actuator.allFinite();
  return result;
}

Px4NormalizedWrench rotorThrustToPx4Normalized(
  const RotorVector & rotor_thrust_n,
  const RotorThrustCommandModel & thrust_model,
  const Px4AllocatorModel & allocator) noexcept
{
  Px4NormalizedWrench result;
  if (!rotor_thrust_n.allFinite() || !std::isfinite(thrust_model.maximum_thrust_n) ||
    thrust_model.maximum_thrust_n <= 0.0 || !std::isfinite(thrust_model.kappa) ||
    thrust_model.kappa < 0.0 || thrust_model.kappa > 1.0 || !allocator.valid ||
    !allocator.actuator_from_control.allFinite() ||
    !allocator.control_from_actuator.allFinite())
  {
    return result;
  }

  constexpr double kTolerance = 1.0e-9;
  if ((rotor_thrust_n.array() < -kTolerance).any() ||
    (rotor_thrust_n.array() > thrust_model.maximum_thrust_n + kTolerance).any())
  {
    return result;
  }

  const RotorVector bounded_thrust = rotor_thrust_n.cwiseMax(0.0).cwiseMin(
    thrust_model.maximum_thrust_n);
  const RotorVector normalized_speed =
    (bounded_thrust.array() / thrust_model.maximum_thrust_n).sqrt().matrix();
  RotorVector throttle;
  if (thrust_model.kappa < 1.0e-9) {
    throttle = normalized_speed.array().square().matrix();
  } else {
    const double linear_coefficient = 1.0 - thrust_model.kappa;
    const RotorVector square_root_throttle = (
      ((linear_coefficient * linear_coefficient +
      4.0 * thrust_model.kappa * normalized_speed.array()).sqrt() -
      linear_coefficient) / (2.0 * thrust_model.kappa)).matrix();
    throttle = square_root_throttle.array().square().matrix();
  }
  if (!throttle.allFinite()) {
    return result;
  }

  const Px4ControlVector requested_control = allocator.control_from_actuator * throttle;
  if (!requested_control.allFinite()) {
    return result;
  }

  Px4ControlVector control = requested_control;
  control.head<3>() = control.head<3>().cwiseMax(-1.0).cwiseMin(1.0);
  double collective_lower = -1.0;
  double collective_upper = 1.0;
  for (int rotor = 0; rotor < kRotorCount; ++rotor) {
    const double coefficient = allocator.actuator_from_control(rotor, 3);
    if (coefficient > kTolerance) {
      collective_lower = std::max(collective_lower, 0.0);
      collective_upper = std::min(collective_upper, 1.0 / coefficient);
    } else if (coefficient < -kTolerance) {
      collective_lower = std::max(collective_lower, 1.0 / coefficient);
      collective_upper = std::min(collective_upper, 0.0);
    }
  }
  if (collective_lower > collective_upper) {
    return result;
  }
  control[3] = std::clamp(control[3], collective_lower, collective_upper);

  const RotorVector collective_actuator =
    allocator.actuator_from_control.col(3) * control[3];
  const RotorVector torque_actuator =
    allocator.actuator_from_control.leftCols<3>() * control.head<3>();
  double torque_scale = 1.0;
  for (int rotor = 0; rotor < kRotorCount; ++rotor) {
    if (collective_actuator[rotor] < -kTolerance ||
      collective_actuator[rotor] > 1.0 + kTolerance)
    {
      return result;
    }
    if (torque_actuator[rotor] > kTolerance) {
      torque_scale = std::min(
        torque_scale, (1.0 - collective_actuator[rotor]) / torque_actuator[rotor]);
    } else if (torque_actuator[rotor] < -kTolerance) {
      torque_scale = std::min(
        torque_scale, collective_actuator[rotor] / -torque_actuator[rotor]);
    }
  }
  torque_scale = std::clamp(torque_scale, 0.0, 1.0);
  control.head<3>() *= torque_scale;
  RotorVector actuator_command = allocator.actuator_from_control * control;
  if (!actuator_command.allFinite() || (actuator_command.array() < -kTolerance).any() ||
    (actuator_command.array() > 1.0 + kTolerance).any())
  {
    return result;
  }
  actuator_command = actuator_command.cwiseMax(0.0).cwiseMin(1.0);
  const RotorVector realized_speed =
    thrust_model.kappa * actuator_command +
    (1.0 - thrust_model.kappa) * actuator_command.array().sqrt().matrix();

  result.saturated =
    (control - requested_control).cwiseAbs().maxCoeff() > kTolerance;
  result.torque_frd = control.head<3>().cast<float>();
  result.thrust_frd.z() = static_cast<float>(control[3]);
  result.actuator_command = actuator_command;
  result.realized_rotor_thrust_n =
    thrust_model.maximum_thrust_n * realized_speed.array().square().matrix();
  result.valid = true;
  return result;
}

WrenchVector BodyVelocity::vector() const noexcept
{
  WrenchVector value;
  value.head<3>() = linear_b_m_s;
  value.tail<3>() = angular_b_rad_s;
  return value;
}

BodyVelocity BodyVelocity::fromVector(const WrenchVector & value) noexcept
{
  BodyVelocity velocity;
  velocity.linear_b_m_s = value.head<3>();
  velocity.angular_b_rad_s = value.tail<3>();
  return velocity;
}

bool BodyVelocity::allFinite() const noexcept
{
  return linear_b_m_s.allFinite() && angular_b_rad_s.allFinite();
}

L1AdaptiveController::L1AdaptiveController(const L1AdaptiveConfig & config)
: config_(config)
{
  if (!isValidConfig(config_)) {
    throw std::invalid_argument("Invalid L1 adaptive controller configuration");
  }
}

L1AdaptiveOutput L1AdaptiveController::update(
  const L1AdaptiveInput & input,
  double dt_s) noexcept
{
  if (!input.measured_body_velocity.allFinite() || !input.nominal_uav_wrench.allFinite() ||
    !input.uav_input_gain.allFinite() ||
    !input.measured_arm_position_rad.allFinite() ||
    !input.nominal_arm_position_rad.allFinite() || !std::isfinite(dt_s) || dt_s <= 0.0 ||
    dt_s > config_.maximum_dt_s || !stateIsFinite())
  {
    reset();
    return safeOutput(input, true);
  }

  const Eigen::LLT<WrenchGainMatrix> input_gain_factor(input.uav_input_gain);
  if (!input.uav_input_gain.isApprox(input.uav_input_gain.transpose(), 1.0e-10) ||
    input_gain_factor.info() != Eigen::Success)
  {
    reset();
    return safeOutput(input, true);
  }

  const WrenchVector measured_body_velocity = input.measured_body_velocity.vector();
  const WrenchVector nominal_uav_wrench = input.nominal_uav_wrench.vector();
  if (!initialized_) {
    predicted_body_velocity_ = measured_body_velocity;
    predicted_arm_position_rad_ = input.measured_arm_position_rad;
    initialized_ = true;
  }

  const WrenchVector uav_prediction_error =
    predicted_body_velocity_ - measured_body_velocity;
  uav_disturbance_estimate_ -= config_.uav.adaptation_gain.asDiagonal() *
    input.uav_input_gain.transpose() * uav_prediction_error * dt_s;
  uav_disturbance_estimate_ = clampSymmetric(
    uav_disturbance_estimate_, config_.uav.wrench_correction_limit);

  const WrenchVector uav_filter_alpha =
    (1.0 -
    (-kTwoPi * config_.uav.low_pass_cutoff_hz.array() * dt_s).exp()).matrix();
  filtered_uav_disturbance_.array() +=
    uav_filter_alpha.array() *
    (uav_disturbance_estimate_ - filtered_uav_disturbance_).array();
  filtered_uav_disturbance_ = clampSymmetric(
    filtered_uav_disturbance_, config_.uav.wrench_correction_limit);

  const WrenchVector unbounded_uav_correction = -filtered_uav_disturbance_;
  const WrenchVector uav_correction = clampSymmetric(
    unbounded_uav_correction, config_.uav.wrench_correction_limit);
  const WrenchVector uav_command = nominal_uav_wrench + uav_correction;

  const JointVector arm_prediction_error =
    predicted_arm_position_rad_ - input.measured_arm_position_rad;
  arm_disturbance_estimate_rad_s_.array() -=
    config_.arm.adaptation_gain_per_s2.array() * arm_prediction_error.array() * dt_s;
  const JointVector arm_disturbance_limit_rad_s =
    config_.arm.position_correction_limit_rad.cwiseQuotient(config_.arm.servo_tau_s);
  arm_disturbance_estimate_rad_s_ = clampSymmetric(
    arm_disturbance_estimate_rad_s_, arm_disturbance_limit_rad_s);

  const JointVector arm_filter_alpha =
    (1.0 -
    (-kTwoPi * config_.arm.low_pass_cutoff_hz.array() * dt_s).exp()).matrix();
  filtered_arm_disturbance_rad_s_.array() +=
    arm_filter_alpha.array() *
    (arm_disturbance_estimate_rad_s_ - filtered_arm_disturbance_rad_s_).array();
  filtered_arm_disturbance_rad_s_ = clampSymmetric(
    filtered_arm_disturbance_rad_s_, arm_disturbance_limit_rad_s);

  const JointVector unbounded_arm_position_correction_rad =
    -config_.arm.servo_tau_s.cwiseProduct(filtered_arm_disturbance_rad_s_);
  const JointVector arm_position_correction_rad = clampSymmetric(
    unbounded_arm_position_correction_rad, config_.arm.position_correction_limit_rad);
  const JointVector arm_position_command_rad =
    input.nominal_arm_position_rad + arm_position_correction_rad;

  last_measured_body_velocity_ = measured_body_velocity;
  last_uav_input_gain_ = input.uav_input_gain;
  last_uav_prediction_error_ = uav_prediction_error;
  last_measured_arm_position_rad_ = input.measured_arm_position_rad;
  last_arm_prediction_error_ = arm_prediction_error;
  last_dt_s_ = dt_s;
  predictor_update_available_ = true;
  updatePredictor(uav_command, arm_position_command_rad);

  if (!uav_command.allFinite() || !arm_position_command_rad.allFinite() || !stateIsFinite()) {
    reset();
    return safeOutput(input, true);
  }

  L1AdaptiveOutput output;
  output.uav_wrench_correction = PhysicalWrench::fromVector(uav_correction);
  output.uav_wrench_command = PhysicalWrench::fromVector(uav_command);
  output.uav_disturbance_estimate = PhysicalWrench::fromVector(uav_disturbance_estimate_);
  output.filtered_uav_disturbance_estimate =
    PhysicalWrench::fromVector(filtered_uav_disturbance_);
  output.predicted_body_velocity = BodyVelocity::fromVector(predicted_body_velocity_);
  output.arm_position_correction_rad = arm_position_correction_rad;
  output.arm_position_command_rad = arm_position_command_rad;
  output.arm_disturbance_estimate_rad_s = arm_disturbance_estimate_rad_s_;
  output.predicted_arm_position_rad = predicted_arm_position_rad_;
  output.valid = true;
  return output;
}

bool L1AdaptiveController::applyAcceptedCommand(
  const PhysicalWrench & modeled_uav_input,
  const JointVector & arm_position_command_rad) noexcept
{
  if (!predictor_update_available_ || !modeled_uav_input.allFinite() ||
    !arm_position_command_rad.allFinite())
  {
    return false;
  }
  updatePredictor(modeled_uav_input.vector(), arm_position_command_rad);
  if (!stateIsFinite()) {
    reset();
    return false;
  }
  return true;
}

void L1AdaptiveController::updatePredictor(
  const WrenchVector & modeled_uav_input,
  const JointVector & arm_position_command_rad) noexcept
{
  const WrenchVector uav_predictor_decay =
    (-config_.uav.predictor_gain_rad_s.array() * last_dt_s_).exp().matrix();
  const WrenchVector uav_predictor_response =
    ((1.0 - uav_predictor_decay.array()) /
    config_.uav.predictor_gain_rad_s.array()).matrix();
  predicted_body_velocity_ = last_measured_body_velocity_ +
    uav_predictor_decay.cwiseProduct(last_uav_prediction_error_) +
    uav_predictor_response.cwiseProduct(
    last_uav_input_gain_ * (modeled_uav_input + uav_disturbance_estimate_));

  const JointVector arm_predictor_decay =
    (-config_.arm.predictor_gain_rad_s.array() * last_dt_s_).exp().matrix();
  const JointVector arm_predictor_response =
    ((1.0 - arm_predictor_decay.array()) /
    config_.arm.predictor_gain_rad_s.array()).matrix();
  const JointVector modeled_arm_velocity_rad_s =
    (arm_position_command_rad - last_measured_arm_position_rad_)
    .cwiseQuotient(config_.arm.servo_tau_s) + arm_disturbance_estimate_rad_s_;
  predicted_arm_position_rad_ = last_measured_arm_position_rad_ +
    arm_predictor_decay.cwiseProduct(last_arm_prediction_error_) +
    arm_predictor_response.cwiseProduct(modeled_arm_velocity_rad_s);
}

void L1AdaptiveController::reset() noexcept
{
  predicted_body_velocity_.setZero();
  uav_disturbance_estimate_.setZero();
  filtered_uav_disturbance_.setZero();
  predicted_arm_position_rad_.setZero();
  arm_disturbance_estimate_rad_s_.setZero();
  filtered_arm_disturbance_rad_s_.setZero();
  last_measured_body_velocity_.setZero();
  last_uav_input_gain_.setIdentity();
  last_uav_prediction_error_.setZero();
  last_measured_arm_position_rad_.setZero();
  last_arm_prediction_error_.setZero();
  last_dt_s_ = 0.0;
  predictor_update_available_ = false;
  initialized_ = false;
}

bool L1AdaptiveController::initialized() const noexcept
{
  return initialized_;
}

const L1AdaptiveConfig & L1AdaptiveController::config() const noexcept
{
  return config_;
}

bool L1AdaptiveController::isValidConfig(const L1AdaptiveConfig & config) noexcept
{
  return isPositiveFinite(config.uav.predictor_gain_rad_s) &&
         isNonnegativeFinite(config.uav.adaptation_gain) &&
         isNonnegativeFinite(config.uav.low_pass_cutoff_hz) &&
         isNonnegativeFinite(config.uav.wrench_correction_limit) &&
         isPositiveFinite(config.arm.servo_tau_s) &&
         isPositiveFinite(config.arm.predictor_gain_rad_s) &&
         isNonnegativeFinite(config.arm.adaptation_gain_per_s2) &&
         isNonnegativeFinite(config.arm.low_pass_cutoff_hz) &&
         isNonnegativeFinite(config.arm.position_correction_limit_rad) &&
         std::isfinite(config.maximum_dt_s) && config.maximum_dt_s > 0.0;
}

L1AdaptiveOutput L1AdaptiveController::safeOutput(
  const L1AdaptiveInput & input,
  bool state_was_reset) const noexcept
{
  L1AdaptiveOutput output;
  if (input.nominal_uav_wrench.allFinite()) {
    output.uav_wrench_command = input.nominal_uav_wrench;
  }
  if (input.nominal_arm_position_rad.allFinite()) {
    output.arm_position_command_rad = input.nominal_arm_position_rad;
  }
  output.state_was_reset = state_was_reset;
  return output;
}

bool L1AdaptiveController::stateIsFinite() const noexcept
{
  return predicted_body_velocity_.allFinite() && uav_disturbance_estimate_.allFinite() &&
         filtered_uav_disturbance_.allFinite() && predicted_arm_position_rad_.allFinite() &&
         arm_disturbance_estimate_rad_s_.allFinite() &&
         filtered_arm_disturbance_rad_s_.allFinite() &&
         last_measured_body_velocity_.allFinite() && last_uav_input_gain_.allFinite() &&
         last_uav_prediction_error_.allFinite() &&
         last_measured_arm_position_rad_.allFinite() && last_arm_prediction_error_.allFinite() &&
         std::isfinite(last_dt_s_);
}

}  // namespace flying_hand_quadrotor_mode
