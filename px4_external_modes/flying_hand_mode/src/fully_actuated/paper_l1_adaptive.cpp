#include <flying_hand_mode/fully_actuated/paper_l1_adaptive.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace flying_hand_mode::fully_actuated
{
namespace
{

constexpr double kTwoPi = 6.28318530717958647692;

template<int Size>
bool positiveFinite(const Eigen::Matrix<double, Size, 1> & value) noexcept
{
  return value.allFinite() && (value.array() > 0.0).all();
}

template<int Size>
bool nonnegativeFinite(const Eigen::Matrix<double, Size, 1> & value) noexcept
{
  return value.allFinite() && (value.array() >= 0.0).all();
}

template<int Size>
Eigen::Matrix<double, Size, 1> clampSymmetric(
  const Eigen::Matrix<double, Size, 1> & value,
  const Eigen::Matrix<double, Size, 1> & limit) noexcept
{
  return value.cwiseMin(limit).cwiseMax(-limit);
}

template<int Size>
Eigen::Matrix<double, Size, 1> paperAdaptationCoefficient(
  const Eigen::Matrix<double, Size, 1> & predictor_rate,
  const Eigen::Matrix<double, Size, 1> & prediction_error,
  double dt_s) noexcept
{
  Eigen::Matrix<double, Size, 1> estimate;
  for (int index = 0; index < Size; ++index) {
    const double a = -predictor_rate[index];
    const double exponential = std::exp(a * dt_s);
    const double denominator = exponential - 1.0;
    estimate[index] = std::abs(denominator) > 1.0e-12 ?
      -(a * exponential / denominator) * prediction_error[index] : 0.0;
  }
  return estimate;
}

}  // namespace

PaperL1AdaptiveController::PaperL1AdaptiveController(const PaperL1AdaptiveConfig & config)
: config_(config)
{
  if (!isValidConfig(config_)) {
    throw std::invalid_argument("Invalid paper L1 adaptive controller configuration");
  }
}

PaperL1AdaptiveOutput PaperL1AdaptiveController::update(
  const PaperL1AdaptiveInput & input, double dt_s) noexcept
{
  PaperL1AdaptiveOutput output;
  if (!input.measured_body_velocity.allFinite() || !input.nominal_wrench_frd.allFinite() ||
    !positiveFinite(input.generalized_mass_diag) ||
    !input.modeled_drift_acceleration.allFinite() ||
    !input.measured_arm_position_rad.allFinite() ||
    !input.nominal_arm_position_command_rad.allFinite() || !std::isfinite(dt_s) ||
    dt_s <= 0.0 || dt_s > config_.maximum_dt_s || !stateIsFinite())
  {
    reset();
    return output;
  }

  if (!initialized_) {
    predicted_body_velocity_ = input.measured_body_velocity;
    predicted_arm_position_rad_ = input.measured_arm_position_rad;
    initialized_ = true;
  }

  const WrenchVector velocity_error =
    predicted_body_velocity_ - input.measured_body_velocity;
  const WrenchVector raw_acceleration = paperAdaptationCoefficient(
    config_.uav_predictor_rate_rad_s, velocity_error, dt_s);
  WrenchVector raw_uav_disturbance =
    input.generalized_mass_diag.cwiseProduct(raw_acceleration);
  raw_uav_disturbance = clampSymmetric(
    raw_uav_disturbance, config_.uav_correction_limit);
  const WrenchVector uav_filter_alpha =
    (1.0 - (-kTwoPi * config_.uav_low_pass_cutoff_hz.array() * dt_s).exp()).matrix();
  filtered_uav_disturbance_.array() += uav_filter_alpha.array() *
    (raw_uav_disturbance - filtered_uav_disturbance_).array();
  filtered_uav_disturbance_ = clampSymmetric(
    filtered_uav_disturbance_, config_.uav_correction_limit);

  const JointVector arm_error =
    predicted_arm_position_rad_ - input.measured_arm_position_rad;
  JointVector raw_arm_disturbance = config_.arm_servo_delay_s.cwiseProduct(
    paperAdaptationCoefficient(config_.arm_predictor_rate_rad_s, arm_error, dt_s));
  raw_arm_disturbance = clampSymmetric(
    raw_arm_disturbance, config_.arm_correction_limit_rad);
  const JointVector arm_filter_alpha =
    (1.0 - (-kTwoPi * config_.arm_low_pass_cutoff_hz.array() * dt_s).exp()).matrix();
  filtered_arm_disturbance_rad_.array() += arm_filter_alpha.array() *
    (raw_arm_disturbance - filtered_arm_disturbance_rad_).array();
  filtered_arm_disturbance_rad_ = clampSymmetric(
    filtered_arm_disturbance_rad_, config_.arm_correction_limit_rad);

  output.wrench_correction_frd = filtered_uav_disturbance_;
  output.wrench_command_frd = input.nominal_wrench_frd + filtered_uav_disturbance_;
  output.disturbance_estimate_frd = filtered_uav_disturbance_;
  output.arm_position_correction_rad = filtered_arm_disturbance_rad_;
  output.arm_position_command_rad =
    input.nominal_arm_position_command_rad + filtered_arm_disturbance_rad_;

  last_input_ = input;
  last_velocity_error_ = velocity_error;
  last_arm_error_ = arm_error;
  last_dt_s_ = dt_s;
  predictor_update_available_ = true;
  updatePredictor(output.wrench_command_frd, output.arm_position_command_rad);

  if (!output.wrench_command_frd.allFinite() ||
    !output.arm_position_command_rad.allFinite() || !stateIsFinite())
  {
    reset();
    return PaperL1AdaptiveOutput{};
  }
  output.predicted_body_velocity = predicted_body_velocity_;
  output.predicted_arm_position_rad = predicted_arm_position_rad_;
  output.arm_disturbance_estimate_rad = filtered_arm_disturbance_rad_;
  output.valid = true;
  return output;
}

bool PaperL1AdaptiveController::applyAcceptedCommand(
  const WrenchVector & accepted_wrench_frd,
  const JointVector & accepted_arm_position_command_rad) noexcept
{
  if (!predictor_update_available_ || !accepted_wrench_frd.allFinite() ||
    !accepted_arm_position_command_rad.allFinite())
  {
    return false;
  }
  updatePredictor(accepted_wrench_frd, accepted_arm_position_command_rad);
  if (!stateIsFinite()) {
    reset();
    return false;
  }
  return true;
}

void PaperL1AdaptiveController::updatePredictor(
  const WrenchVector & accepted_wrench_frd,
  const JointVector & accepted_arm_position_command_rad) noexcept
{
  const WrenchVector decay =
    (-config_.uav_predictor_rate_rad_s.array() * last_dt_s_).exp().matrix();
  const WrenchVector response =
    ((1.0 - decay.array()) / config_.uav_predictor_rate_rad_s.array()).matrix();
  const WrenchVector modeled_acceleration = last_input_.modeled_drift_acceleration +
    accepted_wrench_frd.cwiseQuotient(last_input_.generalized_mass_diag);
  predicted_body_velocity_ = last_input_.measured_body_velocity +
    decay.cwiseProduct(last_velocity_error_) + response.cwiseProduct(modeled_acceleration);

  const JointVector arm_decay =
    (-config_.arm_predictor_rate_rad_s.array() * last_dt_s_).exp().matrix();
  const JointVector arm_response =
    ((1.0 - arm_decay.array()) / config_.arm_predictor_rate_rad_s.array()).matrix();
  const JointVector modeled_arm_velocity =
    (accepted_arm_position_command_rad - last_input_.measured_arm_position_rad)
    .cwiseQuotient(config_.arm_servo_delay_s);
  predicted_arm_position_rad_ = last_input_.measured_arm_position_rad +
    arm_decay.cwiseProduct(last_arm_error_) + arm_response.cwiseProduct(modeled_arm_velocity);
}

void PaperL1AdaptiveController::reset() noexcept
{
  predicted_body_velocity_.setZero();
  filtered_uav_disturbance_.setZero();
  predicted_arm_position_rad_.setZero();
  filtered_arm_disturbance_rad_.setZero();
  last_input_ = PaperL1AdaptiveInput{};
  last_velocity_error_.setZero();
  last_arm_error_.setZero();
  last_dt_s_ = 0.0;
  predictor_update_available_ = false;
  initialized_ = false;
}

const PaperL1AdaptiveConfig & PaperL1AdaptiveController::config() const noexcept
{
  return config_;
}

bool PaperL1AdaptiveController::isValidConfig(
  const PaperL1AdaptiveConfig & config) noexcept
{
  return positiveFinite(config.uav_predictor_rate_rad_s) &&
         nonnegativeFinite(config.uav_low_pass_cutoff_hz) &&
         nonnegativeFinite(config.uav_correction_limit) &&
         positiveFinite(config.arm_predictor_rate_rad_s) &&
         nonnegativeFinite(config.arm_low_pass_cutoff_hz) &&
         nonnegativeFinite(config.arm_correction_limit_rad) &&
         positiveFinite(config.arm_servo_delay_s) &&
         std::isfinite(config.maximum_dt_s) && config.maximum_dt_s > 0.0;
}

bool PaperL1AdaptiveController::stateIsFinite() const noexcept
{
  return predicted_body_velocity_.allFinite() && filtered_uav_disturbance_.allFinite() &&
         predicted_arm_position_rad_.allFinite() &&
         filtered_arm_disturbance_rad_.allFinite() &&
         last_velocity_error_.allFinite() && last_arm_error_.allFinite() &&
         std::isfinite(last_dt_s_);
}

}  // namespace flying_hand_mode::fully_actuated
