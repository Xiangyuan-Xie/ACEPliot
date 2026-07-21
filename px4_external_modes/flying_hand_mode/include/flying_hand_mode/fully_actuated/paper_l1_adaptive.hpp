#pragma once

#include <flying_hand_mode/runtime/controller_types.hpp>

#include <Eigen/Core>

namespace flying_hand_mode::fully_actuated
{

using flying_hand_mode::runtime::JointVector;
using flying_hand_mode::runtime::WrenchVector;

struct PaperL1AdaptiveConfig
{
  WrenchVector uav_predictor_rate_rad_s{WrenchVector::Constant(10.0)};
  WrenchVector uav_low_pass_cutoff_hz{WrenchVector::Constant(2.0)};
  WrenchVector uav_correction_limit{WrenchVector::Ones()};
  JointVector arm_predictor_rate_rad_s{JointVector::Constant(10.0)};
  JointVector arm_low_pass_cutoff_hz{JointVector::Constant(2.0)};
  JointVector arm_correction_limit_rad{JointVector::Constant(0.2)};
  JointVector arm_servo_delay_s{JointVector::Constant(0.5)};
  double maximum_dt_s{0.1};
};

struct PaperL1AdaptiveInput
{
  WrenchVector measured_body_velocity{WrenchVector::Zero()};
  WrenchVector nominal_wrench_frd{WrenchVector::Zero()};
  WrenchVector generalized_mass_diag{WrenchVector::Ones()};
  WrenchVector modeled_drift_acceleration{WrenchVector::Zero()};
  JointVector measured_arm_position_rad{JointVector::Zero()};
  JointVector nominal_arm_position_command_rad{JointVector::Zero()};
};

struct PaperL1AdaptiveOutput
{
  WrenchVector wrench_correction_frd{WrenchVector::Zero()};
  WrenchVector wrench_command_frd{WrenchVector::Zero()};
  WrenchVector disturbance_estimate_frd{WrenchVector::Zero()};
  WrenchVector predicted_body_velocity{WrenchVector::Zero()};
  JointVector arm_position_correction_rad{JointVector::Zero()};
  JointVector arm_position_command_rad{JointVector::Zero()};
  JointVector arm_disturbance_estimate_rad{JointVector::Zero()};
  JointVector predicted_arm_position_rad{JointVector::Zero()};
  bool valid{false};
};

class PaperL1AdaptiveController
{
public:
  explicit PaperL1AdaptiveController(
    const PaperL1AdaptiveConfig & config = PaperL1AdaptiveConfig{});

  PaperL1AdaptiveOutput update(const PaperL1AdaptiveInput & input, double dt_s) noexcept;
  bool applyAcceptedCommand(
    const WrenchVector & accepted_wrench_frd,
    const JointVector & accepted_arm_position_command_rad) noexcept;
  void reset() noexcept;

  const PaperL1AdaptiveConfig & config() const noexcept;
  static bool isValidConfig(const PaperL1AdaptiveConfig & config) noexcept;

private:
  void updatePredictor(
    const WrenchVector & accepted_wrench_frd,
    const JointVector & accepted_arm_position_command_rad) noexcept;
  bool stateIsFinite() const noexcept;

  PaperL1AdaptiveConfig config_{};
  WrenchVector predicted_body_velocity_{WrenchVector::Zero()};
  WrenchVector filtered_uav_disturbance_{WrenchVector::Zero()};
  JointVector predicted_arm_position_rad_{JointVector::Zero()};
  JointVector filtered_arm_disturbance_rad_{JointVector::Zero()};
  PaperL1AdaptiveInput last_input_{};
  WrenchVector last_velocity_error_{WrenchVector::Zero()};
  JointVector last_arm_error_{JointVector::Zero()};
  double last_dt_s_{0.0};
  bool predictor_update_available_{false};
  bool initialized_{false};
};

}  // namespace flying_hand_mode::fully_actuated
