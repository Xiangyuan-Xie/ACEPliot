#pragma once

#include <flying_hand_control_common/controller_types.hpp>

#include <Eigen/Core>

#include <array>

namespace flying_hand_quadrotor_mode
{

constexpr int kRotorCount = 4;
constexpr int kQuadrotorActuationDimension = 4;

using flying_hand_control_common::ControllerInput;
using flying_hand_control_common::ControllerOutput;
using flying_hand_control_common::JointVector;
using flying_hand_control_common::PhysicalWrench;
using flying_hand_control_common::WrenchGainMatrix;
using flying_hand_control_common::WrenchVector;
using flying_hand_control_common::kArmJointCount;
using flying_hand_control_common::kWrenchDimension;

using RotorVector = Eigen::Matrix<double, kRotorCount, 1>;
using QuadrotorActuationVector =
  Eigen::Matrix<double, kQuadrotorActuationDimension, 1>;
using RotorAllocationMatrix =
  Eigen::Matrix<double, kQuadrotorActuationDimension, kRotorCount>;
using Px4ControlVector = Eigen::Matrix<double, kQuadrotorActuationDimension, 1>;

// A conventional quadrotor directly actuates [Fz(up), Mx, My, Mz] in FLU.
struct QuadrotorActuation
{
  double thrust_up_n{0.0};
  Eigen::Vector3d moment_b_nm{Eigen::Vector3d::Zero()};

  QuadrotorActuationVector vector() const noexcept;
  PhysicalWrench physicalWrench() const noexcept;
  static QuadrotorActuation fromVector(const QuadrotorActuationVector & value) noexcept;
  bool allFinite() const noexcept;
};

struct RotorAllocationModel
{
  // Maps rotor thrust [N] to [Fz(up), Mx, My, Mz] in FLU.
  RotorAllocationMatrix allocation_flu{RotorAllocationMatrix::Zero()};
  RotorVector minimum_thrust_n{RotorVector::Zero()};
  RotorVector maximum_thrust_n{RotorVector::Zero()};
  QuadrotorActuationVector actuation_weights{QuadrotorActuationVector::Ones()};
};

enum class RotorConstraint
{
  kLower,
  kFree,
  kUpper,
};

struct WrenchProjectionResult
{
  QuadrotorActuation projected_actuation{};
  RotorVector rotor_thrust_n{RotorVector::Zero()};
  std::array<RotorConstraint, kRotorCount> constraints{
    RotorConstraint::kFree, RotorConstraint::kFree,
    RotorConstraint::kFree, RotorConstraint::kFree};
  double weighted_error_squared{0.0};
  bool valid{false};
};

// Returns the exact weighted 4D physical-wrench projection under rotor box limits.
WrenchProjectionResult projectPhysicalWrench(
  const QuadrotorActuation & desired_actuation,
  const RotorAllocationModel & model) noexcept;

struct RotorThrustCommandModel
{
  double maximum_thrust_n{0.0};
  double kappa{0.0};
};

struct Px4AllocatorGeometry
{
  // Rotor positions use PX4 body FRD. A fixed upward rotor axis is assumed.
  Eigen::Matrix<double, 2, kRotorCount> rotor_position_xy_frd{
    Eigen::Matrix<double, 2, kRotorCount>::Zero()};
  RotorVector moment_ratio{RotorVector::Zero()};
};

struct Px4AllocatorModel
{
  // PX4 actuator = actuator_from_control * [Mx, My, Mz, Fz].
  RotorAllocationMatrix actuator_from_control{RotorAllocationMatrix::Zero()};
  RotorAllocationMatrix control_from_actuator{RotorAllocationMatrix::Zero()};
  bool valid{false};
};

Px4AllocatorModel buildPx4NormalizedAllocator(
  const Px4AllocatorGeometry & geometry) noexcept;

struct Px4NormalizedWrench
{
  Eigen::Vector3f thrust_frd{Eigen::Vector3f::Zero()};
  Eigen::Vector3f torque_frd{Eigen::Vector3f::Zero()};
  RotorVector actuator_command{RotorVector::Zero()};
  RotorVector realized_rotor_thrust_n{RotorVector::Zero()};
  bool saturated{false};
  bool valid{false};
};

// Inverts the calibrated rotor curve and PX4's normalized allocator mixer.
Px4NormalizedWrench rotorThrustToPx4Normalized(
  const RotorVector & rotor_thrust_n,
  const RotorThrustCommandModel & thrust_model,
  const Px4AllocatorModel & allocator) noexcept;

struct BodyVelocity
{
  Eigen::Vector3d linear_b_m_s{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_b_rad_s{Eigen::Vector3d::Zero()};

  WrenchVector vector() const noexcept;
  static BodyVelocity fromVector(const WrenchVector & value) noexcept;
  bool allFinite() const noexcept;
};

struct UavAdaptiveConfig
{
  WrenchVector predictor_gain_rad_s{WrenchVector::Constant(20.0)};
  WrenchVector adaptation_gain{WrenchVector::Constant(100.0)};
  WrenchVector low_pass_cutoff_hz{WrenchVector::Constant(5.0)};
  // First three limits are newtons; final three are newton-metres.
  WrenchVector wrench_correction_limit{WrenchVector::Ones()};
};

struct ArmServoAdaptiveConfig
{
  // q_dot = (q_command - q) / servo_tau_s + disturbance_rad_s.
  JointVector servo_tau_s{JointVector::Constant(0.1)};
  JointVector predictor_gain_rad_s{JointVector::Constant(20.0)};
  JointVector adaptation_gain_per_s2{JointVector::Constant(100.0)};
  JointVector low_pass_cutoff_hz{JointVector::Constant(5.0)};
  JointVector position_correction_limit_rad{JointVector::Constant(0.2)};
};

struct L1AdaptiveConfig
{
  UavAdaptiveConfig uav{};
  ArmServoAdaptiveConfig arm{};
  double maximum_dt_s{0.1};
};

struct L1AdaptiveInput
{
  BodyVelocity measured_body_velocity{};
  PhysicalWrench nominal_uav_wrench{};
  // Maps physical body-FLU force/moment to linear/angular acceleration.
  WrenchGainMatrix uav_input_gain{WrenchGainMatrix::Identity()};
  JointVector measured_arm_position_rad{JointVector::Zero()};
  JointVector nominal_arm_position_rad{JointVector::Zero()};
};

struct L1AdaptiveOutput
{
  PhysicalWrench uav_wrench_correction{};
  PhysicalWrench uav_wrench_command{};
  PhysicalWrench uav_disturbance_estimate{};
  PhysicalWrench filtered_uav_disturbance_estimate{};
  BodyVelocity predicted_body_velocity{};
  JointVector arm_position_correction_rad{JointVector::Zero()};
  JointVector arm_position_command_rad{JointVector::Zero()};
  JointVector arm_disturbance_estimate_rad_s{JointVector::Zero()};
  JointVector predicted_arm_position_rad{JointVector::Zero()};
  bool valid{false};
  bool state_was_reset{false};
};

class L1AdaptiveController
{
public:
  explicit L1AdaptiveController(const L1AdaptiveConfig & config = L1AdaptiveConfig{});

  L1AdaptiveOutput update(const L1AdaptiveInput & input, double dt_s) noexcept;
  bool applyAcceptedCommand(
    const PhysicalWrench & modeled_uav_input,
    const JointVector & arm_position_command_rad) noexcept;
  void reset() noexcept;

  bool initialized() const noexcept;
  const L1AdaptiveConfig & config() const noexcept;
  static bool isValidConfig(const L1AdaptiveConfig & config) noexcept;

private:
  L1AdaptiveOutput safeOutput(const L1AdaptiveInput & input, bool state_was_reset) const noexcept;
  void updatePredictor(
    const WrenchVector & modeled_uav_input,
    const JointVector & arm_position_command_rad) noexcept;
  bool stateIsFinite() const noexcept;

  L1AdaptiveConfig config_{};
  WrenchVector predicted_body_velocity_{WrenchVector::Zero()};
  WrenchVector uav_disturbance_estimate_{WrenchVector::Zero()};
  WrenchVector filtered_uav_disturbance_{WrenchVector::Zero()};
  JointVector predicted_arm_position_rad_{JointVector::Zero()};
  JointVector arm_disturbance_estimate_rad_s_{JointVector::Zero()};
  JointVector filtered_arm_disturbance_rad_s_{JointVector::Zero()};
  WrenchVector last_measured_body_velocity_{WrenchVector::Zero()};
  WrenchGainMatrix last_uav_input_gain_{WrenchGainMatrix::Identity()};
  WrenchVector last_uav_prediction_error_{WrenchVector::Zero()};
  JointVector last_measured_arm_position_rad_{JointVector::Zero()};
  JointVector last_arm_prediction_error_{JointVector::Zero()};
  double last_dt_s_{0.0};
  bool predictor_update_available_{false};
  bool initialized_{false};
};

}  // namespace flying_hand_quadrotor_mode
