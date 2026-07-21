#include <flying_hand_mode/quadrotor/flying_hand_controller.hpp>

#include <Eigen/Cholesky>

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace flying_hand_mode::quadrotor
{
namespace
{

constexpr double kMassKg = 2.35584;
constexpr double kGravityMps2 = 9.80665;
constexpr double kRotorArmM = 0.17688;
constexpr double kMomentConstantM = 0.01296242524483617;
constexpr double kMaximumRotorThrustN = 25.000097317747848;
constexpr double kThrottleKappa = 0.3259609459890776;
constexpr double kMaximumCollectiveN = 4.0 * kMaximumRotorThrustN;
constexpr double kMaximumRollPitchNm = 2.0 * kRotorArmM * kMaximumRotorThrustN;
constexpr double kMaximumYawNm = 2.0 * kMomentConstantM * kMaximumRotorThrustN;
constexpr double kMinimumAirborneCollectiveN = 0.1 * kMassKg * kGravityMps2;
const Eigen::Matrix3d kFluFromFrd =
  Eigen::Vector3d(1.0, -1.0, -1.0).asDiagonal();
const JointVector kArmLowerRad = (
  JointVector() << -2.6485, 0.0, -2.6485, -3.1415).finished();
const JointVector kArmUpperRad = (
  JointVector() << 2.6485, 3.1415, 2.6485, 3.1415).finished();
const JointVector kArmMaximumVelocityRadS = (
  JointVector() << 6.28, 7.85, 4.71, 10.47).finished();
const JointVector kArmServoTauS = (
  JointVector() << 0.08, 0.08, 0.08, 0.06).finished();

L1AdaptiveConfig adaptiveConfig()
{
  L1AdaptiveConfig config;
  config.uav.predictor_gain_rad_s.head<3>().setConstant(5.0);
  config.uav.predictor_gain_rad_s.tail<3>().setConstant(10.0);
  config.uav.adaptation_gain.head<3>().setConstant(35.0);
  config.uav.adaptation_gain.tail<3>().setConstant(60.0);
  config.uav.low_pass_cutoff_hz.head<3>().setConstant(1.27);
  config.uav.low_pass_cutoff_hz.segment<2>(3).setConstant(0.95);
  config.uav.low_pass_cutoff_hz[5] = 0.64;
  config.uav.wrench_correction_limit.head<2>().setConstant(10.0);
  config.uav.wrench_correction_limit[2] = 5.33497;
  config.uav.wrench_correction_limit.segment<2>(3).setConstant(0.83910);
  config.uav.wrench_correction_limit[5] = 0.06149;
  config.arm.servo_tau_s = kArmServoTauS;
  config.arm.predictor_gain_rad_s.setConstant(20.0);
  config.arm.adaptation_gain_per_s2.setConstant(80.0);
  config.arm.low_pass_cutoff_hz.setConstant(1.0);
  config.arm.position_correction_limit_rad.setConstant(0.15);
  config.maximum_dt_s = 0.1;
  return config;
}

RotorAllocationModel rotorModel(const Eigen::Vector3d & center_of_mass_flu_m)
{
  RotorAllocationModel model;
  Eigen::Matrix<double, 3, kRotorCount> rotor_position_flu;
  rotor_position_flu.col(0) = Eigen::Vector3d(kRotorArmM, -kRotorArmM, 0.0071418);
  rotor_position_flu.col(1) = Eigen::Vector3d(-kRotorArmM, kRotorArmM, 0.0071418);
  rotor_position_flu.col(2) = Eigen::Vector3d(kRotorArmM, kRotorArmM, 0.0071418);
  rotor_position_flu.col(3) = Eigen::Vector3d(-kRotorArmM, -kRotorArmM, 0.0071418);
  const RotorVector direction = (RotorVector() << 1.0, 1.0, -1.0, -1.0).finished();
  for (int rotor = 0; rotor < kRotorCount; ++rotor) {
    const Eigen::Vector3d arm_flu =
      rotor_position_flu.col(rotor) - center_of_mass_flu_m;
    model.allocation_flu(0, rotor) = 1.0;
    model.allocation_flu(1, rotor) = arm_flu.y();
    model.allocation_flu(2, rotor) = -arm_flu.x();
    model.allocation_flu(3, rotor) = -direction[rotor] * kMomentConstantM;
  }
  model.maximum_thrust_n.setConstant(kMaximumRotorThrustN);
  model.actuation_weights = Eigen::Vector4d(
    1.0 / kMaximumCollectiveN,
    1.0 / kMaximumRollPitchNm,
    1.0 / kMaximumRollPitchNm,
    1.0 / kMaximumYawNm);
  return model;
}

Eigen::Vector3d clampVectorRate(
  const Eigen::Vector3d & desired, const Eigen::Vector3d & previous,
  const Eigen::Vector3d & rate, double dt_s)
{
  const Eigen::Vector3d maximum_delta = rate * dt_s;
  return previous + (desired - previous).cwiseMin(maximum_delta).cwiseMax(-maximum_delta);
}

PhysicalWrench actuatorWrenchFlu(const QuadrotorActuation & actuation_frd)
{
  PhysicalWrench wrench;
  wrench.force_b_n.z() = actuation_frd.thrust_up_n;
  wrench.moment_b_nm = kFluFromFrd * actuation_frd.moment_b_nm;
  return wrench;
}

QuadrotorActuation actuationFlu(const PhysicalWrench & wrench)
{
  QuadrotorActuation actuation;
  actuation.thrust_up_n = wrench.force_b_n.z();
  actuation.moment_b_nm = wrench.moment_b_nm;
  return actuation;
}

PhysicalWrench modeledBodyInputFlu(
  const PhysicalWrench & actuator_wrench_flu,
  const ControllerInput & input,
  const Eigen::Vector3d & center_of_mass_velocity_ned_m_s,
  const VehicleMassProperties & mass_properties)
{
  const Eigen::Vector3d velocity_frd =
    input.attitude_ned_frd.conjugate() * center_of_mass_velocity_ned_m_s;
  const Eigen::Vector3d velocity_flu = kFluFromFrd * velocity_frd;
  const Eigen::Vector3d angular_velocity_flu =
    kFluFromFrd * input.angular_velocity_frd_rad_s;
  const Eigen::Vector3d gravity_ned_n(
    0.0, 0.0, mass_properties.mass_kg * kGravityMps2);
  const Eigen::Vector3d gravity_flu_n =
    kFluFromFrd * (input.attitude_ned_frd.conjugate() * gravity_ned_n);

  PhysicalWrench modeled = actuator_wrench_flu;
  modeled.force_b_n += gravity_flu_n -
    mass_properties.mass_kg * angular_velocity_flu.cross(velocity_flu);
  modeled.moment_b_nm -=
    angular_velocity_flu.cross(
    mass_properties.inertia_com_flu_kg_m2 * angular_velocity_flu);
  return modeled;
}

}  // namespace

FlyingHandController::FlyingHandController(rclcpp::Node & node)
: adaptive_controller_(adaptiveConfig()),
  adaptive_enabled_(node.declare_parameter<bool>("adaptive_enabled", true)),
  target_position_slew_m_s_(
    node.declare_parameter<double>("target_position_slew_m_s", 0.5)),
  target_orientation_slew_rad_s_(
    node.declare_parameter<double>("target_orientation_slew_rad_s", 0.8)),
  collective_slew_n_s_(node.declare_parameter<double>("collective_slew_n_s", 120.0))
{
  const std::vector<double> moment_slew =
    node.declare_parameter<std::vector<double>>(
    "moment_slew_nm_s", {20.0, 20.0, 2.0});
  const std::vector<double> allocator_position_x =
    node.declare_parameter<std::vector<double>>(
    "px4_allocator_rotor_position_x", {1.0, -1.0, 1.0, -1.0});
  const std::vector<double> allocator_position_y =
    node.declare_parameter<std::vector<double>>(
    "px4_allocator_rotor_position_y", {1.0, -1.0, -1.0, 1.0});
  const std::vector<double> allocator_moment_ratio =
    node.declare_parameter<std::vector<double>>(
    "px4_allocator_rotor_moment_ratio", {0.05, 0.05, -0.05, -0.05});
  if (moment_slew.size() != 3U || allocator_position_x.size() != kRotorCount ||
    allocator_position_y.size() != kRotorCount || allocator_moment_ratio.size() != kRotorCount ||
    !std::isfinite(target_position_slew_m_s_) || target_position_slew_m_s_ <= 0.0 ||
    !std::isfinite(target_orientation_slew_rad_s_) || target_orientation_slew_rad_s_ <= 0.0 ||
    !std::isfinite(collective_slew_n_s_) || collective_slew_n_s_ <= 0.0 ||
    !std::all_of(
      moment_slew.begin(), moment_slew.end(),
      [](double value) {return std::isfinite(value) && value > 0.0;}))
  {
    throw std::invalid_argument("Invalid Flying Hand controller slew limits");
  }
  moment_slew_nm_s_ = Eigen::Map<const Eigen::Vector3d>(moment_slew.data());
  Px4AllocatorGeometry allocator_geometry;
  allocator_geometry.rotor_position_xy_frd.row(0) =
    Eigen::Map<const RotorVector>(allocator_position_x.data()).transpose();
  allocator_geometry.rotor_position_xy_frd.row(1) =
    Eigen::Map<const RotorVector>(allocator_position_y.data()).transpose();
  allocator_geometry.moment_ratio =
    Eigen::Map<const RotorVector>(allocator_moment_ratio.data());
  allocator_model_ = buildPx4NormalizedAllocator(allocator_geometry);
  if (!allocator_model_.valid) {
    throw std::invalid_argument("Invalid PX4 allocator rotor geometry");
  }
  thrust_model_.maximum_thrust_n = kMaximumRotorThrustN;
  thrust_model_.kappa = kThrottleKappa;
  previous_control_[0] = kMassKg * kGravityMps2;
}

ControllerOutput FlyingHandController::update(const ControllerInput & input)
{
  ControllerOutput output;
  if (pending_adaptive_controller_.has_value() || !std::isfinite(input.dt_s) ||
    input.dt_s <= 0.0 || input.dt_s > 0.1 ||
    !input.position_ned_m.allFinite() || !input.attitude_ned_frd.coeffs().allFinite() ||
    !input.linear_velocity_ned_m_s.allFinite() ||
    !input.angular_velocity_frd_rad_s.allFinite() ||
    !input.gyro_frd_rad_s.allFinite() ||
    !input.arm_position_rad.allFinite())
  {
    return output;
  }

  const VehicleMassProperties mass_properties =
    arm_kinematics_.massPropertiesFlu(input.arm_position_rad);
  if (!mass_properties.allFinite()) {
    return output;
  }
  const Eigen::Vector3d center_of_mass_frd_m =
    kFluFromFrd * mass_properties.center_of_mass_flu_m;
  const Eigen::Vector3d center_of_mass_velocity_frd_m_s =
    kFluFromFrd *
    (mass_properties.com_jacobian_flu_m_rad * input.arm_velocity_rad_s);
  const Eigen::Vector3d center_of_mass_position_ned_m =
    input.position_ned_m + input.attitude_ned_frd * center_of_mass_frd_m;
  const Eigen::Vector3d center_of_mass_velocity_ned_m_s =
    input.linear_velocity_ned_m_s + input.attitude_ned_frd *
    (input.angular_velocity_frd_rad_s.cross(center_of_mass_frd_m) +
    center_of_mass_velocity_frd_m_s);

  WholeBodySolverInput solver_input;
  solver_input.state.position_ned_m = center_of_mass_position_ned_m;
  solver_input.state.attitude_ned_frd = input.attitude_ned_frd;
  solver_input.state.velocity_ned_m_s = center_of_mass_velocity_ned_m_s;
  solver_input.state.angular_velocity_frd_rad_s = input.angular_velocity_frd_rad_s;
  solver_input.state.arm_position_rad = input.arm_position_rad;
  solver_input.mass_properties.mass_kg = mass_properties.mass_kg;
  solver_input.mass_properties.center_of_mass_flu_m =
    mass_properties.center_of_mass_flu_m;
  solver_input.mass_properties.inertia_com_flu_kg_m2 =
    mass_properties.inertia_com_flu_kg_m2;
  std::optional<EndEffectorTarget> candidate_slewed_target = slewed_target_;
  solver_input.target = slewTarget(input, candidate_slewed_target);
  const Eigen::Vector3d estimated_force_frd =
    kFluFromFrd * disturbance_estimate_flu_.force_b_n;
  solver_input.disturbance.force_frd_n.x() = estimated_force_frd.x();
  solver_input.disturbance.force_frd_n.y() = estimated_force_frd.y();
  solver_input.previous_control = previous_control_;
  if (!slewed_target_.has_value() && !previous_applied_actuation_flu_.has_value()) {
    solver_input.previous_control.segment<kArmJointCount>(4) = input.arm_position_rad;
  }

  const WholeBodySolverOutput nominal = solver_.solve(solver_input);
  if (!nominal.valid) {
    return output;
  }

  const PhysicalWrench nominal_actuator_flu = actuatorWrenchFlu(nominal.actuation_frd);
  L1AdaptiveInput adaptive_input;
  const Eigen::LLT<Eigen::Matrix3d> inertia_factor(
    mass_properties.inertia_com_flu_kg_m2);
  if (inertia_factor.info() != Eigen::Success) {
    return output;
  }
  adaptive_input.uav_input_gain.setZero();
  adaptive_input.uav_input_gain.topLeftCorner<3, 3>().diagonal().setConstant(
    1.0 / mass_properties.mass_kg);
  adaptive_input.uav_input_gain.bottomRightCorner<3, 3>() =
    inertia_factor.solve(Eigen::Matrix3d::Identity());
  const Eigen::Vector3d velocity_frd =
    input.attitude_ned_frd.conjugate() * center_of_mass_velocity_ned_m_s;
  adaptive_input.measured_body_velocity.linear_b_m_s = kFluFromFrd * velocity_frd;
  adaptive_input.measured_body_velocity.angular_b_rad_s =
    kFluFromFrd * input.gyro_frd_rad_s;
  adaptive_input.nominal_uav_wrench = modeledBodyInputFlu(
    nominal_actuator_flu, input, center_of_mass_velocity_ned_m_s, mass_properties);
  adaptive_input.measured_arm_position_rad = input.arm_position_rad;
  adaptive_input.nominal_arm_position_rad = nominal.arm_position_command_rad;

  L1AdaptiveController candidate_adaptive_controller = adaptive_controller_;
  PhysicalWrench candidate_disturbance_estimate = disturbance_estimate_flu_;
  L1AdaptiveOutput adaptive;
  if (adaptive_enabled_) {
    adaptive = candidate_adaptive_controller.update(adaptive_input, input.dt_s);
    if (!adaptive.valid) {
      return output;
    }
    candidate_disturbance_estimate = adaptive.filtered_uav_disturbance_estimate;
  } else {
    adaptive.uav_wrench_command = adaptive_input.nominal_uav_wrench;
    adaptive.arm_position_command_rad = nominal.arm_position_command_rad;
    adaptive.valid = true;
  }

  PhysicalWrench matched_correction = adaptive.uav_wrench_correction;
  matched_correction.force_b_n.x() = 0.0;
  matched_correction.force_b_n.y() = 0.0;
  PhysicalWrench requested_actuator_flu = nominal_actuator_flu;
  requested_actuator_flu.force_b_n += matched_correction.force_b_n;
  requested_actuator_flu.moment_b_nm += matched_correction.moment_b_nm;

  QuadrotorActuation requested = actuationFlu(requested_actuator_flu);
  if (previous_applied_actuation_flu_.has_value()) {
    const double maximum_collective_delta = collective_slew_n_s_ * input.dt_s;
    requested.thrust_up_n = previous_applied_actuation_flu_->thrust_up_n + std::clamp(
      requested.thrust_up_n - previous_applied_actuation_flu_->thrust_up_n,
      -maximum_collective_delta, maximum_collective_delta);
    requested.moment_b_nm = clampVectorRate(
      requested.moment_b_nm, previous_applied_actuation_flu_->moment_b_nm,
      moment_slew_nm_s_, input.dt_s);
  }

  const RotorAllocationModel rotor_model = rotorModel(mass_properties.center_of_mass_flu_m);
  const WrenchProjectionResult projected = projectPhysicalWrench(requested, rotor_model);
  if (!projected.valid || projected.projected_actuation.thrust_up_n <
    kMinimumAirborneCollectiveN)
  {
    return output;
  }

  const Px4NormalizedWrench normalized = rotorThrustToPx4Normalized(
    projected.rotor_thrust_n, thrust_model_, allocator_model_);
  if (!normalized.valid) {
    return output;
  }
  const QuadrotorActuation applied_actuation = QuadrotorActuation::fromVector(
    rotor_model.allocation_flu * normalized.realized_rotor_thrust_n);
  if (!applied_actuation.allFinite() ||
    applied_actuation.thrust_up_n < kMinimumAirborneCollectiveN)
  {
    return output;
  }
  const PhysicalWrench applied_actuator_flu = applied_actuation.physicalWrench();

  JointVector arm_command = adaptive.arm_position_command_rad;
  arm_command = arm_command.cwiseMin(kArmUpperRad).cwiseMax(kArmLowerRad);
  const JointVector maximum_arm_delta = kArmMaximumVelocityRadS.cwiseProduct(kArmServoTauS);
  arm_command = input.arm_position_rad +
    (arm_command - input.arm_position_rad).cwiseMin(maximum_arm_delta).cwiseMax(-maximum_arm_delta);

  if (adaptive_enabled_ && !candidate_adaptive_controller.applyAcceptedCommand(
      modeledBodyInputFlu(
        applied_actuator_flu, input, center_of_mass_velocity_ned_m_s, mass_properties),
      arm_command))
  {
    return output;
  }

  output.nominal_wrench_flu = nominal_actuator_flu;
  output.sample_timestamp_us = input.sample_timestamp_us;
  output.adaptive_wrench_flu = matched_correction;
  output.applied_wrench_flu = applied_actuator_flu;
  output.normalized_thrust_frd = normalized.thrust_frd;
  output.normalized_torque_frd = normalized.torque_frd;
  output.arm_position_command_rad = arm_command;
  output.rotor_saturated = normalized.saturated || projected.weighted_error_squared > 1.0e-12;
  output.feasible = true;

  pending_adaptive_controller_ = std::move(candidate_adaptive_controller);
  pending_disturbance_estimate_flu_ = candidate_disturbance_estimate;
  pending_previous_control_ = solver_input.previous_control;
  pending_previous_control_[0] = applied_actuation.thrust_up_n;
  pending_previous_control_.segment<3>(1) =
    kFluFromFrd * applied_actuation.moment_b_nm;
  pending_previous_control_.segment<kArmJointCount>(4) = arm_command;
  pending_slewed_target_ = candidate_slewed_target;
  pending_applied_actuation_flu_ = applied_actuation;
  return output;
}

bool FlyingHandController::acceptPendingUpdate() noexcept
{
  if (!pending_adaptive_controller_.has_value() || !pending_slewed_target_.has_value() ||
    !pending_applied_actuation_flu_.has_value())
  {
    return false;
  }
  adaptive_controller_ = std::move(*pending_adaptive_controller_);
  disturbance_estimate_flu_ = pending_disturbance_estimate_flu_;
  previous_control_ = pending_previous_control_;
  slewed_target_ = pending_slewed_target_;
  previous_applied_actuation_flu_ = pending_applied_actuation_flu_;
  rejectPendingUpdate();
  return true;
}

void FlyingHandController::rejectPendingUpdate() noexcept
{
  pending_adaptive_controller_.reset();
  pending_disturbance_estimate_flu_ = PhysicalWrench{};
  pending_previous_control_.setZero();
  pending_slewed_target_.reset();
  pending_applied_actuation_flu_.reset();
}

void FlyingHandController::recoverAfterRejectedUpdate() noexcept
{
  solver_.reset();
  adaptive_controller_.reset();
  disturbance_estimate_flu_ = PhysicalWrench{};
  rejectPendingUpdate();
}

void FlyingHandController::reset() noexcept
{
  solver_.reset();
  adaptive_controller_.reset();
  disturbance_estimate_flu_ = PhysicalWrench{};
  previous_control_.setZero();
  previous_control_[0] = kMassKg * kGravityMps2;
  slewed_target_.reset();
  previous_applied_actuation_flu_.reset();
  rejectPendingUpdate();
}

EndEffectorTarget FlyingHandController::slewTarget(
  const ControllerInput & input,
  std::optional<EndEffectorTarget> & target_state) const
{
  EndEffectorTarget desired;
  desired.position_ned_m = input.target_ee_pose_ned.translation();
  desired.attitude_ned = Eigen::Quaterniond(input.target_ee_pose_ned.linear()).normalized();
  if (!target_state.has_value()) {
    EndEffectorTarget current;
    current.position_ned_m = input.current_ee_pose_ned.translation();
    current.attitude_ned = Eigen::Quaterniond(input.current_ee_pose_ned.linear()).normalized();
    target_state = current;
  }

  const double maximum_position_delta = target_position_slew_m_s_ * input.dt_s;
  Eigen::Vector3d position_delta = desired.position_ned_m - target_state->position_ned_m;
  if (position_delta.norm() > maximum_position_delta) {
    position_delta *= maximum_position_delta / position_delta.norm();
  }
  target_state->position_ned_m += position_delta;

  Eigen::Quaterniond target_attitude = desired.attitude_ned;
  if (target_state->attitude_ned.dot(target_attitude) < 0.0) {
    target_attitude.coeffs() *= -1.0;
  }
  const double angular_distance = target_state->attitude_ned.angularDistance(target_attitude);
  const double interpolation = angular_distance > 1.0e-9 ?
    std::min(1.0, target_orientation_slew_rad_s_ * input.dt_s / angular_distance) : 1.0;
  target_state->attitude_ned =
    target_state->attitude_ned.slerp(interpolation, target_attitude).normalized();
  return *target_state;
}

}  // namespace flying_hand_mode::quadrotor
