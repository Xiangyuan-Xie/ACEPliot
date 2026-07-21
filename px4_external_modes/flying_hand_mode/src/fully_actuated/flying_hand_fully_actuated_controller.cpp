#include <flying_hand_mode/fully_actuated/flying_hand_fully_actuated_controller.hpp>

#include <flying_hand_mode/fully_actuated/dh_kinematics.hpp>
#include <flying_hand_mode/fully_actuated/fully_actuated_core.hpp>
#include <flying_hand_mode/fully_actuated/paper_l1_adaptive.hpp>
#include <flying_hand_mode/fully_actuated/whole_body_solver.hpp>

#include <Eigen/Cholesky>

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace flying_hand_mode::fully_actuated
{
namespace
{

using flying_hand_mode::runtime::ControllerInput;
using flying_hand_mode::runtime::ControllerOutput;

constexpr double kGravityMps2 = 9.80665;

struct ControllerConfiguration
{
  FullyActuatedModelConfig model;
  RotorThrustCommandModel thrust_model;
  PaperL1AdaptiveConfig adaptive;
  bool adaptive_enabled{true};
  double target_position_slew_m_s{0.25};
  double target_orientation_slew_rad_s{0.4};
  Eigen::Vector3d force_slew_n_s{Eigen::Vector3d::Zero()};
  Eigen::Vector3d moment_slew_nm_s{Eigen::Vector3d::Zero()};
};

std::vector<double> vectorParameter(
  rclcpp::Node & node, const std::string & name,
  const std::vector<double> & default_value, std::size_t expected_size)
{
  const std::vector<double> value =
    node.declare_parameter<std::vector<double>>(name, default_value);
  if (value.size() != expected_size ||
    !std::all_of(
      value.begin(), value.end(), [](double element) {return std::isfinite(element);}))
  {
    throw std::invalid_argument(
            "Parameter '" + name + "' must contain " +
            std::to_string(expected_size) + " finite values");
  }
  return value;
}

template<int Size>
Eigen::Matrix<double, Size, 1> fixedVector(
  rclcpp::Node & node, const std::string & name,
  const std::vector<double> & default_value)
{
  const std::vector<double> value = vectorParameter(node, name, default_value, Size);
  return Eigen::Map<const Eigen::Matrix<double, Size, 1>>(value.data());
}

RotorGeometry loadRotorGeometry(rclcpp::Node & node)
{
  const std::vector<double> default_positions{
    0.34, 0.0, 0.0,
    0.17, 0.2944486373, 0.0,
    -0.17, 0.2944486373, 0.0,
    -0.34, 0.0, 0.0,
    -0.17, -0.2944486373, 0.0,
    0.17, -0.2944486373, 0.0};
  const std::vector<double> default_axes{
    0.0, 0.3420201433, -0.9396926208,
    0.2961981327, -0.1710100717, -0.9396926208,
    -0.2961981327, -0.1710100717, -0.9396926208,
    0.0, 0.3420201433, -0.9396926208,
    0.2961981327, -0.1710100717, -0.9396926208,
    -0.2961981327, -0.1710100717, -0.9396926208};
  const std::vector<double> positions =
    vectorParameter(node, "rotor.position_frd_m", default_positions, 18);
  const std::vector<double> axes =
    vectorParameter(node, "rotor.axis_frd", default_axes, 18);

  RotorGeometry geometry;
  for (int rotor = 0; rotor < kRotorCount; ++rotor) {
    for (int axis = 0; axis < 3; ++axis) {
      const std::size_t index = static_cast<std::size_t>(rotor * 3 + axis);
      geometry.position_frd_m(axis, rotor) = positions[index];
      geometry.axis_frd(axis, rotor) = axes[index];
    }
  }
  geometry.moment_ratio_m = fixedVector<kRotorCount>(
    node, "rotor.moment_ratio_m", {0.02, -0.02, 0.02, -0.02, 0.02, -0.02});
  geometry.minimum_thrust_n = fixedVector<kRotorCount>(
    node, "rotor.minimum_thrust_n", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  geometry.maximum_thrust_n = fixedVector<kRotorCount>(
    node, "rotor.maximum_thrust_n", {20.0, 20.0, 20.0, 20.0, 20.0, 20.0});
  return geometry;
}

DhKinematicsConfig loadKinematics(rclcpp::Node & node)
{
  DhKinematicsConfig config;
  config.d_m = fixedVector<4>(
    node, "arm.dh_d_m", {0.0, 0.050, 0.0, 0.076});
  config.a_m = fixedVector<4>(
    node, "arm.dh_a_m", {0.363, 0.441, 0.007, 0.200});
  config.alpha_rad = fixedVector<4>(
    node, "arm.dh_alpha_rad", {0.10, -0.10, -1.578, 0.0});
  config.theta_offset_rad = fixedVector<4>(
    node, "arm.dh_theta_offset_rad", {0.0, 0.0, 0.0, 0.0});
  const Eigen::Vector3d translation = fixedVector<3>(
    node, "arm.base_position_flu_m", {0.0, 0.0, 0.0});
  const Eigen::Vector3d rpy = fixedVector<3>(
    node, "arm.base_rpy_flu_rad", {0.0, 0.0, 0.0});
  config.body_from_arm_base_flu = Eigen::Isometry3d::Identity();
  config.body_from_arm_base_flu.translation() = translation;
  config.body_from_arm_base_flu.linear() =
    (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())).toRotationMatrix();
  return config;
}

ControllerConfiguration loadConfiguration(rclcpp::Node & node)
{
  ControllerConfiguration configuration;
  configuration.model.mass_kg = node.declare_parameter<double>("uav.mass_kg", 5.0);
  const std::vector<double> inertia = vectorParameter(
    node, "uav.inertia_frd_kg_m2",
    {0.10, 0.0, 0.0, 0.0, 0.10, 0.0, 0.0, 0.0, 0.15}, 9);
  configuration.model.inertia_frd_kg_m2 =
    Eigen::Map<const Eigen::Matrix3d>(inertia.data());
  configuration.model.arm_kinematics = loadKinematics(node);
  configuration.model.arm_servo_delay_s = fixedVector<4>(
    node, "arm.servo_delay_s", {0.66, 0.68, 0.81, 0.85});
  configuration.model.arm_lower_rad = fixedVector<4>(
    node, "arm.joint_lower_rad", {-2.6, -2.6, -2.6, -3.1415926536});
  configuration.model.arm_upper_rad = fixedVector<4>(
    node, "arm.joint_upper_rad", {2.6, 2.6, 2.6, 3.1415926536});
  configuration.model.arm_max_velocity_rad_s = fixedVector<4>(
    node, "arm.max_velocity_rad_s", {2.0, 2.0, 2.0, 3.0});
  configuration.model.arm_home_rad = fixedVector<4>(
    node, "arm.home_rad", {0.0, 0.0, 0.0, 0.0});
  configuration.model.maximum_linear_velocity_m_s = fixedVector<3>(
    node, "uav.maximum_linear_velocity_m_s", {5.0, 5.0, 5.0});
  configuration.model.maximum_angular_velocity_rad_s = fixedVector<3>(
    node, "uav.maximum_angular_velocity_rad_s", {3.0, 3.0, 3.0});

  const RotorGeometry geometry = loadRotorGeometry(node);
  const double condition_limit =
    node.declare_parameter<double>("allocation.maximum_condition_number", 50.0);
  configuration.model.allocation = buildAllocationModel(geometry, condition_limit);
  if (!configuration.model.allocation.valid) {
    throw std::invalid_argument(
            "Tilted-hex rotor geometry must form a finite, full-rank 6x6 allocation matrix");
  }
  configuration.thrust_model.maximum_thrust_n = geometry.maximum_thrust_n;
  configuration.thrust_model.kappa = fixedVector<kRotorCount>(
    node, "rotor.thrust_curve_kappa", {0.3, 0.3, 0.3, 0.3, 0.3, 0.3});

  configuration.adaptive.arm_servo_delay_s = configuration.model.arm_servo_delay_s;
  configuration.adaptive.uav_predictor_rate_rad_s = fixedVector<6>(
    node, "l1.uav_predictor_rate_rad_s", {8.0, 8.0, 8.0, 12.0, 12.0, 12.0});
  configuration.adaptive.uav_low_pass_cutoff_hz = fixedVector<6>(
    node, "l1.uav_low_pass_cutoff_hz", {1.5, 1.5, 1.5, 1.0, 1.0, 1.0});
  configuration.adaptive.uav_correction_limit = fixedVector<6>(
    node, "l1.uav_correction_limit", {10.0, 10.0, 10.0, 2.0, 2.0, 2.0});
  configuration.adaptive.arm_predictor_rate_rad_s = fixedVector<4>(
    node, "l1.arm_predictor_rate_rad_s", {10.0, 10.0, 10.0, 10.0});
  configuration.adaptive.arm_low_pass_cutoff_hz = fixedVector<4>(
    node, "l1.arm_low_pass_cutoff_hz", {1.0, 1.0, 1.0, 1.0});
  configuration.adaptive.arm_correction_limit_rad = fixedVector<4>(
    node, "l1.arm_correction_limit_rad", {0.15, 0.15, 0.15, 0.15});

  configuration.adaptive_enabled = node.declare_parameter<bool>("adaptive_enabled", true);
  configuration.target_position_slew_m_s =
    node.declare_parameter<double>("target_position_slew_m_s", 0.25);
  configuration.target_orientation_slew_rad_s =
    node.declare_parameter<double>("target_orientation_slew_rad_s", 0.4);
  configuration.force_slew_n_s = fixedVector<3>(
    node, "force_slew_n_s", {40.0, 40.0, 100.0});
  configuration.moment_slew_nm_s = fixedVector<3>(
    node, "moment_slew_nm_s", {10.0, 10.0, 5.0});

  if (!configuration.model.allFinite() ||
    !PaperL1AdaptiveController::isValidConfig(configuration.adaptive) ||
    !configuration.thrust_model.maximum_thrust_n.allFinite() ||
    !configuration.thrust_model.kappa.allFinite() ||
    (configuration.thrust_model.maximum_thrust_n.array() <= 0.0).any() ||
    (configuration.thrust_model.kappa.array() < 0.0).any() ||
    (configuration.thrust_model.kappa.array() > 1.0).any() ||
    !std::isfinite(configuration.target_position_slew_m_s) ||
    configuration.target_position_slew_m_s <= 0.0 ||
    !std::isfinite(configuration.target_orientation_slew_rad_s) ||
    configuration.target_orientation_slew_rad_s <= 0.0 ||
    !configuration.force_slew_n_s.allFinite() ||
    !configuration.moment_slew_nm_s.allFinite() ||
    (configuration.force_slew_n_s.array() <= 0.0).any() ||
    (configuration.moment_slew_nm_s.array() <= 0.0).any())
  {
    throw std::invalid_argument("Invalid fully actuated Flying Hand controller parameters");
  }
  return configuration;
}

Eigen::Vector3d clampRate(
  const Eigen::Vector3d & desired, const Eigen::Vector3d & previous,
  const Eigen::Vector3d & rate, double dt_s)
{
  const Eigen::Vector3d maximum_delta = rate * dt_s;
  return previous + (desired - previous).cwiseMin(maximum_delta).cwiseMax(-maximum_delta);
}

}  // namespace

struct FlyingHandFullyActuatedController::Implementation
{
  explicit Implementation(rclcpp::Node & node)
  : configuration(loadConfiguration(node)),
    kinematics(configuration.model.arm_kinematics),
    solver(configuration.model),
    adaptive_controller(configuration.adaptive)
  {
  }

  EndEffectorTarget slewTarget(
    const ControllerInput & input, std::optional<EndEffectorTarget> & target_state) const
  {
    EndEffectorTarget desired;
    desired.position_ned_m = input.target_ee_pose_ned.translation();
    desired.attitude_ned = Eigen::Quaterniond(input.target_ee_pose_ned.linear()).normalized();
    if (!target_state.has_value()) {
      EndEffectorTarget current;
      current.position_ned_m = input.current_ee_pose_ned.translation();
      current.attitude_ned =
        Eigen::Quaterniond(input.current_ee_pose_ned.linear()).normalized();
      target_state = current;
    }

    const double maximum_position_delta =
      configuration.target_position_slew_m_s * input.dt_s;
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
      std::min(
      1.0, configuration.target_orientation_slew_rad_s * input.dt_s / angular_distance) :
      1.0;
    target_state->attitude_ned =
      target_state->attitude_ned.slerp(interpolation, target_attitude).normalized();
    return *target_state;
  }

  ControllerOutput update(const ControllerInput & input)
  {
    ControllerOutput output;
    if (pending_adaptive_controller.has_value() || !std::isfinite(input.dt_s) ||
      input.dt_s <= 0.0 || input.dt_s > configuration.adaptive.maximum_dt_s ||
      !input.position_ned_m.allFinite() || !input.attitude_ned_frd.coeffs().allFinite() ||
      !input.linear_velocity_ned_m_s.allFinite() ||
      !input.angular_velocity_frd_rad_s.allFinite() || !input.gyro_frd_rad_s.allFinite() ||
      !input.arm_position_rad.allFinite())
    {
      return output;
    }

    WholeBodySolverInput solver_input;
    solver_input.state.position_ned_m = input.position_ned_m;
    solver_input.state.attitude_ned_frd = input.attitude_ned_frd;
    solver_input.state.velocity_ned_m_s = input.linear_velocity_ned_m_s;
    solver_input.state.angular_velocity_frd_rad_s = input.angular_velocity_frd_rad_s;
    solver_input.state.arm_position_rad = input.arm_position_rad;
    std::optional<EndEffectorTarget> candidate_target = slewed_target;
    solver_input.target = slewTarget(input, candidate_target);

    const WholeBodySolverOutput nominal = solver.solve(solver_input);
    if (!nominal.valid) {
      return output;
    }

    PaperL1AdaptiveInput adaptive_input;
    adaptive_input.measured_body_velocity.head<3>() =
      input.attitude_ned_frd.conjugate() * input.linear_velocity_ned_m_s;
    adaptive_input.measured_body_velocity.tail<3>() = input.gyro_frd_rad_s;
    adaptive_input.nominal_wrench_frd = nominal.wrench_frd;
    adaptive_input.generalized_mass_diag.head<3>().setConstant(configuration.model.mass_kg);
    adaptive_input.generalized_mass_diag.tail<3>() =
      configuration.model.inertia_frd_kg_m2.diagonal();
    const Eigen::Vector3d gravity_ned(0.0, 0.0, kGravityMps2);
    adaptive_input.modeled_drift_acceleration.head<3>() =
      input.attitude_ned_frd.conjugate() * gravity_ned -
      input.gyro_frd_rad_s.cross(adaptive_input.measured_body_velocity.head<3>());
    const Eigen::Vector3d angular_momentum =
      configuration.model.inertia_frd_kg_m2 * input.gyro_frd_rad_s;
    adaptive_input.modeled_drift_acceleration.tail<3>() =
      -configuration.model.inertia_frd_kg_m2.inverse() *
      input.gyro_frd_rad_s.cross(angular_momentum);
    adaptive_input.measured_arm_position_rad = input.arm_position_rad;
    adaptive_input.nominal_arm_position_command_rad = nominal.arm_position_command_rad;

    PaperL1AdaptiveController candidate_adaptive = adaptive_controller;
    PaperL1AdaptiveOutput adaptive;
    if (configuration.adaptive_enabled) {
      adaptive = candidate_adaptive.update(adaptive_input, input.dt_s);
      if (!adaptive.valid) {
        return output;
      }
    } else {
      adaptive.wrench_command_frd = nominal.wrench_frd;
      adaptive.arm_position_command_rad = nominal.arm_position_command_rad;
      adaptive.valid = true;
    }

    WrenchVector requested_wrench = adaptive.wrench_command_frd;
    if (previous_applied_wrench_frd.has_value()) {
      requested_wrench.head<3>() = clampRate(
        requested_wrench.head<3>(), previous_applied_wrench_frd->head<3>(),
        configuration.force_slew_n_s, input.dt_s);
      requested_wrench.tail<3>() = clampRate(
        requested_wrench.tail<3>(), previous_applied_wrench_frd->tail<3>(),
        configuration.moment_slew_nm_s, input.dt_s);
    }

    const WrenchProjectionResult projected = projectWrenchToRotorBox(
      requested_wrench, configuration.model.allocation);
    if (!projected.valid) {
      return output;
    }
    const Px4NormalizedWrench normalized = rotorThrustToPx4Normalized(
      projected.rotor_thrust_n, configuration.thrust_model,
      configuration.model.allocation);
    if (!normalized.valid) {
      return output;
    }
    const WrenchVector applied_wrench =
      configuration.model.allocation.physical_allocation_frd *
      normalized.realized_rotor_thrust_n;
    const Eigen::Vector3d applied_force_ned =
      input.attitude_ned_frd * applied_wrench.head<3>();
    if (!applied_wrench.allFinite() ||
      -applied_force_ned.z() < 0.1 * configuration.model.mass_kg * kGravityMps2)
    {
      return output;
    }

    JointVector arm_command = adaptive.arm_position_command_rad;
    arm_command = arm_command.cwiseMin(configuration.model.arm_upper_rad)
      .cwiseMax(configuration.model.arm_lower_rad);
    const JointVector maximum_arm_delta =
      configuration.model.arm_max_velocity_rad_s * input.dt_s;
    arm_command = input.arm_position_rad +
      (arm_command - input.arm_position_rad).cwiseMin(maximum_arm_delta)
      .cwiseMax(-maximum_arm_delta);

    if (configuration.adaptive_enabled && !candidate_adaptive.applyAcceptedCommand(
        applied_wrench, arm_command))
    {
      return output;
    }

    output.sample_timestamp_us = input.sample_timestamp_us;
    output.nominal_wrench_flu = frdWrenchToFlu(nominal.wrench_frd);
    output.adaptive_wrench_flu = frdWrenchToFlu(adaptive.wrench_correction_frd);
    output.applied_wrench_flu = frdWrenchToFlu(applied_wrench);
    output.normalized_thrust_frd = normalized.thrust_frd;
    output.normalized_torque_frd = normalized.torque_frd;
    output.arm_position_command_rad = arm_command;
    output.tracking_cost = nominal.tracking_cost;
    output.allocation_condition_number = configuration.model.allocation.condition_number;
    output.rotor_saturated = projected.saturated;
    output.feasible = true;

    pending_adaptive_controller = std::move(candidate_adaptive);
    pending_slewed_target = candidate_target;
    pending_applied_wrench_frd = applied_wrench;
    return output;
  }

  bool acceptPendingUpdate() noexcept
  {
    if (!pending_adaptive_controller.has_value() || !pending_slewed_target.has_value() ||
      !pending_applied_wrench_frd.has_value())
    {
      return false;
    }
    adaptive_controller = std::move(*pending_adaptive_controller);
    slewed_target = pending_slewed_target;
    previous_applied_wrench_frd = pending_applied_wrench_frd;
    rejectPendingUpdate();
    return true;
  }

  void rejectPendingUpdate() noexcept
  {
    pending_adaptive_controller.reset();
    pending_slewed_target.reset();
    pending_applied_wrench_frd.reset();
  }

  void recoverAfterRejectedUpdate() noexcept
  {
    solver.reset();
    adaptive_controller.reset();
    previous_applied_wrench_frd.reset();
    rejectPendingUpdate();
  }

  void reset() noexcept
  {
    solver.reset();
    adaptive_controller.reset();
    slewed_target.reset();
    previous_applied_wrench_frd.reset();
    rejectPendingUpdate();
  }

  const ControllerConfiguration configuration;
  const DhKinematics kinematics;
  WholeBodySolver solver;
  PaperL1AdaptiveController adaptive_controller;
  std::optional<EndEffectorTarget> slewed_target;
  std::optional<WrenchVector> previous_applied_wrench_frd;
  std::optional<PaperL1AdaptiveController> pending_adaptive_controller;
  std::optional<EndEffectorTarget> pending_slewed_target;
  std::optional<WrenchVector> pending_applied_wrench_frd;
};

FlyingHandFullyActuatedController::FlyingHandFullyActuatedController(rclcpp::Node & node)
: implementation_(std::make_unique<Implementation>(node))
{
}

FlyingHandFullyActuatedController::~FlyingHandFullyActuatedController() = default;

ControllerOutput FlyingHandFullyActuatedController::update(const ControllerInput & input)
{
  return implementation_->update(input);
}

bool FlyingHandFullyActuatedController::acceptPendingUpdate() noexcept
{
  return implementation_->acceptPendingUpdate();
}

void FlyingHandFullyActuatedController::rejectPendingUpdate() noexcept
{
  implementation_->rejectPendingUpdate();
}

void FlyingHandFullyActuatedController::recoverAfterRejectedUpdate() noexcept
{
  implementation_->recoverAfterRejectedUpdate();
}

void FlyingHandFullyActuatedController::reset() noexcept
{
  implementation_->reset();
}

Eigen::Isometry3d FlyingHandFullyActuatedController::endEffectorPoseFlu(
  const JointVector & joint_position_rad) const noexcept
{
  return implementation_->kinematics.endEffectorPoseFlu(joint_position_rad);
}

}  // namespace flying_hand_mode::fully_actuated
