#include <flying_hand_fully_actuated_mode/whole_body_solver.hpp>

#include <acados_solver_flying_hand_fully_actuated.h>

#include <acados_c/ocp_nlp_interface.h>

#include <Eigen/Cholesky>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace flying_hand_fully_actuated_mode
{
namespace
{

constexpr double kGravityMps2 = 9.80665;

bool finiteQuaternion(const Eigen::Quaterniond & quaternion) noexcept
{
  return quaternion.coeffs().allFinite() && std::isfinite(quaternion.norm()) &&
         quaternion.norm() > 1.0e-9;
}

}  // namespace

bool FullyActuatedModelConfig::allFinite() const noexcept
{
  return std::isfinite(mass_kg) && mass_kg > 0.0 && inertia_frd_kg_m2.allFinite() &&
         inertia_frd_kg_m2.isApprox(inertia_frd_kg_m2.transpose(), 1.0e-9) &&
         Eigen::LLT<Eigen::Matrix3d>(inertia_frd_kg_m2).info() == Eigen::Success &&
         arm_kinematics.allFinite() && arm_servo_delay_s.allFinite() &&
         arm_lower_rad.allFinite() && arm_upper_rad.allFinite() &&
         arm_max_velocity_rad_s.allFinite() && arm_home_rad.allFinite() &&
         maximum_linear_velocity_m_s.allFinite() &&
         maximum_angular_velocity_rad_s.allFinite() && allocation.valid &&
         (arm_servo_delay_s.array() > 0.0).all() &&
         (arm_lower_rad.array() < arm_upper_rad.array()).all() &&
         (arm_home_rad.array() >= arm_lower_rad.array()).all() &&
         (arm_home_rad.array() <= arm_upper_rad.array()).all() &&
         (arm_max_velocity_rad_s.array() > 0.0).all() &&
         (maximum_linear_velocity_m_s.array() > 0.0).all() &&
         (maximum_angular_velocity_rad_s.array() > 0.0).all();
}

WholeBodyStateVector WholeBodyState::vector() const noexcept
{
  WholeBodyStateVector value;
  value.segment<3>(0) = position_ned_m;
  const Eigen::Quaterniond normalized = attitude_ned_frd.normalized();
  value.segment<4>(3) << normalized.w(), normalized.x(), normalized.y(), normalized.z();
  value.segment<3>(7) = velocity_ned_m_s;
  value.segment<3>(10) = angular_velocity_frd_rad_s;
  value.segment<4>(13) = arm_position_rad;
  return value;
}

bool WholeBodyState::allFinite() const noexcept
{
  return position_ned_m.allFinite() && finiteQuaternion(attitude_ned_frd) &&
         velocity_ned_m_s.allFinite() && angular_velocity_frd_rad_s.allFinite() &&
         arm_position_rad.allFinite();
}

bool EndEffectorTarget::allFinite() const noexcept
{
  return position_ned_m.allFinite() && finiteQuaternion(attitude_ned);
}

struct WholeBodySolver::Implementation
{
  flying_hand_fully_actuated_solver_capsule * capsule{nullptr};
  bool initialized{false};

  Implementation()
  {
    capsule = flying_hand_fully_actuated_acados_create_capsule();
    if (capsule == nullptr || flying_hand_fully_actuated_acados_create(capsule) != 0) {
      if (capsule != nullptr) {
        flying_hand_fully_actuated_acados_free_capsule(capsule);
      }
      throw std::runtime_error("Failed to create fully actuated Flying Hand ACADOS solver");
    }
  }

  ~Implementation()
  {
    if (capsule != nullptr) {
      flying_hand_fully_actuated_acados_free(capsule);
      flying_hand_fully_actuated_acados_free_capsule(capsule);
    }
  }
};

WholeBodySolver::WholeBodySolver(FullyActuatedModelConfig config)
: config_(std::move(config)),
  implementation_(std::make_unique<Implementation>())
{
  if (!config_.allFinite()) {
    throw std::invalid_argument("Invalid fully actuated Flying Hand model configuration");
  }
}

WholeBodySolver::~WholeBodySolver() = default;

WholeBodySolverOutput WholeBodySolver::solve(const WholeBodySolverInput & input) noexcept
{
  WholeBodySolverOutput output;
  if (!input.state.allFinite() || !input.target.allFinite() || !config_.allFinite()) {
    return output;
  }

  WholeBodyStateVector state = input.state.vector();
  for (int joint = 0; joint < flying_hand_control_common::kArmJointCount; ++joint) {
    state[13 + joint] = std::clamp(
      state[13 + joint], config_.arm_lower_rad[joint], config_.arm_upper_rad[joint]);
  }

  std::array<double, kWholeBodyParameterDimension> parameters{};
  const Eigen::Quaterniond target_attitude = input.target.attitude_ned.normalized();
  Eigen::Map<Eigen::Vector3d>(parameters.data()) = input.target.position_ned_m;
  parameters[3] = target_attitude.w();
  parameters[4] = target_attitude.x();
  parameters[5] = target_attitude.y();
  parameters[6] = target_attitude.z();
  parameters[7] = config_.mass_kg;
  Eigen::Map<Eigen::Matrix3d>(parameters.data() + 8) = config_.inertia_frd_kg_m2;
  Eigen::Map<JointVector>(parameters.data() + 17) = config_.arm_kinematics.d_m;
  Eigen::Map<JointVector>(parameters.data() + 21) = config_.arm_kinematics.a_m;
  Eigen::Map<JointVector>(parameters.data() + 25) = config_.arm_kinematics.alpha_rad;
  Eigen::Map<JointVector>(parameters.data() + 29) =
    config_.arm_kinematics.theta_offset_rad;
  Eigen::Map<Eigen::Vector3d>(parameters.data() + 33) =
    config_.arm_kinematics.body_from_arm_base_flu.translation();
  Eigen::Quaterniond base_attitude(
    config_.arm_kinematics.body_from_arm_base_flu.linear());
  base_attitude.normalize();
  parameters[36] = base_attitude.w();
  parameters[37] = base_attitude.x();
  parameters[38] = base_attitude.y();
  parameters[39] = base_attitude.z();
  Eigen::Map<JointVector>(parameters.data() + 40) = config_.arm_servo_delay_s;
  Eigen::Map<AllocationMatrix>(parameters.data() + 44) =
    config_.allocation.physical_inverse_frd;
  Eigen::Map<JointVector>(parameters.data() + 80) = config_.arm_home_rad;

  flying_hand_fully_actuated_solver_capsule * const capsule = implementation_->capsule;
  ocp_nlp_constraints_model_set(
    capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out,
    0, "lbx", state.data());
  ocp_nlp_constraints_model_set(
    capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out,
    0, "ubx", state.data());

  std::array<double, 10> lower_state_bounds{};
  std::array<double, 10> upper_state_bounds{};
  for (int axis = 0; axis < 3; ++axis) {
    lower_state_bounds[static_cast<std::size_t>(axis)] =
      -config_.maximum_linear_velocity_m_s[axis];
    upper_state_bounds[static_cast<std::size_t>(axis)] =
      config_.maximum_linear_velocity_m_s[axis];
    lower_state_bounds[static_cast<std::size_t>(3 + axis)] =
      -config_.maximum_angular_velocity_rad_s[axis];
    upper_state_bounds[static_cast<std::size_t>(3 + axis)] =
      config_.maximum_angular_velocity_rad_s[axis];
  }
  for (int joint = 0; joint < flying_hand_control_common::kArmJointCount; ++joint) {
    lower_state_bounds[static_cast<std::size_t>(6 + joint)] = config_.arm_lower_rad[joint];
    upper_state_bounds[static_cast<std::size_t>(6 + joint)] = config_.arm_upper_rad[joint];
  }

  std::array<double, 10> lower_path_bounds{};
  std::array<double, 10> upper_path_bounds{};
  for (int rotor = 0; rotor < kRotorCount; ++rotor) {
    lower_path_bounds[static_cast<std::size_t>(rotor)] =
      config_.allocation.minimum_thrust_n[rotor];
    upper_path_bounds[static_cast<std::size_t>(rotor)] =
      config_.allocation.maximum_thrust_n[rotor];
  }
  for (int joint = 0; joint < flying_hand_control_common::kArmJointCount; ++joint) {
    lower_path_bounds[static_cast<std::size_t>(6 + joint)] =
      -config_.arm_max_velocity_rad_s[joint];
    upper_path_bounds[static_cast<std::size_t>(6 + joint)] =
      config_.arm_max_velocity_rad_s[joint];
  }

  for (int stage = 0; stage <= kWholeBodyHorizonNodes; ++stage) {
    if (flying_hand_fully_actuated_acados_update_params(
        capsule, stage, parameters.data(), kWholeBodyParameterDimension) != 0)
    {
      return output;
    }
    if (stage > 0) {
      ocp_nlp_constraints_model_set(
        capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out,
        stage, "lbx", lower_state_bounds.data());
      ocp_nlp_constraints_model_set(
        capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out,
        stage, "ubx", upper_state_bounds.data());
    }
    if (stage < kWholeBodyHorizonNodes) {
      ocp_nlp_constraints_model_set(
        capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out,
        stage, "lbu", config_.arm_lower_rad.data());
      ocp_nlp_constraints_model_set(
        capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out,
        stage, "ubu", config_.arm_upper_rad.data());
      ocp_nlp_constraints_model_set(
        capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out,
        stage, "lh", lower_path_bounds.data());
      ocp_nlp_constraints_model_set(
        capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out,
        stage, "uh", upper_path_bounds.data());
    }
  }

  if (!implementation_->initialized) {
    WholeBodyControlVector hover_control = WholeBodyControlVector::Zero();
    const Eigen::Vector3d hover_force_ned(0.0, 0.0, -config_.mass_kg * kGravityMps2);
    hover_control.head<3>() =
      input.state.attitude_ned_frd.conjugate() * hover_force_ned;
    hover_control.tail<4>() = input.state.arm_position_rad;
    for (int stage = 0; stage < kWholeBodyHorizonNodes; ++stage) {
      ocp_nlp_out_set(
        capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, capsule->nlp_in,
        stage, "x", state.data());
      ocp_nlp_out_set(
        capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, capsule->nlp_in,
        stage, "u", hover_control.data());
    }
    ocp_nlp_out_set(
      capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, capsule->nlp_in,
      kWholeBodyHorizonNodes, "x", state.data());
    implementation_->initialized = true;
  }

  const auto solve_start = std::chrono::steady_clock::now();
  output.solver_status = flying_hand_fully_actuated_acados_solve(capsule);
  const auto solve_end = std::chrono::steady_clock::now();
  output.solver_time_s = std::chrono::duration<double>(solve_end - solve_start).count();
  if (output.solver_status != 0) {
    return output;
  }

  WholeBodyControlVector control;
  ocp_nlp_out_get(
    capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out, 0, "u", control.data());
  if (!control.allFinite()) {
    return output;
  }
  ocp_nlp_eval_cost(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);
  ocp_nlp_get(capsule->nlp_solver, "cost_value", &output.tracking_cost);

  output.wrench_frd = control.head<6>();
  output.arm_position_command_rad = control.tail<4>();
  output.valid = output.wrench_frd.allFinite() &&
    output.arm_position_command_rad.allFinite() && std::isfinite(output.tracking_cost);
  return output;
}

void WholeBodySolver::reset() noexcept
{
  if (implementation_ == nullptr || implementation_->capsule == nullptr) {
    return;
  }
  flying_hand_fully_actuated_acados_reset(implementation_->capsule, 1, 1, 0, 0);
  implementation_->initialized = false;
}

const FullyActuatedModelConfig & WholeBodySolver::config() const noexcept
{
  return config_;
}

}  // namespace flying_hand_fully_actuated_mode
