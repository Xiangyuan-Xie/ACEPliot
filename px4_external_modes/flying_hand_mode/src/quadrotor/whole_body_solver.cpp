#include <flying_hand_mode/quadrotor/whole_body_solver.hpp>

#include <acados_solver_flying_hand_quadrotor.h>

#include <acados_c/ocp_nlp_interface.h>

#include <Eigen/Cholesky>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace flying_hand_mode::quadrotor
{
namespace
{

constexpr double kGravityMps2 = 9.80665;
constexpr std::array<double, kArmJointCount> kArmLowerRad{
  -2.6485, 0.0, -2.6485, -3.1415};
constexpr std::array<double, kArmJointCount> kArmUpperRad{
  2.6485, 3.1415, 2.6485, 3.1415};

bool quaternionIsFinite(const Eigen::Quaterniond & quaternion) noexcept
{
  return quaternion.coeffs().allFinite() && std::isfinite(quaternion.norm()) &&
         quaternion.norm() > 1.0e-9;
}

}  // namespace

WholeBodyStateVector WholeBodyState::vector() const noexcept
{
  WholeBodyStateVector value;
  value.segment<3>(0) = position_ned_m;
  const Eigen::Quaterniond normalized = attitude_ned_frd.normalized();
  value.segment<4>(3) << normalized.w(), normalized.x(), normalized.y(), normalized.z();
  value.segment<3>(7) = velocity_ned_m_s;
  value.segment<3>(10) = angular_velocity_frd_rad_s;
  value.segment<kArmJointCount>(13) = arm_position_rad;
  return value;
}

bool WholeBodyState::allFinite() const noexcept
{
  return position_ned_m.allFinite() && quaternionIsFinite(attitude_ned_frd) &&
         velocity_ned_m_s.allFinite() && angular_velocity_frd_rad_s.allFinite() &&
         arm_position_rad.allFinite();
}

bool WholeBodyMassProperties::allFinite() const noexcept
{
  return std::isfinite(mass_kg) && mass_kg > 0.0 && center_of_mass_flu_m.allFinite() &&
         inertia_com_flu_kg_m2.allFinite() &&
         inertia_com_flu_kg_m2.isApprox(inertia_com_flu_kg_m2.transpose(), 1.0e-9) &&
         Eigen::LLT<Eigen::Matrix3d>(inertia_com_flu_kg_m2).info() == Eigen::Success;
}

bool EndEffectorTarget::allFinite() const noexcept
{
  return position_ned_m.allFinite() && quaternionIsFinite(attitude_ned);
}

bool WholeBodyDisturbance::allFinite() const noexcept
{
  return force_frd_n.allFinite() && moment_frd_nm.allFinite() &&
         arm_velocity_rad_s.allFinite();
}

struct WholeBodySolver::Implementation
{
  flying_hand_quadrotor_solver_capsule * capsule{nullptr};
  bool initialized{false};

  Implementation()
  {
    capsule = flying_hand_quadrotor_acados_create_capsule();
    if (capsule == nullptr || flying_hand_quadrotor_acados_create(capsule) != 0) {
      if (capsule != nullptr) {
        flying_hand_quadrotor_acados_free_capsule(capsule);
      }
      throw std::runtime_error("Failed to create Flying Hand ACADOS solver");
    }
  }

  ~Implementation()
  {
    if (capsule != nullptr) {
      flying_hand_quadrotor_acados_free(capsule);
      flying_hand_quadrotor_acados_free_capsule(capsule);
    }
  }
};

WholeBodySolver::WholeBodySolver()
: implementation_(std::make_unique<Implementation>())
{
}

WholeBodySolver::~WholeBodySolver() = default;

WholeBodySolverOutput WholeBodySolver::solve(const WholeBodySolverInput & input) noexcept
{
  WholeBodySolverOutput output;
  if (!input.state.allFinite() || !input.target.allFinite() ||
    !input.mass_properties.allFinite() || !input.disturbance.allFinite() ||
    !input.previous_control.allFinite())
  {
    return output;
  }

  WholeBodyStateVector state = input.state.vector();
  for (int joint = 0; joint < kArmJointCount; ++joint) {
    state[13 + joint] = std::clamp(
      state[13 + joint], kArmLowerRad[static_cast<std::size_t>(joint)],
      kArmUpperRad[static_cast<std::size_t>(joint)]);
  }

  std::array<double, kWholeBodyParameterDimension> parameters{};
  const Eigen::Quaterniond target_attitude = input.target.attitude_ned.normalized();
  Eigen::Map<Eigen::Vector3d>(parameters.data()) = input.target.position_ned_m;
  parameters[3] = target_attitude.w();
  parameters[4] = target_attitude.x();
  parameters[5] = target_attitude.y();
  parameters[6] = target_attitude.z();
  Eigen::Map<Eigen::Vector3d>(parameters.data() + 7) = input.disturbance.force_frd_n;
  Eigen::Map<Eigen::Vector3d>(parameters.data() + 10) = input.disturbance.moment_frd_nm;
  Eigen::Map<JointVector>(parameters.data() + 13) = input.disturbance.arm_velocity_rad_s;
  Eigen::Map<WholeBodyControlVector>(parameters.data() + 17) = input.previous_control;
  Eigen::Map<Eigen::Vector3d>(parameters.data() + 25) =
    input.mass_properties.center_of_mass_flu_m;
  Eigen::Map<Eigen::Matrix3d>(parameters.data() + 28) =
    input.mass_properties.inertia_com_flu_kg_m2;

  flying_hand_quadrotor_solver_capsule * const capsule = implementation_->capsule;
  ocp_nlp_constraints_model_set(
    capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out,
    0, "lbx", state.data());
  ocp_nlp_constraints_model_set(
    capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, capsule->nlp_out,
    0, "ubx", state.data());
  for (int stage = 0; stage <= kWholeBodyHorizonNodes; ++stage) {
    if (flying_hand_quadrotor_acados_update_params(
        capsule, stage, parameters.data(), kWholeBodyParameterDimension) != 0)
    {
      return output;
    }
  }

  if (!implementation_->initialized) {
    WholeBodyControlVector hover_control = input.previous_control;
    if (hover_control.isZero(0.0)) {
      hover_control[0] = input.mass_properties.mass_kg * kGravityMps2;
      hover_control.segment<kArmJointCount>(4) = input.state.arm_position_rad;
    }
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
  output.solver_status = flying_hand_quadrotor_acados_solve(capsule);
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

  output.actuation_frd.thrust_up_n = control[0];
  output.actuation_frd.moment_b_nm = control.segment<3>(1);
  output.arm_position_command_rad = control.segment<kArmJointCount>(4);
  output.valid = output.actuation_frd.allFinite() &&
    output.arm_position_command_rad.allFinite();
  return output;
}

void WholeBodySolver::reset() noexcept
{
  if (implementation_ == nullptr || implementation_->capsule == nullptr) {
    return;
  }
  flying_hand_quadrotor_acados_reset(implementation_->capsule, 1, 1, 0, 0);
  implementation_->initialized = false;
}

}  // namespace flying_hand_mode::quadrotor
