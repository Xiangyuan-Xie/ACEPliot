#pragma once

#include <flying_hand_mode/runtime/controller_types.hpp>

#include <Eigen/Core>

#include <array>

namespace flying_hand_mode::fully_actuated
{

constexpr int kRotorCount = 6;
constexpr int kActuationDimension = 6;

using flying_hand_mode::runtime::PhysicalWrench;
using flying_hand_mode::runtime::WrenchVector;
using RotorVector = Eigen::Matrix<double, kRotorCount, 1>;
using AllocationMatrix = Eigen::Matrix<double, kActuationDimension, kRotorCount>;

// Rotor geometry follows PX4 FRD conventions. Each axis points in the generated
// thrust direction and each moment ratio uses PX4's signed CA_ROTORx_KM convention.
struct RotorGeometry
{
  Eigen::Matrix<double, 3, kRotorCount> position_frd_m{
    Eigen::Matrix<double, 3, kRotorCount>::Zero()};
  Eigen::Matrix<double, 3, kRotorCount> axis_frd{
    Eigen::Matrix<double, 3, kRotorCount>::Zero()};
  RotorVector moment_ratio_m{RotorVector::Zero()};
  RotorVector minimum_thrust_n{RotorVector::Zero()};
  RotorVector maximum_thrust_n{RotorVector::Zero()};
};

struct FullyActuatedAllocationModel
{
  // Maps rotor thrust [N] to [Fx, Fy, Fz, Mx, My, Mz] in body FRD.
  AllocationMatrix physical_allocation_frd{AllocationMatrix::Zero()};
  AllocationMatrix physical_inverse_frd{AllocationMatrix::Zero()};
  // Maps normalized PX4 [Mx, My, Mz, Fx, Fy, Fz] to motor command.
  AllocationMatrix px4_actuator_from_control{AllocationMatrix::Zero()};
  AllocationMatrix px4_control_from_actuator{AllocationMatrix::Zero()};
  RotorVector minimum_thrust_n{RotorVector::Zero()};
  RotorVector maximum_thrust_n{RotorVector::Zero()};
  WrenchVector wrench_weights{WrenchVector::Ones()};
  double condition_number{0.0};
  bool valid{false};
};

FullyActuatedAllocationModel buildAllocationModel(
  const RotorGeometry & geometry, double maximum_condition_number) noexcept;

struct WrenchProjectionResult
{
  WrenchVector requested_wrench_frd{WrenchVector::Zero()};
  WrenchVector projected_wrench_frd{WrenchVector::Zero()};
  RotorVector rotor_thrust_n{RotorVector::Zero()};
  double weighted_error_squared{0.0};
  bool saturated{false};
  bool valid{false};
};

WrenchProjectionResult projectWrenchToRotorBox(
  const WrenchVector & desired_wrench_frd,
  const FullyActuatedAllocationModel & model) noexcept;

struct RotorThrustCommandModel
{
  RotorVector maximum_thrust_n{RotorVector::Zero()};
  RotorVector kappa{RotorVector::Zero()};
};

struct Px4NormalizedWrench
{
  Eigen::Vector3f thrust_frd{Eigen::Vector3f::Zero()};
  Eigen::Vector3f torque_frd{Eigen::Vector3f::Zero()};
  RotorVector actuator_command{RotorVector::Zero()};
  RotorVector realized_rotor_thrust_n{RotorVector::Zero()};
  bool valid{false};
};

Px4NormalizedWrench rotorThrustToPx4Normalized(
  const RotorVector & rotor_thrust_n,
  const RotorThrustCommandModel & thrust_model,
  const FullyActuatedAllocationModel & allocator) noexcept;

PhysicalWrench frdWrenchToFlu(const WrenchVector & wrench_frd) noexcept;
WrenchVector fluWrenchToFrd(const PhysicalWrench & wrench_flu) noexcept;

}  // namespace flying_hand_mode::fully_actuated
