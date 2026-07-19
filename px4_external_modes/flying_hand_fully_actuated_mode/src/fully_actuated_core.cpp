#include <flying_hand_fully_actuated_mode/fully_actuated_core.hpp>

#include <Eigen/LU>
#include <Eigen/QR>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <limits>

namespace flying_hand_fully_actuated_mode
{
namespace
{

constexpr int kActiveSetCount = 729;
constexpr double kTolerance = 1.0e-9;
const Eigen::Matrix3d kFluFromFrd =
  Eigen::Vector3d(1.0, -1.0, -1.0).asDiagonal();

bool finiteGeometry(const RotorGeometry & geometry) noexcept
{
  return geometry.position_frd_m.allFinite() && geometry.axis_frd.allFinite() &&
         geometry.moment_ratio_m.allFinite() && geometry.minimum_thrust_n.allFinite() &&
         geometry.maximum_thrust_n.allFinite() &&
         (geometry.minimum_thrust_n.array() >= 0.0).all() &&
         (geometry.minimum_thrust_n.array() < geometry.maximum_thrust_n.array()).all();
}

bool finiteAllocationModel(const FullyActuatedAllocationModel & model) noexcept
{
  return model.valid && model.physical_allocation_frd.allFinite() &&
         model.physical_inverse_frd.allFinite() &&
         model.px4_actuator_from_control.allFinite() &&
         model.px4_control_from_actuator.allFinite() &&
         model.minimum_thrust_n.allFinite() && model.maximum_thrust_n.allFinite() &&
         model.wrench_weights.allFinite() && (model.wrench_weights.array() > 0.0).all();
}

}  // namespace

FullyActuatedAllocationModel buildAllocationModel(
  const RotorGeometry & geometry, double maximum_condition_number) noexcept
{
  FullyActuatedAllocationModel model;
  if (!finiteGeometry(geometry) || !std::isfinite(maximum_condition_number) ||
    maximum_condition_number <= 1.0)
  {
    return model;
  }

  for (int rotor = 0; rotor < kRotorCount; ++rotor) {
    Eigen::Vector3d axis = geometry.axis_frd.col(rotor);
    const double axis_norm = axis.norm();
    if (!std::isfinite(axis_norm) || axis_norm <= kTolerance) {
      return model;
    }
    axis /= axis_norm;
    const Eigen::Vector3d moment =
      geometry.position_frd_m.col(rotor).cross(axis) -
      geometry.moment_ratio_m[rotor] * axis;
    model.physical_allocation_frd.col(rotor).head<3>() = axis;
    model.physical_allocation_frd.col(rotor).tail<3>() = moment;
  }

  const Eigen::JacobiSVD<AllocationMatrix> svd(model.physical_allocation_frd);
  const auto singular_values = svd.singularValues();
  const double largest = singular_values[0];
  const double smallest = singular_values[kActuationDimension - 1];
  if (!std::isfinite(largest) || !std::isfinite(smallest) || smallest <= 1.0e-8) {
    return model;
  }
  model.condition_number = largest / smallest;
  if (!std::isfinite(model.condition_number) ||
    model.condition_number > maximum_condition_number)
  {
    return model;
  }

  const Eigen::FullPivLU<AllocationMatrix> physical_lu(model.physical_allocation_frd);
  if (!physical_lu.isInvertible()) {
    return model;
  }
  model.physical_inverse_frd = physical_lu.inverse();

  AllocationMatrix effectiveness;
  effectiveness.topRows<3>() = model.physical_allocation_frd.bottomRows<3>();
  effectiveness.bottomRows<3>() = model.physical_allocation_frd.topRows<3>();
  const Eigen::FullPivLU<AllocationMatrix> effectiveness_lu(effectiveness);
  if (!effectiveness_lu.isInvertible()) {
    return model;
  }
  AllocationMatrix mixer = effectiveness_lu.inverse();

  int nonzero_roll = 0;
  int nonzero_pitch = 0;
  for (int rotor = 0; rotor < kRotorCount; ++rotor) {
    nonzero_roll += std::abs(mixer(rotor, 0)) > 1.0e-3 ? 1 : 0;
    nonzero_pitch += std::abs(mixer(rotor, 1)) > 1.0e-3 ? 1 : 0;
  }
  if (nonzero_roll == 0 || nonzero_pitch == 0) {
    return model;
  }
  const double roll_scale = std::sqrt(
    mixer.col(0).squaredNorm() / (static_cast<double>(nonzero_roll) / 2.0));
  const double pitch_scale = std::sqrt(
    mixer.col(1).squaredNorm() / (static_cast<double>(nonzero_pitch) / 2.0));
  const double roll_pitch_scale = std::max(roll_scale, pitch_scale);
  const double yaw_scale = mixer.col(2).maxCoeff();
  if (!std::isfinite(roll_pitch_scale) || roll_pitch_scale <= kTolerance ||
    !std::isfinite(yaw_scale) || yaw_scale <= kTolerance)
  {
    return model;
  }
  mixer.col(0) /= roll_pitch_scale;
  mixer.col(1) /= roll_pitch_scale;
  mixer.col(2) /= yaw_scale;

  std::array<double, 3> thrust_scale{};
  for (int axis = 2; axis >= 0; --axis) {
    double magnitude_sum = 0.0;
    int nonzero = 0;
    for (int rotor = 0; rotor < kRotorCount; ++rotor) {
      const double magnitude = std::abs(mixer(rotor, 3 + axis));
      magnitude_sum += magnitude;
      nonzero += magnitude > std::numeric_limits<double>::epsilon() ? 1 : 0;
    }
    if (nonzero > 0) {
      thrust_scale[static_cast<std::size_t>(axis)] =
        magnitude_sum / static_cast<double>(nonzero);
    } else if (axis != 2) {
      thrust_scale[static_cast<std::size_t>(axis)] = thrust_scale[2];
    } else {
      return model;
    }
  }
  for (int axis = 0; axis < 3; ++axis) {
    const double scale = thrust_scale[static_cast<std::size_t>(axis)];
    if (!std::isfinite(scale) || scale <= kTolerance) {
      return model;
    }
    mixer.col(3 + axis) /= scale;
  }

  const Eigen::FullPivLU<AllocationMatrix> mixer_lu(mixer);
  if (!mixer.allFinite() || !mixer_lu.isInvertible()) {
    return model;
  }
  model.px4_actuator_from_control = mixer;
  model.px4_control_from_actuator = mixer_lu.inverse();
  model.minimum_thrust_n = geometry.minimum_thrust_n;
  model.maximum_thrust_n = geometry.maximum_thrust_n;

  const WrenchVector maximum_wrench =
    model.physical_allocation_frd.cwiseAbs() * model.maximum_thrust_n;
  if (!maximum_wrench.allFinite() || (maximum_wrench.array() <= kTolerance).any()) {
    return model;
  }
  model.wrench_weights = maximum_wrench.cwiseInverse();
  model.valid = true;
  return model;
}

WrenchProjectionResult projectWrenchToRotorBox(
  const WrenchVector & desired_wrench_frd,
  const FullyActuatedAllocationModel & model) noexcept
{
  WrenchProjectionResult result;
  result.requested_wrench_frd = desired_wrench_frd;
  if (!desired_wrench_frd.allFinite() || !finiteAllocationModel(model)) {
    return result;
  }

  const AllocationMatrix weighted_allocation =
    model.wrench_weights.asDiagonal() * model.physical_allocation_frd;
  const WrenchVector weighted_desired =
    model.wrench_weights.cwiseProduct(desired_wrench_frd);
  double best_cost = std::numeric_limits<double>::infinity();

  for (int active_set = 0; active_set < kActiveSetCount; ++active_set) {
    int encoded = active_set;
    RotorVector candidate = RotorVector::Zero();
    std::array<int, kRotorCount> free_indices{};
    int free_count = 0;
    for (int rotor = 0; rotor < kRotorCount; ++rotor) {
      const int constraint = encoded % 3;
      encoded /= 3;
      if (constraint == 0) {
        candidate[rotor] = model.minimum_thrust_n[rotor];
      } else if (constraint == 1) {
        free_indices[static_cast<std::size_t>(free_count++)] = rotor;
      } else {
        candidate[rotor] = model.maximum_thrust_n[rotor];
      }
    }

    if (free_count > 0) {
      Eigen::MatrixXd free_allocation(kActuationDimension, free_count);
      for (int column = 0; column < free_count; ++column) {
        free_allocation.col(column) =
          weighted_allocation.col(free_indices[static_cast<std::size_t>(column)]);
      }
      const WrenchVector right_hand_side =
        weighted_desired - weighted_allocation * candidate;
      const Eigen::VectorXd solution =
        free_allocation.colPivHouseholderQr().solve(right_hand_side);
      if (!solution.allFinite()) {
        continue;
      }
      for (int column = 0; column < free_count; ++column) {
        candidate[free_indices[static_cast<std::size_t>(column)]] = solution[column];
      }
    }

    bool within_bounds = true;
    for (int rotor = 0; rotor < kRotorCount; ++rotor) {
      const double scale = std::max(1.0, model.maximum_thrust_n[rotor]);
      const double tolerance = 1.0e-9 * scale;
      if (!std::isfinite(candidate[rotor]) ||
        candidate[rotor] < model.minimum_thrust_n[rotor] - tolerance ||
        candidate[rotor] > model.maximum_thrust_n[rotor] + tolerance)
      {
        within_bounds = false;
        break;
      }
      candidate[rotor] = std::clamp(
        candidate[rotor], model.minimum_thrust_n[rotor], model.maximum_thrust_n[rotor]);
    }
    if (!within_bounds) {
      continue;
    }

    const WrenchVector error = model.wrench_weights.cwiseProduct(
      model.physical_allocation_frd * candidate - desired_wrench_frd);
    const double cost = error.squaredNorm();
    if (std::isfinite(cost) && cost < best_cost) {
      best_cost = cost;
      result.rotor_thrust_n = candidate;
      result.weighted_error_squared = cost;
      result.valid = true;
    }
  }

  if (result.valid) {
    result.projected_wrench_frd =
      model.physical_allocation_frd * result.rotor_thrust_n;
    result.saturated = best_cost > 1.0e-12;
  }
  return result;
}

Px4NormalizedWrench rotorThrustToPx4Normalized(
  const RotorVector & rotor_thrust_n,
  const RotorThrustCommandModel & thrust_model,
  const FullyActuatedAllocationModel & allocator) noexcept
{
  Px4NormalizedWrench result;
  if (!rotor_thrust_n.allFinite() || !thrust_model.maximum_thrust_n.allFinite() ||
    !thrust_model.kappa.allFinite() || !finiteAllocationModel(allocator) ||
    (thrust_model.maximum_thrust_n.array() <= 0.0).any() ||
    (thrust_model.kappa.array() < 0.0).any() ||
    (thrust_model.kappa.array() > 1.0).any())
  {
    return result;
  }

  RotorVector actuator_command;
  for (int rotor = 0; rotor < kRotorCount; ++rotor) {
    if (rotor_thrust_n[rotor] < -kTolerance ||
      rotor_thrust_n[rotor] > thrust_model.maximum_thrust_n[rotor] + kTolerance)
    {
      return result;
    }
    const double normalized_speed = std::sqrt(
      std::clamp(
        rotor_thrust_n[rotor] / thrust_model.maximum_thrust_n[rotor], 0.0, 1.0));
    const double kappa = thrust_model.kappa[rotor];
    if (kappa < 1.0e-9) {
      actuator_command[rotor] = normalized_speed * normalized_speed;
    } else {
      const double linear = 1.0 - kappa;
      const double root =
        (std::sqrt(linear * linear + 4.0 * kappa * normalized_speed) - linear) /
        (2.0 * kappa);
      actuator_command[rotor] = root * root;
    }
  }

  const WrenchVector normalized_control =
    allocator.px4_control_from_actuator * actuator_command;
  if (!normalized_control.allFinite() ||
    (normalized_control.array().abs() > 1.0 + 1.0e-7).any())
  {
    return result;
  }

  const RotorVector realized_actuator =
    allocator.px4_actuator_from_control * normalized_control;
  if (!realized_actuator.allFinite() || (realized_actuator.array() < -1.0e-7).any() ||
    (realized_actuator.array() > 1.0 + 1.0e-7).any())
  {
    return result;
  }

  for (int rotor = 0; rotor < kRotorCount; ++rotor) {
    const double bounded = std::clamp(realized_actuator[rotor], 0.0, 1.0);
    const double speed = thrust_model.kappa[rotor] * bounded +
      (1.0 - thrust_model.kappa[rotor]) * std::sqrt(bounded);
    result.realized_rotor_thrust_n[rotor] =
      thrust_model.maximum_thrust_n[rotor] * speed * speed;
  }
  result.torque_frd = normalized_control.head<3>().cast<float>();
  result.thrust_frd = normalized_control.tail<3>().cast<float>();
  result.actuator_command = actuator_command;
  result.valid = true;
  return result;
}

PhysicalWrench frdWrenchToFlu(const WrenchVector & wrench_frd) noexcept
{
  PhysicalWrench result;
  result.force_b_n = kFluFromFrd * wrench_frd.head<3>();
  result.moment_b_nm = kFluFromFrd * wrench_frd.tail<3>();
  return result;
}

WrenchVector fluWrenchToFrd(const PhysicalWrench & wrench_flu) noexcept
{
  WrenchVector result;
  result.head<3>() = kFluFromFrd * wrench_flu.force_b_n;
  result.tail<3>() = kFluFromFrd * wrench_flu.moment_b_nm;
  return result;
}

}  // namespace flying_hand_fully_actuated_mode
