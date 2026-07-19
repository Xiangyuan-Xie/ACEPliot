#pragma once

#include <flying_hand_quadrotor_mode/arm_kinematics.hpp>
#include <flying_hand_quadrotor_mode/whole_body_solver.hpp>

#include <rclcpp/rclcpp.hpp>

#include <optional>

namespace flying_hand_quadrotor_mode
{

class FlyingHandController
{
public:
  explicit FlyingHandController(rclcpp::Node & node);

  ControllerOutput update(const ControllerInput & input);
  bool acceptPendingUpdate() noexcept;
  void rejectPendingUpdate() noexcept;
  void recoverAfterRejectedUpdate() noexcept;
  void reset() noexcept;

private:
  EndEffectorTarget slewTarget(
    const ControllerInput & input,
    std::optional<EndEffectorTarget> & target_state) const;

  WholeBodySolver solver_;
  ArmKinematics arm_kinematics_;
  L1AdaptiveController adaptive_controller_;
  RotorThrustCommandModel thrust_model_;
  Px4AllocatorModel allocator_model_;
  PhysicalWrench disturbance_estimate_flu_{};
  WholeBodyControlVector previous_control_{WholeBodyControlVector::Zero()};
  std::optional<EndEffectorTarget> slewed_target_;
  std::optional<QuadrotorActuation> previous_applied_actuation_flu_;
  bool adaptive_enabled_;
  double target_position_slew_m_s_;
  double target_orientation_slew_rad_s_;
  double collective_slew_n_s_;
  Eigen::Vector3d moment_slew_nm_s_;

  std::optional<L1AdaptiveController> pending_adaptive_controller_;
  PhysicalWrench pending_disturbance_estimate_flu_{};
  WholeBodyControlVector pending_previous_control_{WholeBodyControlVector::Zero()};
  std::optional<EndEffectorTarget> pending_slewed_target_;
  std::optional<QuadrotorActuation> pending_applied_actuation_flu_;
};

}  // namespace flying_hand_quadrotor_mode
