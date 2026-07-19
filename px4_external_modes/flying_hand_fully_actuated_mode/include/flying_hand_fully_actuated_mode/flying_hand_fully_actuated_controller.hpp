#pragma once

#include <flying_hand_control_common/controller_types.hpp>

#include <Eigen/Geometry>

#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace flying_hand_fully_actuated_mode
{

class FlyingHandFullyActuatedController
{
public:
  explicit FlyingHandFullyActuatedController(rclcpp::Node & node);
  ~FlyingHandFullyActuatedController();

  flying_hand_control_common::ControllerOutput update(
    const flying_hand_control_common::ControllerInput & input);
  bool acceptPendingUpdate() noexcept;
  void rejectPendingUpdate() noexcept;
  void recoverAfterRejectedUpdate() noexcept;
  void reset() noexcept;

  Eigen::Isometry3d endEffectorPoseFlu(
    const flying_hand_control_common::JointVector & joint_position_rad) const noexcept;

private:
  struct Implementation;
  std::unique_ptr<Implementation> implementation_;
};

}  // namespace flying_hand_fully_actuated_mode
