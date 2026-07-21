#pragma once

#include <flying_hand_mode/runtime/controller_types.hpp>

#include <Eigen/Geometry>

#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace flying_hand_mode::fully_actuated
{

class FlyingHandFullyActuatedController
{
public:
  explicit FlyingHandFullyActuatedController(rclcpp::Node & node);
  ~FlyingHandFullyActuatedController();

  flying_hand_mode::runtime::ControllerOutput update(
    const flying_hand_mode::runtime::ControllerInput & input);
  bool acceptPendingUpdate() noexcept;
  void rejectPendingUpdate() noexcept;
  void recoverAfterRejectedUpdate() noexcept;
  void reset() noexcept;

  Eigen::Isometry3d endEffectorPoseFlu(
    const flying_hand_mode::runtime::JointVector & joint_position_rad) const noexcept;

private:
  struct Implementation;
  std::unique_ptr<Implementation> implementation_;
};

}  // namespace flying_hand_mode::fully_actuated
