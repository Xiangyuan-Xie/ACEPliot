/****************************************************************************
 * Copyright (c) 2026 Xiangyuan Xie <dragonboat_xxy@163.com>.
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#pragma once

#include <am_position_offboard_reference.hpp>
#include <arm_robot_data.hpp>
#include <flight_logger.hpp>
#include <observation_buffer.hpp>
#include <policy_inference/onnx_policy.hpp>
#include <policy_inference/recurrent_state.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/rates.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

namespace am_position_mode
{

inline constexpr char kModeName[] = "AM Position";
inline constexpr bool kActivateEvenWhileDisarmed = true;

struct DeploymentMetadata
{
  std::string action_semantics;
  std::string body_frame;
  std::string publish_frame;
  std::string collective_preprocess;
  float max_body_rate_rad_s{6.0F};
};

struct BodyRateThrustSetpointFrd
{
  Eigen::Vector3f body_rate_rad_s{Eigen::Vector3f::Zero()};
  Eigen::Vector3f thrust{Eigen::Vector3f::Zero()};
};

DeploymentMetadata loadDeploymentMetadata(const std::string & metadata_path);
BodyRateThrustSetpointFrd mapRawActionToSetpoint(
  const std::vector<float> & raw_action,
  float max_body_rate_rad_s,
  float collective_scale);

class AmPositionMode final : public px4_ros2::ModeBase
{
public:
  explicit AmPositionMode(
    rclcpp::Node & node,
    const std::string & mode_name = kModeName,
    bool activate_disarmed = kActivateEvenWhileDisarmed,
    const std::string & topic_namespace_prefix = "",
    const std::string & root_dir = "");

  ~AmPositionMode() override = default;

  void onActivate() override;
  void onDeactivate() override;
  void updateSetpoint(float dt_s) override;

private:
  struct CommandReference
  {
    Eigen::Vector3f desired_lin_vel_b{Eigen::Vector3f::Zero()};
    Eigen::Vector3f desired_ang_vel_b{Eigen::Vector3f::Zero()};
    Eigen::Vector3f desired_pos_w{Eigen::Vector3f::Zero()};
    Eigen::Quaternionf desired_quat_w{Eigen::Quaternionf::Identity()};
    std::array<bool, 3> lin_cmd_active{{false, false, false}};
    std::array<bool, 3> ang_cmd_active{{false, false, false}};
    bool has_lin_vel_cmd{false};
    bool has_ang_vel_cmd{false};
  };

  void getObservation(policy_inference::TensorMap & inputs, float dt_s);
  void applyAction(const policy_inference::TensorMap & action);
  void updateTargets(float dt_s);
  void logTargetValues(float dt_s);
  void logOdometryData();
  void logTensor(const std::string & topic, const policy_inference::TensorMap & tensors);

  void offboardControlModeCallback(
    const px4_msgs::msg::OffboardControlMode::SharedPtr message);
  void trajectorySetpointCallback(
    const px4_msgs::msg::TrajectorySetpoint::SharedPtr message);
  void simClockCallback(const rosgraph_msgs::msg::Clock::SharedPtr message);
  bool hasFreshExternalReference();
  bool getCurrentModeTime(rclcpp::Time & now);
  void configureClockSource(rclcpp::Node & node);
  bool updateExternalReferenceReceiptTime(rclcpp::Time & receipt_time);

  std::unique_ptr<policy_inference::OnnxPolicy> policy_;
  policy_inference::RecurrentState recurrent_state_;
  std::unique_ptr<ArmRobotData> robot_data_;
  std::unique_ptr<FlightLogger> flight_logger_;
  ObservationBuffer action_observation_buffer_;
  std::shared_ptr<px4_ros2::RatesSetpointType> rates_setpoint_;

  DeploymentMetadata metadata_;
  std::string metadata_path_;
  float max_body_rate_rad_s_{6.0F};
  float collective_scale_{1.0F};

  std::string offboard_control_mode_topic_{"/fmu/in/offboard_control_mode"};
  std::string trajectory_setpoint_topic_{"/fmu/in/trajectory_setpoint"};
  double offboard_setpoint_timeout_s_{0.5};
  rclcpp::Subscription<px4_msgs::msg::OffboardControlMode>::SharedPtr
    offboard_control_mode_sub_;
  rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr
    trajectory_setpoint_sub_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr sim_clock_sub_;
  px4_msgs::msg::OffboardControlMode last_offboard_control_mode_msg_{};
  px4_msgs::msg::TrajectorySetpoint last_trajectory_setpoint_msg_{};
  rclcpp::Time last_offboard_control_mode_time_{0};
  rclcpp::Time last_trajectory_setpoint_time_{0};
  bool has_offboard_control_mode_msg_{false};
  bool has_trajectory_setpoint_msg_{false};

  bool use_sim_time_{false};
  bool has_sim_time_{false};
  std::string sim_clock_topic_{"/acesim/clock"};
  rclcpp::Time latest_sim_time_{0, 0, RCL_ROS_TIME};
  bool warned_waiting_for_sim_time_{false};

  bool hover_lock_active_{false};
  Eigen::Vector3f hover_lock_pos_w_{Eigen::Vector3f::Zero()};
  float hover_lock_yaw_w_{0.0F};
  CommandReference current_cmd_ref_{};
};

}  // namespace am_position_mode
