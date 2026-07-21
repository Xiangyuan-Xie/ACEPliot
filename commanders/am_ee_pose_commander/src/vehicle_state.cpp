/****************************************************************************
 * Copyright (c) 2026 Xiangyuan Xie <dragonboat_xxy@163.com>.
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <am_ee_pose_commander/vehicle_state.hpp>

#include <px4_ros2/utils/frame_conversion.hpp>

#include <cmath>

namespace am_ee_pose_commander
{
namespace
{

constexpr float kQuaternionNormEpsilon = 1.0e-6F;

void setError(std::string * error, const std::string & value)
{
  if (error != nullptr) {
    *error = value;
  }
}

}  // namespace

bool VehicleState::allFinite() const noexcept
{
  return position_enu_m.allFinite() && attitude_enu.coeffs().allFinite() &&
         linear_velocity_flu_m_s.allFinite() && angular_velocity_flu_rad_s.allFinite();
}

std::optional<VehicleState> vehicleStateFromPx4Odometry(
  const px4_msgs::msg::VehicleOdometry & message,
  std::string * error)
{
  if (message.pose_frame != px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED) {
    setError(error, "vehicle odometry pose_frame must be NED");
    return std::nullopt;
  }

  const Eigen::Vector3f position_ned(
    message.position[0], message.position[1], message.position[2]);
  Eigen::Quaternionf attitude_ned(
    message.q[0], message.q[1], message.q[2], message.q[3]);
  const Eigen::Vector3f angular_velocity_frd(
    message.angular_velocity[0], message.angular_velocity[1], message.angular_velocity[2]);
  if (!position_ned.allFinite() || !attitude_ned.coeffs().allFinite() ||
    attitude_ned.norm() < kQuaternionNormEpsilon || !angular_velocity_frd.allFinite())
  {
    setError(error, "vehicle odometry contains invalid pose or angular velocity");
    return std::nullopt;
  }

  attitude_ned.normalize();
  VehicleState state;
  state.position_enu_m = px4_ros2::positionNedToEnu(position_ned);
  state.attitude_enu = px4_ros2::attitudeNedToEnu(attitude_ned).normalized();
  state.angular_velocity_flu_rad_s = px4_ros2::frdToFlu(angular_velocity_frd);

  const Eigen::Vector3f velocity(
    message.velocity[0], message.velocity[1], message.velocity[2]);
  if (!velocity.allFinite()) {
    setError(error, "vehicle odometry contains invalid linear velocity");
    return std::nullopt;
  }
  if (message.velocity_frame == px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED) {
    const Eigen::Vector3f velocity_enu = px4_ros2::positionNedToEnu(velocity);
    state.linear_velocity_flu_m_s = state.attitude_enu.inverse() * velocity_enu;
  } else {
    if (message.velocity_frame != px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD) {
      setError(error, "vehicle odometry velocity_frame must be NED or BODY_FRD");
      return std::nullopt;
    }
    state.linear_velocity_flu_m_s = px4_ros2::frdToFlu(velocity);
  }

  state.timestamp_us = message.timestamp;
  state.timestamp_sample_us = message.timestamp_sample;
  if (!state.allFinite()) {
    setError(error, "converted vehicle state contains non-finite values");
    return std::nullopt;
  }
  return state;
}

}  // namespace am_ee_pose_commander
