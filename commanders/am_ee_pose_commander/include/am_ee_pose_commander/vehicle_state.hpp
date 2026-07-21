/****************************************************************************
 * Copyright (c) 2026 Xiangyuan Xie <dragonboat_xxy@163.com>.
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#pragma once

#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <optional>
#include <string>

namespace am_ee_pose_commander
{

struct VehicleState
{
  Eigen::Vector3f position_enu_m{Eigen::Vector3f::Zero()};
  Eigen::Quaternionf attitude_enu{Eigen::Quaternionf::Identity()};
  Eigen::Vector3f linear_velocity_flu_m_s{Eigen::Vector3f::Zero()};
  Eigen::Vector3f angular_velocity_flu_rad_s{Eigen::Vector3f::Zero()};
  uint64_t timestamp_us{0};
  uint64_t timestamp_sample_us{0};

  bool allFinite() const noexcept;
};

std::optional<VehicleState> vehicleStateFromPx4Odometry(
  const px4_msgs::msg::VehicleOdometry & message,
  std::string * error = nullptr);

}  // namespace am_ee_pose_commander
