#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstdint>
#include <functional>

namespace flying_hand_mode::runtime
{

constexpr int kArmJointCount = 4;
constexpr int kWrenchDimension = 6;

using JointVector = Eigen::Matrix<double, kArmJointCount, 1>;
using WrenchVector = Eigen::Matrix<double, kWrenchDimension, 1>;
using WrenchGainMatrix = Eigen::Matrix<double, kWrenchDimension, kWrenchDimension>;

// Physical vectors use a forward-left-up (FLU) body frame.
struct PhysicalWrench
{
  Eigen::Vector3d force_b_n{Eigen::Vector3d::Zero()};
  Eigen::Vector3d moment_b_nm{Eigen::Vector3d::Zero()};

  WrenchVector vector() const noexcept
  {
    WrenchVector value;
    value.head<3>() = force_b_n;
    value.tail<3>() = moment_b_nm;
    return value;
  }

  static PhysicalWrench fromVector(const WrenchVector & value) noexcept
  {
    PhysicalWrench wrench;
    wrench.force_b_n = value.head<3>();
    wrench.moment_b_nm = value.tail<3>();
    return wrench;
  }

  bool allFinite() const noexcept
  {
    return force_b_n.allFinite() && moment_b_nm.allFinite();
  }
};

struct ControllerInput
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::uint64_t sample_timestamp_us{0};
  double dt_s{0.01};
  Eigen::Vector3d position_ned_m{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond attitude_ned_frd{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d linear_velocity_ned_m_s{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_velocity_frd_rad_s{Eigen::Vector3d::Zero()};
  Eigen::Vector3d gyro_frd_rad_s{Eigen::Vector3d::Zero()};
  JointVector arm_position_rad{JointVector::Zero()};
  JointVector arm_velocity_rad_s{JointVector::Zero()};
  Eigen::Isometry3d current_ee_pose_ned{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d target_ee_pose_ned{Eigen::Isometry3d::Identity()};
};

struct ControllerOutput
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::uint64_t sample_timestamp_us{0};
  PhysicalWrench nominal_wrench_flu{};
  PhysicalWrench adaptive_wrench_flu{};
  PhysicalWrench applied_wrench_flu{};
  Eigen::Vector3f normalized_thrust_frd{Eigen::Vector3f::Zero()};
  Eigen::Vector3f normalized_torque_frd{Eigen::Vector3f::Zero()};
  JointVector arm_position_command_rad{JointVector::Zero()};
  double tracking_cost{0.0};
  double allocation_condition_number{0.0};
  bool rotor_saturated{false};
  bool feasible{false};

  bool allFinite() const noexcept;
  bool normalizedCommandValid() const noexcept;
};

using ControllerCallback = std::function<ControllerOutput(const ControllerInput &)>;

struct ControllerCallbacks
{
  ControllerCallback update;
  std::function<bool()> accept;
  std::function<void()> reject;
  std::function<void()> recover;
  std::function<void()> reset;

  bool valid() const noexcept
  {
    return static_cast<bool>(update) && static_cast<bool>(accept) &&
           static_cast<bool>(reject) && static_cast<bool>(recover) &&
           static_cast<bool>(reset);
  }
};

}  // namespace flying_hand_mode::runtime
