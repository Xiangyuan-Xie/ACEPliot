#pragma once

#include <flying_hand_fully_actuated_mode/dh_kinematics.hpp>
#include <flying_hand_fully_actuated_mode/fully_actuated_core.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <memory>

namespace flying_hand_fully_actuated_mode
{

constexpr int kWholeBodyStateDimension = 17;
constexpr int kWholeBodyControlDimension = 10;
constexpr int kWholeBodyParameterDimension = 84;
constexpr int kWholeBodyHorizonNodes = 100;

using WholeBodyStateVector = Eigen::Matrix<double, kWholeBodyStateDimension, 1>;
using WholeBodyControlVector = Eigen::Matrix<double, kWholeBodyControlDimension, 1>;

struct FullyActuatedModelConfig
{
  double mass_kg{0.0};
  Eigen::Matrix3d inertia_frd_kg_m2{Eigen::Matrix3d::Zero()};
  DhKinematicsConfig arm_kinematics{};
  JointVector arm_servo_delay_s{JointVector::Zero()};
  JointVector arm_lower_rad{JointVector::Zero()};
  JointVector arm_upper_rad{JointVector::Zero()};
  JointVector arm_max_velocity_rad_s{JointVector::Zero()};
  JointVector arm_home_rad{JointVector::Zero()};
  Eigen::Vector3d maximum_linear_velocity_m_s{Eigen::Vector3d::Zero()};
  Eigen::Vector3d maximum_angular_velocity_rad_s{Eigen::Vector3d::Zero()};
  FullyActuatedAllocationModel allocation{};

  bool allFinite() const noexcept;
};

struct WholeBodyState
{
  Eigen::Vector3d position_ned_m{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond attitude_ned_frd{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d velocity_ned_m_s{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_velocity_frd_rad_s{Eigen::Vector3d::Zero()};
  JointVector arm_position_rad{JointVector::Zero()};

  WholeBodyStateVector vector() const noexcept;
  bool allFinite() const noexcept;
};

struct EndEffectorTarget
{
  Eigen::Vector3d position_ned_m{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond attitude_ned{Eigen::Quaterniond::Identity()};

  bool allFinite() const noexcept;
};

struct WholeBodySolverInput
{
  WholeBodyState state{};
  EndEffectorTarget target{};
};

struct WholeBodySolverOutput
{
  WrenchVector wrench_frd{WrenchVector::Zero()};
  JointVector arm_position_command_rad{JointVector::Zero()};
  double tracking_cost{0.0};
  double solver_time_s{0.0};
  int solver_status{-1};
  bool valid{false};
};

class WholeBodySolver
{
public:
  explicit WholeBodySolver(FullyActuatedModelConfig config);
  ~WholeBodySolver();

  WholeBodySolver(const WholeBodySolver &) = delete;
  WholeBodySolver & operator=(const WholeBodySolver &) = delete;

  WholeBodySolverOutput solve(const WholeBodySolverInput & input) noexcept;
  void reset() noexcept;
  const FullyActuatedModelConfig & config() const noexcept;

private:
  struct Implementation;
  FullyActuatedModelConfig config_;
  std::unique_ptr<Implementation> implementation_;
};

}  // namespace flying_hand_fully_actuated_mode
