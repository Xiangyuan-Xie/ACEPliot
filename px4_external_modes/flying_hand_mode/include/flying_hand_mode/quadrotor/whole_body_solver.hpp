#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <memory>

#include <flying_hand_mode/quadrotor/controller_core.hpp>

namespace flying_hand_mode::quadrotor
{

constexpr int kWholeBodyStateDimension = 17;
constexpr int kWholeBodyControlDimension = 8;
constexpr int kWholeBodyParameterDimension = 37;
constexpr int kWholeBodyHorizonNodes = 100;

using WholeBodyStateVector = Eigen::Matrix<double, kWholeBodyStateDimension, 1>;
using WholeBodyControlVector = Eigen::Matrix<double, kWholeBodyControlDimension, 1>;

struct WholeBodyState
{
  // Position and linear velocity refer to the current whole-vehicle center of mass.
  Eigen::Vector3d position_ned_m{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond attitude_ned_frd{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d velocity_ned_m_s{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_velocity_frd_rad_s{Eigen::Vector3d::Zero()};
  JointVector arm_position_rad{JointVector::Zero()};

  WholeBodyStateVector vector() const noexcept;
  bool allFinite() const noexcept;
};

struct WholeBodyMassProperties
{
  double mass_kg{0.0};
  Eigen::Vector3d center_of_mass_flu_m{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d inertia_com_flu_kg_m2{Eigen::Matrix3d::Zero()};

  bool allFinite() const noexcept;
};

struct EndEffectorTarget
{
  Eigen::Vector3d position_ned_m{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond attitude_ned{Eigen::Quaterniond::Identity()};

  bool allFinite() const noexcept;
};

struct WholeBodyDisturbance
{
  Eigen::Vector3d force_frd_n{Eigen::Vector3d::Zero()};
  Eigen::Vector3d moment_frd_nm{Eigen::Vector3d::Zero()};
  JointVector arm_velocity_rad_s{JointVector::Zero()};

  bool allFinite() const noexcept;
};

struct WholeBodySolverInput
{
  WholeBodyState state{};
  WholeBodyMassProperties mass_properties{};
  EndEffectorTarget target{};
  WholeBodyDisturbance disturbance{};
  WholeBodyControlVector previous_control{WholeBodyControlVector::Zero()};
};

struct WholeBodySolverOutput
{
  QuadrotorActuation actuation_frd{};
  JointVector arm_position_command_rad{JointVector::Zero()};
  double solver_time_s{0.0};
  int solver_status{-1};
  bool valid{false};
};

class WholeBodySolver
{
public:
  WholeBodySolver();
  ~WholeBodySolver();

  WholeBodySolver(const WholeBodySolver &) = delete;
  WholeBodySolver & operator=(const WholeBodySolver &) = delete;

  WholeBodySolverOutput solve(const WholeBodySolverInput & input) noexcept;
  void reset() noexcept;

private:
  struct Implementation;
  std::unique_ptr<Implementation> implementation_;
};

}  // namespace flying_hand_mode::quadrotor
