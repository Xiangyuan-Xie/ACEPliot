#include <flying_hand_mode/fully_actuated/whole_body_solver.hpp>

#include <acados_solver_flying_hand_fully_actuated.h>

#include <gtest/gtest.h>

#include <cmath>

namespace flying_hand_mode::fully_actuated
{
namespace
{

FullyActuatedModelConfig modelConfig()
{
  RotorGeometry geometry;
  constexpr double radius = 0.34;
  constexpr double tilt = 0.3490658503988659;
  for (int rotor = 0; rotor < kRotorCount; ++rotor) {
    const double angle = static_cast<double>(rotor) * M_PI / 3.0;
    const double direction = rotor % 2 == 0 ? 1.0 : -1.0;
    geometry.position_frd_m.col(rotor) =
      Eigen::Vector3d(radius * std::cos(angle), radius * std::sin(angle), 0.0);
    geometry.axis_frd.col(rotor) = Eigen::Vector3d(
      -direction * std::sin(tilt) * std::sin(angle),
      direction * std::sin(tilt) * std::cos(angle), -std::cos(tilt));
    geometry.moment_ratio_m[rotor] = direction * 0.02;
  }
  geometry.maximum_thrust_n.setConstant(20.0);

  FullyActuatedModelConfig config;
  config.mass_kg = 5.0;
  config.inertia_frd_kg_m2 = Eigen::Vector3d(0.1, 0.1, 0.15).asDiagonal();
  config.arm_kinematics.d_m << 0.0, 0.050, 0.0, 0.076;
  config.arm_kinematics.a_m << 0.363, 0.441, 0.007, 0.200;
  config.arm_kinematics.alpha_rad << 0.10, -0.10, -1.578, 0.0;
  config.arm_servo_delay_s << 0.66, 0.68, 0.81, 0.85;
  config.arm_lower_rad.setConstant(-2.6);
  config.arm_upper_rad.setConstant(2.6);
  config.arm_max_velocity_rad_s << 2.0, 2.0, 2.0, 3.0;
  config.maximum_linear_velocity_m_s.setConstant(5.0);
  config.maximum_angular_velocity_rad_s.setConstant(3.0);
  config.allocation = buildAllocationModel(geometry, 50.0);
  return config;
}

WholeBodySolverInput hoverInput(const FullyActuatedModelConfig & config)
{
  WholeBodySolverInput input;
  const DhKinematics kinematics(config.arm_kinematics);
  const Eigen::Isometry3d ee_flu = kinematics.endEffectorPoseFlu(JointVector::Zero());
  const Eigen::Matrix3d frd_from_flu =
    Eigen::Vector3d(1.0, -1.0, -1.0).asDiagonal();
  input.target.position_ned_m = frd_from_flu * ee_flu.translation();
  input.target.attitude_ned = Eigen::Quaterniond(frd_from_flu * ee_flu.linear());
  return input;
}

TEST(FullyActuatedWholeBodySolver, GeneratedDimensionsMatchPaperArchitecture)
{
  EXPECT_EQ(FLYING_HAND_FULLY_ACTUATED_NX, 17);
  EXPECT_EQ(FLYING_HAND_FULLY_ACTUATED_NU, 10);
  EXPECT_EQ(FLYING_HAND_FULLY_ACTUATED_N, 100);
  EXPECT_EQ(FLYING_HAND_FULLY_ACTUATED_NH0, 10);
}

TEST(FullyActuatedWholeBodySolver, ProducesFiniteHoverWrench)
{
  const FullyActuatedModelConfig config = modelConfig();
  ASSERT_TRUE(config.allFinite());
  WholeBodySolver solver(config);
  const WholeBodySolverOutput output = solver.solve(hoverInput(config));
  ASSERT_TRUE(output.valid) << output.solver_status;
  EXPECT_LT(output.wrench_frd.z(), 0.0);
  EXPECT_TRUE(output.arm_position_command_rad.allFinite());
  EXPECT_GE(output.tracking_cost, 0.0);
}

TEST(FullyActuatedWholeBodySolver, RespondsWithLateralForceWithoutTiltState)
{
  const FullyActuatedModelConfig config = modelConfig();
  WholeBodySolver solver(config);
  WholeBodySolverInput input = hoverInput(config);
  ASSERT_TRUE(solver.solve(input).valid);
  input.target.position_ned_m.x() += 0.2;
  const WholeBodySolverOutput output = solver.solve(input);
  ASSERT_TRUE(output.valid) << output.solver_status;
  EXPECT_GT(std::abs(output.wrench_frd.x()), 1.0e-4);
  EXPECT_DOUBLE_EQ(
    input.state.attitude_ned_frd.angularDistance(Eigen::Quaterniond::Identity()), 0.0);
}

}  // namespace
}  // namespace flying_hand_mode::fully_actuated
