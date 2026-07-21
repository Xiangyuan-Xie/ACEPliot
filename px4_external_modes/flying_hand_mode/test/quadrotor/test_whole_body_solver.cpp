#include <flying_hand_mode/quadrotor/arm_kinematics.hpp>
#include <flying_hand_mode/quadrotor/whole_body_solver.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace flying_hand_mode::quadrotor
{
namespace
{

WholeBodySolverInput hoverInput()
{
  WholeBodySolverInput input;
  input.state.position_ned_m = Eigen::Vector3d(0.0, 0.0, -1.0);
  input.state.arm_position_rad << -1.5708, 3.1415, 0.0, 0.0;
  const VehicleMassProperties mass_properties =
    ArmKinematics{}.massPropertiesFlu(input.state.arm_position_rad);
  input.mass_properties.mass_kg = mass_properties.mass_kg;
  input.mass_properties.center_of_mass_flu_m = mass_properties.center_of_mass_flu_m;
  input.mass_properties.inertia_com_flu_kg_m2 =
    mass_properties.inertia_com_flu_kg_m2;
  input.target.position_ned_m = input.state.position_ned_m +
    Eigen::Vector3d(0.153390235794, -0.000701428823, 0.224250144130);
  input.previous_control[0] = mass_properties.mass_kg * 9.80665;
  input.previous_control.segment<kArmJointCount>(4) = input.state.arm_position_rad;
  return input;
}

TEST(WholeBodySolver, RejectsNonfiniteState)
{
  WholeBodySolver solver;
  WholeBodySolverInput input = hoverInput();
  input.state.position_ned_m.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(solver.solve(input).valid);
}

TEST(WholeBodySolver, ProducesFiniteFeasibleHoverCommand)
{
  WholeBodySolver solver;
  const WholeBodySolverOutput output = solver.solve(hoverInput());
  ASSERT_TRUE(output.valid) << "ACADOS status " << output.solver_status;
  EXPECT_GE(output.actuation_frd.thrust_up_n, 0.0);
  EXPECT_LE(output.actuation_frd.thrust_up_n, 100.0005);
  EXPECT_TRUE(output.actuation_frd.moment_b_nm.allFinite());
  EXPECT_TRUE(output.arm_position_command_rad.allFinite());
  EXPECT_GT(output.solver_time_s, 0.0);
}

TEST(WholeBodySolver, CanResetAndResolve)
{
  WholeBodySolver solver;
  EXPECT_TRUE(solver.solve(hoverInput()).valid);
  solver.reset();
  EXPECT_TRUE(solver.solve(hoverInput()).valid);
}

TEST(WholeBodySolver, RespondsToExactHalfTurnEndEffectorTarget)
{
  WholeBodySolver solver;
  const WholeBodySolverOutput baseline = solver.solve(hoverInput());
  ASSERT_TRUE(baseline.valid);

  solver.reset();
  WholeBodySolverInput rotated_input = hoverInput();
  rotated_input.target.attitude_ned =
    Eigen::Quaterniond(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
  const WholeBodySolverOutput rotated = solver.solve(rotated_input);

  ASSERT_TRUE(rotated.valid) << "ACADOS status " << rotated.solver_status;
  const double control_difference =
    (rotated.actuation_frd.vector() - baseline.actuation_frd.vector()).norm() +
    (rotated.arm_position_command_rad - baseline.arm_position_command_rad).norm();
  EXPECT_GT(control_difference, 1.0e-4);
}

TEST(WholeBodySolver, MeetsEightMillisecondSteadyStateBudget)
{
  WholeBodySolver solver;
  const WholeBodySolverInput input = hoverInput();
  for (int warmup = 0; warmup < 20; ++warmup) {
    ASSERT_TRUE(solver.solve(input).valid);
  }
  std::vector<double> solve_times;
  solve_times.reserve(1000);
  for (int sample = 0; sample < 1000; ++sample) {
    const WholeBodySolverOutput output = solver.solve(input);
    ASSERT_TRUE(output.valid) << "sample " << sample << " status " << output.solver_status;
    solve_times.push_back(output.solver_time_s);
  }
  std::sort(solve_times.begin(), solve_times.end());
  const std::size_t p99_index =
    static_cast<std::size_t>(std::ceil(0.99 * static_cast<double>(solve_times.size()))) - 1;
  const double p99 = solve_times[p99_index];
  RecordProperty("p99_solver_time_ms", p99 * 1.0e3);
  EXPECT_LT(p99, 0.008) << "p99 solver time was " << p99 * 1.0e3 << " ms";
}

}  // namespace
}  // namespace flying_hand_mode::quadrotor
