#include <flying_hand_fully_actuated_mode/fully_actuated_core.hpp>

#include <gtest/gtest.h>

#include <cmath>

namespace flying_hand_fully_actuated_mode
{
namespace
{

RotorGeometry tiltedHexGeometry()
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
  return geometry;
}

TEST(FullyActuatedAllocation, TiltedHexIsFullRankAndWellConditioned)
{
  const FullyActuatedAllocationModel model = buildAllocationModel(tiltedHexGeometry(), 50.0);
  ASSERT_TRUE(model.valid);
  EXPECT_LT(model.condition_number, 10.0);
  EXPECT_NEAR(
    (model.physical_allocation_frd * model.physical_inverse_frd -
    AllocationMatrix::Identity()).norm(),
    0.0, 1.0e-10);
}

TEST(FullyActuatedAllocation, VerticalHexIsRejectedAsUnderactuated)
{
  RotorGeometry geometry = tiltedHexGeometry();
  for (int rotor = 0; rotor < kRotorCount; ++rotor) {
    geometry.axis_frd.col(rotor) = Eigen::Vector3d(0.0, 0.0, -1.0);
  }
  EXPECT_FALSE(buildAllocationModel(geometry, 50.0).valid);
}

TEST(FullyActuatedAllocation, ProducesIndependentLateralForces)
{
  const FullyActuatedAllocationModel model = buildAllocationModel(tiltedHexGeometry(), 50.0);
  ASSERT_TRUE(model.valid);
  const RotorVector hover = RotorVector::Constant(8.0);
  const WrenchVector hover_wrench = model.physical_allocation_frd * hover;

  for (int axis = 0; axis < 2; ++axis) {
    WrenchVector requested = hover_wrench;
    requested[axis] += 1.0;
    const WrenchProjectionResult projected = projectWrenchToRotorBox(requested, model);
    ASSERT_TRUE(projected.valid);
    EXPECT_FALSE(projected.saturated);
    EXPECT_NEAR(projected.projected_wrench_frd[axis], requested[axis], 1.0e-9);
    EXPECT_NEAR(
      (projected.projected_wrench_frd - requested).norm(), 0.0, 1.0e-8);
  }
}

TEST(FullyActuatedAllocation, ProjectionRespectsEveryRotorLimit)
{
  const FullyActuatedAllocationModel model = buildAllocationModel(tiltedHexGeometry(), 50.0);
  WrenchVector requested = WrenchVector::Constant(1000.0);
  const WrenchProjectionResult projected = projectWrenchToRotorBox(requested, model);
  ASSERT_TRUE(projected.valid);
  EXPECT_TRUE(projected.saturated);
  EXPECT_TRUE((projected.rotor_thrust_n.array() >= -1.0e-9).all());
  EXPECT_TRUE((projected.rotor_thrust_n.array() <= 20.0 + 1.0e-9).all());
}

TEST(FullyActuatedAllocation, Px4MixerReconstructsRotorThrust)
{
  const FullyActuatedAllocationModel model = buildAllocationModel(tiltedHexGeometry(), 50.0);
  ASSERT_TRUE(model.valid);
  RotorThrustCommandModel thrust_model;
  thrust_model.maximum_thrust_n.setConstant(20.0);
  thrust_model.kappa.setConstant(0.3);
  const RotorVector thrust = RotorVector::Constant(8.0);
  const Px4NormalizedWrench normalized =
    rotorThrustToPx4Normalized(thrust, thrust_model, model);
  ASSERT_TRUE(normalized.valid);
  EXPECT_NEAR((normalized.realized_rotor_thrust_n - thrust).norm(), 0.0, 1.0e-9);
  EXPECT_LT(normalized.thrust_frd.z(), 0.0F);
}

}  // namespace
}  // namespace flying_hand_fully_actuated_mode
