#include <gtest/gtest.h>

#include <flying_hand_quadrotor_mode/controller_core.hpp>

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace flying_hand_quadrotor_mode
{
namespace
{

constexpr double kTolerance = 1.0e-9;
constexpr double kTwoPi = 6.28318530717958647692;
constexpr double kX500ArmLengthM = 0.17688;
constexpr double kX500YawMomentPerThrustM = 0.01296242524483617;
const Eigen::Vector2d kX500ComFluM{-0.01394428, 0.00035525};

template<int Size>
void expectVectorNear(
  const Eigen::Matrix<double, Size, 1> & actual,
  const Eigen::Matrix<double, Size, 1> & expected,
  double tolerance)
{
  for (int index = 0; index < Size; ++index) {
    EXPECT_NEAR(actual[index], expected[index], tolerance) << "index " << index;
  }
}

RotorAllocationModel makeX500Model()
{
  RotorAllocationModel model;
  // ACESim rotor order: (+x,-y), (-x,+y), (+x,+y), (-x,-y).
  const std::array<Eigen::Vector2d, kRotorCount> position_flu{{
    {kX500ArmLengthM, -kX500ArmLengthM},
    {-kX500ArmLengthM, kX500ArmLengthM},
    {kX500ArmLengthM, kX500ArmLengthM},
    {-kX500ArmLengthM, -kX500ArmLengthM},
  }};
  const std::array<double, kRotorCount> direction{{1.0, 1.0, -1.0, -1.0}};
  for (int rotor = 0; rotor < kRotorCount; ++rotor) {
    const Eigen::Vector2d arm_flu =
      position_flu[static_cast<std::size_t>(rotor)] - kX500ComFluM;
    model.allocation_flu(0, rotor) = 1.0;
    model.allocation_flu(1, rotor) = arm_flu.y();
    model.allocation_flu(2, rotor) = -arm_flu.x();
    model.allocation_flu(3, rotor) =
      -direction[static_cast<std::size_t>(rotor)] * kX500YawMomentPerThrustM;
  }
  model.maximum_thrust_n.setConstant(25.0);
  return model;
}

RotorAllocationModel makeCoupledModel()
{
  RotorAllocationModel model;
  model.allocation_flu.setZero();
  model.allocation_flu(0, 0) = 1.0;
  model.allocation_flu(0, 1) = 1.0;
  model.allocation_flu(1, 1) = 1.0;
  model.allocation_flu(2, 2) = 1.0;
  model.allocation_flu(3, 3) = 1.0;
  model.maximum_thrust_n.setOnes();
  return model;
}

L1AdaptiveConfig makeAdaptiveConfig()
{
  L1AdaptiveConfig config;
  config.uav.predictor_gain_rad_s.setConstant(20.0);
  config.uav.adaptation_gain.setConstant(100.0);
  config.uav.low_pass_cutoff_hz.setConstant(5.0);
  config.uav.wrench_correction_limit.setConstant(2.0);
  config.arm.servo_tau_s << 0.08, 0.10, 0.12, 0.15;
  config.arm.predictor_gain_rad_s.setConstant(20.0);
  config.arm.adaptation_gain_per_s2.setConstant(100.0);
  config.arm.low_pass_cutoff_hz.setConstant(5.0);
  config.arm.position_correction_limit_rad.setConstant(0.3);
  config.maximum_dt_s = 0.2;
  return config;
}

TEST(ControllerCoreData, FixedSizeStructsPreservePhysicalChannelOrder)
{
  WrenchVector wrench_vector;
  wrench_vector << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0;
  const PhysicalWrench wrench = PhysicalWrench::fromVector(wrench_vector);
  const BodyVelocity velocity = BodyVelocity::fromVector(wrench_vector);

  expectVectorNear<kWrenchDimension>(wrench.vector(), wrench_vector, 0.0);
  expectVectorNear<kWrenchDimension>(velocity.vector(), wrench_vector, 0.0);
  EXPECT_DOUBLE_EQ(wrench.force_b_n.z(), 3.0);
  EXPECT_DOUBLE_EQ(wrench.moment_b_nm.x(), 4.0);
  EXPECT_DOUBLE_EQ(velocity.linear_b_m_s.z(), 3.0);
  EXPECT_DOUBLE_EQ(velocity.angular_b_rad_s.x(), 4.0);

  QuadrotorActuationVector actuation_vector;
  actuation_vector << 7.0, 8.0, 9.0, 10.0;
  const QuadrotorActuation actuation = QuadrotorActuation::fromVector(actuation_vector);
  expectVectorNear<kQuadrotorActuationDimension>(
    actuation.vector(), actuation_vector, 0.0);
  EXPECT_DOUBLE_EQ(actuation.physicalWrench().force_b_n.x(), 0.0);
  EXPECT_DOUBLE_EQ(actuation.physicalWrench().force_b_n.y(), 0.0);
  EXPECT_DOUBLE_EQ(actuation.physicalWrench().force_b_n.z(), 7.0);
  expectVectorNear<3>(
    actuation.physicalWrench().moment_b_nm, Eigen::Vector3d(8.0, 9.0, 10.0), 0.0);
}

TEST(PhysicalWrenchProjection, UsesActualX500FluFourDimensionalAllocation)
{
  const RotorAllocationModel model = makeX500Model();

  EXPECT_DOUBLE_EQ(model.allocation_flu(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(
    model.allocation_flu(1, 0), -kX500ArmLengthM - kX500ComFluM.y());
  EXPECT_DOUBLE_EQ(
    model.allocation_flu(2, 0), -kX500ArmLengthM + kX500ComFluM.x());
  EXPECT_DOUBLE_EQ(model.allocation_flu(3, 0), -kX500YawMomentPerThrustM);
  EXPECT_DOUBLE_EQ(
    model.allocation_flu(1, 2), kX500ArmLengthM - kX500ComFluM.y());
  EXPECT_DOUBLE_EQ(
    model.allocation_flu(2, 2), -kX500ArmLengthM + kX500ComFluM.x());
  EXPECT_DOUBLE_EQ(model.allocation_flu(3, 2), kX500YawMomentPerThrustM);

  RotorVector expected_thrust;
  expected_thrust << 4.0, 5.0, 6.0, 7.0;
  const QuadrotorActuation desired =
    QuadrotorActuation::fromVector(model.allocation_flu * expected_thrust);
  const WrenchProjectionResult result = projectPhysicalWrench(desired, model);

  ASSERT_TRUE(result.valid);
  expectVectorNear<kRotorCount>(result.rotor_thrust_n, expected_thrust, kTolerance);
  expectVectorNear<kQuadrotorActuationDimension>(
    result.projected_actuation.vector(), desired.vector(), kTolerance);
  EXPECT_NEAR(result.weighted_error_squared, 0.0, kTolerance);
  for (const RotorConstraint constraint : result.constraints) {
    EXPECT_EQ(constraint, RotorConstraint::kFree);
  }
}

TEST(PhysicalWrenchProjection, SaturatesCollectiveAtAllRotorUpperBounds)
{
  const RotorAllocationModel model = makeX500Model();
  QuadrotorActuation desired;
  desired.thrust_up_n = 1000.0;

  const WrenchProjectionResult result = projectPhysicalWrench(desired, model);

  ASSERT_TRUE(result.valid);
  expectVectorNear<kRotorCount>(
    result.rotor_thrust_n, RotorVector::Constant(25.0), kTolerance);
  const QuadrotorActuation expected = QuadrotorActuation::fromVector(
    model.allocation_flu * RotorVector::Constant(25.0));
  expectVectorNear<kQuadrotorActuationDimension>(
    result.projected_actuation.vector(), expected.vector(), kTolerance);
  for (const RotorConstraint constraint : result.constraints) {
    EXPECT_EQ(constraint, RotorConstraint::kUpper);
  }
}

TEST(PhysicalWrenchProjection, SolvesCoupledBoxProblemInsteadOfClippingRotorSolution)
{
  const RotorAllocationModel model = makeCoupledModel();
  QuadrotorActuationVector desired_vector = QuadrotorActuationVector::Zero();
  desired_vector[0] = -0.5;
  desired_vector[1] = 0.5;

  const WrenchProjectionResult result = projectPhysicalWrench(
    QuadrotorActuation::fromVector(desired_vector), model);

  ASSERT_TRUE(result.valid);
  expectVectorNear<kRotorCount>(result.rotor_thrust_n, RotorVector::Zero(), kTolerance);
  EXPECT_NEAR(result.weighted_error_squared, 0.5, kTolerance);

  RotorVector clipped_unconstrained = RotorVector::Zero();
  clipped_unconstrained[1] = 0.5;
  const double clipped_cost =
    (model.allocation_flu * clipped_unconstrained - desired_vector).squaredNorm();
  EXPECT_LT(result.weighted_error_squared, clipped_cost);
}

TEST(PhysicalWrenchProjection, HonorsActuationMetricWeights)
{
  RotorAllocationModel model = makeCoupledModel();
  model.actuation_weights[1] = 2.0;
  QuadrotorActuationVector desired_vector = QuadrotorActuationVector::Zero();
  desired_vector[0] = -0.5;
  desired_vector[1] = 0.5;

  const WrenchProjectionResult result = projectPhysicalWrench(
    QuadrotorActuation::fromVector(desired_vector), model);

  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.rotor_thrust_n[0], 0.0, kTolerance);
  EXPECT_NEAR(result.rotor_thrust_n[1], 0.3, kTolerance);
  EXPECT_NEAR(result.rotor_thrust_n[2], 0.0, kTolerance);
  EXPECT_NEAR(result.rotor_thrust_n[3], 0.0, kTolerance);
  EXPECT_NEAR(result.weighted_error_squared, 0.8, kTolerance);
}

TEST(PhysicalWrenchProjection, ResultSatisfiesBoxKktConditions)
{
  RotorAllocationModel model = makeX500Model();
  model.maximum_thrust_n.setConstant(8.0);
  model.actuation_weights << 1.0, 3.0, 3.0, 12.0;
  QuadrotorActuation desired;
  desired.thrust_up_n = 18.0;
  desired.moment_b_nm << 3.0, -2.0, 0.8;

  const WrenchProjectionResult result = projectPhysicalWrench(desired, model);

  ASSERT_TRUE(result.valid);
  const QuadrotorActuationVector residual =
    model.allocation_flu * result.rotor_thrust_n - desired.vector();
  const QuadrotorActuationVector weighted_residual =
    model.actuation_weights.array().square().matrix().cwiseProduct(residual);
  const RotorVector gradient = model.allocation_flu.transpose() * weighted_residual;
  for (int rotor = 0; rotor < kRotorCount; ++rotor) {
    EXPECT_GE(result.rotor_thrust_n[rotor], model.minimum_thrust_n[rotor] - kTolerance);
    EXPECT_LE(result.rotor_thrust_n[rotor], model.maximum_thrust_n[rotor] + kTolerance);
    if (result.constraints[rotor] == RotorConstraint::kLower) {
      EXPECT_GE(gradient[rotor], -kTolerance);
    } else if (result.constraints[rotor] == RotorConstraint::kUpper) {
      EXPECT_LE(gradient[rotor], kTolerance);
    } else {
      EXPECT_NEAR(gradient[rotor], 0.0, kTolerance);
    }
  }
}

TEST(PhysicalWrenchProjection, RejectsInvalidOrNonfiniteInputs)
{
  RotorAllocationModel model = makeX500Model();
  QuadrotorActuation desired;

  model.maximum_thrust_n[2] = -1.0;
  EXPECT_FALSE(projectPhysicalWrench(desired, model).valid);

  model = makeX500Model();
  model.actuation_weights[0] = 0.0;
  EXPECT_FALSE(projectPhysicalWrench(desired, model).valid);

  model = makeX500Model();
  desired.moment_b_nm.y() = std::numeric_limits<double>::quiet_NaN();
  const WrenchProjectionResult result = projectPhysicalWrench(desired, model);
  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(result.rotor_thrust_n.isZero());
}

TEST(Px4AllocatorConversion, MatchesPx4NormalizedXQuadMixer)
{
  Px4AllocatorGeometry geometry;
  geometry.rotor_position_xy_frd.row(0) =
    Eigen::RowVector4d(1.0, -1.0, 1.0, -1.0);
  geometry.rotor_position_xy_frd.row(1) =
    Eigen::RowVector4d(1.0, -1.0, -1.0, 1.0);
  geometry.moment_ratio = (RotorVector() << 0.05, 0.05, -0.05, -0.05).finished();

  const Px4AllocatorModel allocator = buildPx4NormalizedAllocator(geometry);

  ASSERT_TRUE(allocator.valid);
  const double roll_pitch = std::sqrt(0.5);
  expectVectorNear<kRotorCount>(
    allocator.actuator_from_control.col(0),
    (RotorVector() << -roll_pitch, roll_pitch, roll_pitch, -roll_pitch).finished(),
    kTolerance);
  expectVectorNear<kRotorCount>(
    allocator.actuator_from_control.col(1),
    (RotorVector() << roll_pitch, -roll_pitch, roll_pitch, -roll_pitch).finished(),
    kTolerance);
  expectVectorNear<kRotorCount>(
    allocator.actuator_from_control.col(2),
    (RotorVector() << 1.0, 1.0, -1.0, -1.0).finished(), kTolerance);
  expectVectorNear<kRotorCount>(
    allocator.actuator_from_control.col(3), RotorVector::Constant(-1.0), kTolerance);
}

TEST(Px4AllocatorConversion, ReconstructsCalibratedRotorThrustAfterPx4Allocation)
{
  Px4AllocatorGeometry geometry;
  geometry.rotor_position_xy_frd.row(0) =
    Eigen::RowVector4d(0.1515, -0.1515, 0.1515, -0.1515);
  geometry.rotor_position_xy_frd.row(1) =
    Eigen::RowVector4d(0.245, -0.1875, -0.245, 0.1875);
  geometry.moment_ratio = (RotorVector() << 0.05, 0.05, -0.05, -0.05).finished();
  const Px4AllocatorModel allocator = buildPx4NormalizedAllocator(geometry);
  const RotorThrustCommandModel thrust_model{25.000097317747848, 0.3259609459890776};
  const RotorVector thrust = (RotorVector() << 4.0, 5.0, 6.0, 7.0).finished();

  const Px4NormalizedWrench result =
    rotorThrustToPx4Normalized(thrust, thrust_model, allocator);

  ASSERT_TRUE(result.valid);
  EXPECT_FALSE(result.saturated);
  Px4ControlVector control;
  control.head<3>() = result.torque_frd.cast<double>();
  control[3] = result.thrust_frd.z();
  expectVectorNear<kRotorCount>(
    allocator.actuator_from_control * control, result.actuator_command, 1.0e-7);

  const RotorVector normalized_speed =
    thrust_model.kappa * result.actuator_command +
    (1.0 - thrust_model.kappa) * result.actuator_command.array().sqrt().matrix();
  const RotorVector reconstructed_thrust =
    thrust_model.maximum_thrust_n * normalized_speed.array().square().matrix();
  expectVectorNear<kRotorCount>(reconstructed_thrust, thrust, 1.0e-7);
}

TEST(Px4AllocatorConversion, ProjectsRotorCornerIntoPx4ControlAndActuatorBoxes)
{
  Px4AllocatorGeometry geometry;
  geometry.rotor_position_xy_frd.row(0) =
    Eigen::RowVector4d(0.1515, -0.1515, 0.1515, -0.1515);
  geometry.rotor_position_xy_frd.row(1) =
    Eigen::RowVector4d(0.245, -0.1875, -0.245, 0.1875);
  geometry.moment_ratio = (RotorVector() << 0.05, 0.05, -0.05, -0.05).finished();
  const Px4AllocatorModel allocator = buildPx4NormalizedAllocator(geometry);
  const RotorThrustCommandModel thrust_model{25.000097317747848, 0.3259609459890776};
  const RotorVector requested_actuator =
    (RotorVector() << 1.0, 0.0, 0.0, 1.0).finished();
  const RotorVector requested_speed =
    thrust_model.kappa * requested_actuator +
    (1.0 - thrust_model.kappa) * requested_actuator.array().sqrt().matrix();
  const RotorVector requested_thrust =
    thrust_model.maximum_thrust_n * requested_speed.array().square().matrix();

  const Px4NormalizedWrench result =
    rotorThrustToPx4Normalized(requested_thrust, thrust_model, allocator);

  ASSERT_TRUE(result.valid);
  EXPECT_TRUE(result.saturated);
  EXPECT_LE(result.torque_frd.cast<double>().cwiseAbs().maxCoeff(), 1.0);
  EXPECT_GE(result.thrust_frd.z(), -1.0F);
  EXPECT_LE(result.thrust_frd.z(), 1.0F);
  EXPECT_TRUE((result.actuator_command.array() >= 0.0).all());
  EXPECT_TRUE((result.actuator_command.array() <= 1.0).all());
  const RotorVector realized_speed =
    thrust_model.kappa * result.actuator_command +
    (1.0 - thrust_model.kappa) * result.actuator_command.array().sqrt().matrix();
  const RotorVector realized_thrust =
    thrust_model.maximum_thrust_n * realized_speed.array().square().matrix();
  expectVectorNear<kRotorCount>(
    result.realized_rotor_thrust_n, realized_thrust, 1.0e-9);
}

TEST(Px4AllocatorConversion, RejectsInvalidGeometryCurveAndThrust)
{
  Px4AllocatorGeometry geometry;
  EXPECT_FALSE(buildPx4NormalizedAllocator(geometry).valid);

  geometry.rotor_position_xy_frd.row(0) =
    Eigen::RowVector4d(1.0, -1.0, 1.0, -1.0);
  geometry.rotor_position_xy_frd.row(1) =
    Eigen::RowVector4d(1.0, -1.0, -1.0, 1.0);
  geometry.moment_ratio = (RotorVector() << 0.05, 0.05, -0.05, -0.05).finished();
  const Px4AllocatorModel allocator = buildPx4NormalizedAllocator(geometry);
  ASSERT_TRUE(allocator.valid);

  RotorThrustCommandModel thrust_model{25.0, -0.1};
  EXPECT_FALSE(rotorThrustToPx4Normalized(RotorVector::Ones(), thrust_model, allocator).valid);
  thrust_model.kappa = 0.5;
  RotorVector invalid_thrust = RotorVector::Ones();
  invalid_thrust[0] = 26.0;
  EXPECT_FALSE(rotorThrustToPx4Normalized(invalid_thrust, thrust_model, allocator).valid);
}

TEST(L1AdaptiveControllerTest, UsesActualDtForUavAndArmServoFilters)
{
  L1AdaptiveConfig config = makeAdaptiveConfig();
  config.uav.adaptation_gain.setConstant(2.0);
  config.uav.low_pass_cutoff_hz.setConstant(1.5);
  config.uav.wrench_correction_limit.setConstant(10.0);
  config.arm.adaptation_gain_per_s2.setConstant(3.0);
  config.arm.low_pass_cutoff_hz.setConstant(2.0);
  config.arm.position_correction_limit_rad.setConstant(10.0);
  config.maximum_dt_s = 1.0;
  L1AdaptiveController controller(config);
  L1AdaptiveInput input;

  ASSERT_TRUE(controller.update(input, 0.01).valid);
  WrenchVector measured_body_velocity = WrenchVector::Zero();
  measured_body_velocity[0] = 1.0;
  input.measured_body_velocity = BodyVelocity::fromVector(measured_body_velocity);
  input.measured_arm_position_rad[0] = 1.0;
  constexpr double kDt = 0.2;

  const L1AdaptiveOutput output = controller.update(input, kDt);

  const double expected_uav_estimate = config.uav.adaptation_gain[0] * kDt;
  const double uav_alpha =
    1.0 - std::exp(-kTwoPi * config.uav.low_pass_cutoff_hz[0] * kDt);
  const double expected_arm_estimate_rad_s =
    config.arm.adaptation_gain_per_s2[0] * kDt;
  const double arm_alpha =
    1.0 - std::exp(-kTwoPi * config.arm.low_pass_cutoff_hz[0] * kDt);

  ASSERT_TRUE(output.valid);
  EXPECT_NEAR(
    output.uav_disturbance_estimate.force_b_n.x(), expected_uav_estimate, kTolerance);
  EXPECT_NEAR(
    output.uav_wrench_correction.force_b_n.x(),
    -uav_alpha * expected_uav_estimate, kTolerance);
  EXPECT_NEAR(
    output.arm_disturbance_estimate_rad_s[0], expected_arm_estimate_rad_s, kTolerance);
  EXPECT_NEAR(
    output.arm_position_correction_rad[0],
    -config.arm.servo_tau_s[0] * arm_alpha * expected_arm_estimate_rad_s, kTolerance);
}

TEST(L1AdaptiveControllerTest, UavChannelsRemainPhysicalForceAndMoment)
{
  L1AdaptiveConfig config = makeAdaptiveConfig();
  config.uav.adaptation_gain.setOnes();
  config.uav.low_pass_cutoff_hz.setConstant(100.0);
  config.uav.wrench_correction_limit.setConstant(100.0);
  L1AdaptiveController controller(config);
  L1AdaptiveInput input;
  ASSERT_TRUE(controller.update(input, 0.01).valid);

  WrenchVector measured;
  measured << 1.0, -2.0, 3.0, -4.0, 5.0, -6.0;
  input.measured_body_velocity = BodyVelocity::fromVector(measured);
  constexpr double kDt = 0.05;
  const L1AdaptiveOutput output = controller.update(input, kDt);

  ASSERT_TRUE(output.valid);
  expectVectorNear<kWrenchDimension>(
    output.uav_disturbance_estimate.vector(), measured * kDt, kTolerance);
  EXPECT_DOUBLE_EQ(output.uav_disturbance_estimate.force_b_n.z(), 3.0 * kDt);
  EXPECT_DOUBLE_EQ(output.uav_disturbance_estimate.moment_b_nm.x(), -4.0 * kDt);
  for (int channel = 0; channel < kWrenchDimension; ++channel) {
    EXPECT_EQ(
      std::signbit(output.uav_wrench_correction.vector()[channel]),
      std::signbit(-measured[channel])) << "channel " << channel;
  }
}

TEST(L1AdaptiveControllerTest, UsesFullDynamicInertiaGainForMomentAdaptation)
{
  L1AdaptiveConfig config = makeAdaptiveConfig();
  config.uav.adaptation_gain.setOnes();
  config.uav.low_pass_cutoff_hz.setZero();
  config.uav.wrench_correction_limit.setConstant(100.0);
  L1AdaptiveController controller(config);
  L1AdaptiveInput input;
  input.uav_input_gain.bottomRightCorner<3, 3>() <<
    40.0, 0.0, -5.0,
    0.0, 12.0, 0.0,
    -5.0, 0.0, 10.0;
  ASSERT_TRUE(controller.update(input, 0.01).valid);

  WrenchVector measured = WrenchVector::Zero();
  measured[5] = 1.0;
  input.measured_body_velocity = BodyVelocity::fromVector(measured);
  constexpr double kDt = 0.02;
  const L1AdaptiveOutput output = controller.update(input, kDt);

  ASSERT_TRUE(output.valid);
  EXPECT_NEAR(output.uav_disturbance_estimate.moment_b_nm.x(), -5.0 * kDt, kTolerance);
  EXPECT_NEAR(output.uav_disturbance_estimate.moment_b_nm.y(), 0.0, kTolerance);
  EXPECT_NEAR(output.uav_disturbance_estimate.moment_b_nm.z(), 10.0 * kDt, kTolerance);
}

TEST(L1AdaptiveControllerTest, MapsArmVelocityDisturbanceThroughEachServoTau)
{
  L1AdaptiveConfig config = makeAdaptiveConfig();
  config.arm.adaptation_gain_per_s2.setOnes();
  config.arm.low_pass_cutoff_hz.setConstant(10.0);
  config.arm.position_correction_limit_rad.setConstant(100.0);
  config.arm.servo_tau_s << 0.05, 0.10, 0.15, 0.20;
  L1AdaptiveController controller(config);
  L1AdaptiveInput input;
  ASSERT_TRUE(controller.update(input, 0.01).valid);
  input.measured_arm_position_rad.setOnes();
  constexpr double kDt = 0.05;

  const L1AdaptiveOutput output = controller.update(input, kDt);

  const double alpha = 1.0 - std::exp(-kTwoPi * 10.0 * kDt);
  const JointVector expected_estimate = JointVector::Constant(kDt);
  const JointVector expected_correction =
    -alpha * config.arm.servo_tau_s.cwiseProduct(expected_estimate);
  ASSERT_TRUE(output.valid);
  expectVectorNear<kArmJointCount>(
    output.arm_disturbance_estimate_rad_s, expected_estimate, kTolerance);
  expectVectorNear<kArmJointCount>(
    output.arm_position_correction_rad, expected_correction, kTolerance);
  expectVectorNear<kArmJointCount>(
    output.arm_position_command_rad, expected_correction, kTolerance);
}

TEST(L1AdaptiveControllerTest, LearnsUavAndServoDisturbancesWithVariableDt)
{
  L1AdaptiveConfig config = makeAdaptiveConfig();
  config.uav.predictor_gain_rad_s.setConstant(25.0);
  config.uav.adaptation_gain.setConstant(120.0);
  config.uav.low_pass_cutoff_hz.setConstant(4.0);
  WrenchGainMatrix input_gain = WrenchGainMatrix::Identity();
  input_gain.diagonal() << 0.8, 1.0, 1.2, 2.0, 1.5, 1.0;
  config.arm.predictor_gain_rad_s.setConstant(25.0);
  config.arm.adaptation_gain_per_s2.setConstant(120.0);
  config.arm.low_pass_cutoff_hz.setConstant(4.0);
  L1AdaptiveController controller(config);

  WrenchVector uav_disturbance = WrenchVector::Zero();
  uav_disturbance[2] = 0.8;
  uav_disturbance[4] = -0.25;
  JointVector arm_disturbance_rad_s;
  arm_disturbance_rad_s << 0.4, -0.2, 0.0, 0.15;
  WrenchVector body_velocity = WrenchVector::Zero();
  JointVector nominal_arm_position;
  nominal_arm_position << 0.2, -0.1, 0.3, -0.2;
  JointVector arm_position = nominal_arm_position;
  L1AdaptiveOutput output;
  const std::array<double, 4> time_steps{0.004, 0.007, 0.011, 0.006};

  for (int step = 0; step < 1600; ++step) {
    const double dt_s = time_steps[static_cast<std::size_t>(step) % time_steps.size()];
    L1AdaptiveInput input;
    input.uav_input_gain = input_gain;
    input.measured_body_velocity = BodyVelocity::fromVector(body_velocity);
    input.measured_arm_position_rad = arm_position;
    input.nominal_arm_position_rad = nominal_arm_position;
    output = controller.update(input, dt_s);
    ASSERT_TRUE(output.valid) << "step " << step;
    body_velocity += dt_s * input_gain *
      (output.uav_wrench_command.vector() + uav_disturbance);
    arm_position.array() += dt_s *
      ((output.arm_position_command_rad - arm_position).array() /
      config.arm.servo_tau_s.array() + arm_disturbance_rad_s.array());
  }

  expectVectorNear<kWrenchDimension>(
    output.uav_disturbance_estimate.vector(), uav_disturbance, 2.0e-2);
  expectVectorNear<kWrenchDimension>(
    output.uav_wrench_correction.vector(), -uav_disturbance, 2.0e-2);
  expectVectorNear<kArmJointCount>(
    output.arm_disturbance_estimate_rad_s, arm_disturbance_rad_s, 2.0e-2);
  const JointVector expected_arm_correction =
    -config.arm.servo_tau_s.cwiseProduct(arm_disturbance_rad_s);
  expectVectorNear<kArmJointCount>(
    output.arm_position_correction_rad, expected_arm_correction, 3.0e-3);
  expectVectorNear<kArmJointCount>(arm_position, nominal_arm_position, 3.0e-3);
}

TEST(L1AdaptiveControllerTest, LeadsACommandToCompensateExtraServoDelay)
{
  L1AdaptiveConfig config = makeAdaptiveConfig();
  config.arm.servo_tau_s.setConstant(0.08);
  config.arm.predictor_gain_rad_s.setConstant(20.0);
  config.arm.adaptation_gain_per_s2.setConstant(80.0);
  config.arm.low_pass_cutoff_hz.setConstant(3.0);
  config.arm.position_correction_limit_rad.setConstant(0.2);
  L1AdaptiveController controller(config);

  constexpr double kActualServoTauS = 0.25;
  constexpr double kCommandVelocityRadS = 0.2;
  constexpr double kDt = 0.01;
  JointVector arm_position = JointVector::Zero();
  JointVector baseline_position = JointVector::Zero();
  L1AdaptiveOutput output;
  double nominal_position = 0.0;

  for (int step = 0; step < 400; ++step) {
    nominal_position = kCommandVelocityRadS * static_cast<double>(step) * kDt;
    L1AdaptiveInput input;
    input.measured_arm_position_rad = arm_position;
    input.nominal_arm_position_rad[0] = nominal_position;
    output = controller.update(input, kDt);
    ASSERT_TRUE(output.valid);
    arm_position[0] +=
      kDt * (output.arm_position_command_rad[0] - arm_position[0]) / kActualServoTauS;
    baseline_position[0] +=
      kDt * (nominal_position - baseline_position[0]) / kActualServoTauS;
  }

  EXPECT_GT(output.arm_position_correction_rad[0], 0.0);
  EXPECT_LE(
    output.arm_position_correction_rad[0],
    config.arm.position_correction_limit_rad[0] + kTolerance);
  EXPECT_LT(
    std::abs(nominal_position - arm_position[0]),
    std::abs(nominal_position - baseline_position[0]));
}

TEST(L1AdaptiveControllerTest, CapsUavWrenchAndArmPositionCorrectionsSeparately)
{
  L1AdaptiveConfig config = makeAdaptiveConfig();
  config.uav.adaptation_gain.setConstant(1000.0);
  config.uav.low_pass_cutoff_hz.setConstant(100.0);
  config.uav.wrench_correction_limit << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
  config.arm.adaptation_gain_per_s2.setConstant(1000.0);
  config.arm.low_pass_cutoff_hz.setConstant(100.0);
  config.arm.position_correction_limit_rad << 0.01, 0.02, 0.03, 0.04;
  L1AdaptiveController controller(config);
  L1AdaptiveInput input;
  ASSERT_TRUE(controller.update(input, 0.01).valid);
  input.measured_body_velocity = BodyVelocity::fromVector(WrenchVector::Constant(100.0));
  input.measured_arm_position_rad.setConstant(100.0);

  const L1AdaptiveOutput output = controller.update(input, 0.05);

  ASSERT_TRUE(output.valid);
  expectVectorNear<kWrenchDimension>(
    output.uav_disturbance_estimate.vector(),
    config.uav.wrench_correction_limit, kTolerance);
  expectVectorNear<kWrenchDimension>(
    output.uav_wrench_correction.vector(),
    -config.uav.wrench_correction_limit, 1.0e-8);
  const JointVector expected_arm_disturbance_limit =
    config.arm.position_correction_limit_rad.cwiseQuotient(config.arm.servo_tau_s);
  expectVectorNear<kArmJointCount>(
    output.arm_disturbance_estimate_rad_s, expected_arm_disturbance_limit, kTolerance);
  expectVectorNear<kArmJointCount>(
    output.arm_position_correction_rad,
    -config.arm.position_correction_limit_rad, 1.0e-8);
}

TEST(L1AdaptiveControllerTest, ExplicitResetClearsBothAdaptiveModels)
{
  L1AdaptiveConfig config = makeAdaptiveConfig();
  config.uav.low_pass_cutoff_hz.setConstant(100.0);
  config.arm.low_pass_cutoff_hz.setConstant(100.0);
  L1AdaptiveController controller(config);
  L1AdaptiveInput input;
  input.nominal_uav_wrench = PhysicalWrench::fromVector(WrenchVector::Constant(0.25));
  input.nominal_arm_position_rad << 0.1, 0.2, 0.3, 0.4;
  ASSERT_TRUE(controller.update(input, 0.01).valid);
  input.measured_body_velocity = BodyVelocity::fromVector(WrenchVector::Ones());
  input.measured_arm_position_rad.setOnes();
  const L1AdaptiveOutput adapted = controller.update(input, 0.05);
  ASSERT_FALSE(adapted.uav_wrench_correction.vector().isZero());
  ASSERT_FALSE(adapted.arm_position_correction_rad.isZero());

  controller.reset();
  EXPECT_FALSE(controller.initialized());
  const L1AdaptiveOutput output = controller.update(input, 0.01);

  ASSERT_TRUE(output.valid);
  EXPECT_TRUE(controller.initialized());
  EXPECT_TRUE(output.uav_wrench_correction.vector().isZero());
  EXPECT_TRUE(output.uav_disturbance_estimate.vector().isZero());
  EXPECT_TRUE(output.arm_position_correction_rad.isZero());
  EXPECT_TRUE(output.arm_disturbance_estimate_rad_s.isZero());
  expectVectorNear<kWrenchDimension>(
    output.uav_wrench_command.vector(), input.nominal_uav_wrench.vector(), 0.0);
  expectVectorNear<kArmJointCount>(
    output.arm_position_command_rad, input.nominal_arm_position_rad, 0.0);
}

TEST(L1AdaptiveControllerTest, PredictorCanUseAcceptedProjectedCommand)
{
  L1AdaptiveConfig config = makeAdaptiveConfig();
  config.uav.predictor_gain_rad_s.setConstant(10.0);
  config.uav.adaptation_gain.setZero();
  config.uav.low_pass_cutoff_hz.setZero();
  L1AdaptiveController low_command_controller(config);
  L1AdaptiveController requested_command_controller(config);
  L1AdaptiveInput input;
  input.nominal_uav_wrench.force_b_n.z() = 10.0;

  ASSERT_TRUE(low_command_controller.update(input, 0.01).valid);
  ASSERT_TRUE(requested_command_controller.update(input, 0.01).valid);
  PhysicalWrench accepted;
  accepted.force_b_n.z() = 2.0;
  ASSERT_TRUE(low_command_controller.applyAcceptedCommand(accepted, JointVector::Zero()));
  accepted.force_b_n.z() = 10.0;
  ASSERT_TRUE(
    requested_command_controller.applyAcceptedCommand(
      accepted, JointVector::Zero()));

  const L1AdaptiveOutput projected_next = low_command_controller.update(input, 0.01);
  const L1AdaptiveOutput requested_next = requested_command_controller.update(input, 0.01);
  ASSERT_TRUE(projected_next.valid);
  ASSERT_TRUE(requested_next.valid);
  EXPECT_LT(
    projected_next.predicted_body_velocity.linear_b_m_s.z(),
    requested_next.predicted_body_velocity.linear_b_m_s.z());
}

TEST(L1AdaptiveControllerTest, InvalidSampleResetsAndPassesThroughFiniteNominals)
{
  L1AdaptiveController controller(makeAdaptiveConfig());
  L1AdaptiveInput input;
  input.nominal_uav_wrench = PhysicalWrench::fromVector(WrenchVector::Constant(0.3));
  input.nominal_arm_position_rad << 0.1, 0.2, 0.3, 0.4;
  ASSERT_TRUE(controller.update(input, 0.01).valid);
  input.measured_arm_position_rad[2] = std::numeric_limits<double>::quiet_NaN();

  L1AdaptiveOutput output = controller.update(input, 0.01);

  EXPECT_FALSE(output.valid);
  EXPECT_TRUE(output.state_was_reset);
  EXPECT_FALSE(controller.initialized());
  EXPECT_TRUE(output.uav_wrench_correction.vector().isZero());
  EXPECT_TRUE(output.arm_position_correction_rad.isZero());
  expectVectorNear<kWrenchDimension>(
    output.uav_wrench_command.vector(), input.nominal_uav_wrench.vector(), 0.0);
  expectVectorNear<kArmJointCount>(
    output.arm_position_command_rad, input.nominal_arm_position_rad, 0.0);

  input.measured_arm_position_rad.setZero();
  output = controller.update(input, 1.0);
  EXPECT_FALSE(output.valid);
  EXPECT_TRUE(output.state_was_reset);

  input.nominal_uav_wrench.force_b_n.z() = std::numeric_limits<double>::infinity();
  output = controller.update(input, 0.01);
  EXPECT_FALSE(output.valid);
  EXPECT_TRUE(output.uav_wrench_command.vector().isZero());
  expectVectorNear<kArmJointCount>(
    output.arm_position_command_rad, input.nominal_arm_position_rad, 0.0);
}

TEST(L1AdaptiveControllerTest, RejectsInvalidUavOrServoConfiguration)
{
  L1AdaptiveConfig config = makeAdaptiveConfig();
  config.uav.wrench_correction_limit[5] = -1.0;
  EXPECT_FALSE(L1AdaptiveController::isValidConfig(config));

  config = makeAdaptiveConfig();
  config.arm.servo_tau_s[1] = 0.0;
  EXPECT_FALSE(L1AdaptiveController::isValidConfig(config));

  config = makeAdaptiveConfig();
  config.arm.position_correction_limit_rad[2] = -0.1;
  EXPECT_FALSE(L1AdaptiveController::isValidConfig(config));

  config = makeAdaptiveConfig();
  config.arm.low_pass_cutoff_hz[2] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(L1AdaptiveController::isValidConfig(config));

  config = makeAdaptiveConfig();
  config.maximum_dt_s = 0.0;
  EXPECT_FALSE(L1AdaptiveController::isValidConfig(config));
}

TEST(L1AdaptiveControllerTest, RejectsNonpositiveOrNonsymmetricDynamicInputGain)
{
  L1AdaptiveController controller(makeAdaptiveConfig());
  L1AdaptiveInput input;
  input.uav_input_gain(5, 5) = 0.0;
  EXPECT_FALSE(controller.update(input, 0.01).valid);

  input.uav_input_gain.setIdentity();
  input.uav_input_gain(0, 1) = 0.1;
  EXPECT_FALSE(controller.update(input, 0.01).valid);
}

}  // namespace
}  // namespace flying_hand_quadrotor_mode
