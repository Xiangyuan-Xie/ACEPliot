#include <flying_hand_mode/runtime/external_mode.hpp>
#include <flying_hand_mode/runtime/topic_utils.hpp>

#include <gtest/gtest.h>

#include <limits>

namespace flying_hand_mode::runtime
{
namespace
{

TEST(TopicPrefix, PreservesAbsoluteTopicWithoutPrefix)
{
  EXPECT_EQ(joinTopicPrefix("", "/fmu/out/vehicle_odometry"), "/fmu/out/vehicle_odometry");
}

TEST(TopicPrefix, JoinsAbsoluteTopicWithoutDuplicateSeparator)
{
  EXPECT_EQ(
    joinTopicPrefix("/vehicle_1/", "/fmu/out/vehicle_odometry"),
    "/vehicle_1/fmu/out/vehicle_odometry");
  EXPECT_EQ(
    joinTopicPrefix("/vehicle_1", "fmu/out/vehicle_odometry"),
    "/vehicle_1/fmu/out/vehicle_odometry");
}

SafetySnapshot readySnapshot()
{
  SafetySnapshot snapshot;
  snapshot.closed_loop = true;
  snapshot.controller_available = true;
  snapshot.controller_output_ready = true;
  snapshot.vehicle_status_received = true;
  snapshot.armed = true;
  snapshot.rotary_wing = true;
  snapshot.land_state_received = true;
  snapshot.airborne = true;
  snapshot.odometry_valid = true;
  snapshot.imu_valid = true;
  snapshot.arm_state_valid = true;
  snapshot.follower_ready = true;
  snapshot.odometry_age_s = 0.005;
  snapshot.imu_age_s = 0.005;
  snapshot.arm_state_age_s = 0.005;
  snapshot.controller_output_age_s = 0.005;
  return snapshot;
}

TEST(FlyingHandSafety, AcceptsOnlyCompleteAirborneState)
{
  FlyingHandSafety safety(0.02, 0.008, 3);
  SafetySnapshot snapshot = readySnapshot();
  EXPECT_EQ(safety.readiness(snapshot), FaultReason::kNone);

  snapshot.closed_loop = false;
  EXPECT_EQ(safety.readiness(snapshot), FaultReason::kShadowOnly);
  snapshot = readySnapshot();
  snapshot.calibration_confirmed = false;
  EXPECT_EQ(safety.readiness(snapshot), FaultReason::kCalibrationUnconfirmed);
  snapshot = readySnapshot();
  snapshot.controller_output_ready = false;
  EXPECT_EQ(safety.readiness(snapshot), FaultReason::kControllerOutputMissing);
  snapshot = readySnapshot();
  snapshot.armed = false;
  EXPECT_EQ(safety.readiness(snapshot), FaultReason::kNotArmed);
  snapshot = readySnapshot();
  snapshot.airborne = false;
  EXPECT_EQ(safety.readiness(snapshot), FaultReason::kNotAirborne);
  snapshot = readySnapshot();
  snapshot.external_command_publisher = true;
  EXPECT_EQ(safety.readiness(snapshot), FaultReason::kExternalCommandPublisher);
}

TEST(FlyingHandSafety, EnforcesAllocatorFeedbackWhenRequested)
{
  FlyingHandSafety safety(0.02, 0.008, 3);
  SafetySnapshot snapshot = readySnapshot();
  snapshot.allocator_status_required = true;
  EXPECT_EQ(safety.readiness(snapshot), FaultReason::kAllocatorStatusMissing);

  snapshot.allocator_status_received = true;
  snapshot.allocator_status_age_s = 0.005;
  EXPECT_EQ(safety.readiness(snapshot), FaultReason::kNone);

  snapshot.allocator_setpoint_achieved = false;
  EXPECT_EQ(safety.readiness(snapshot), FaultReason::kAllocatorUnachieved);

  snapshot.allocator_setpoint_achieved = true;
  snapshot.allocator_saturated = true;
  EXPECT_EQ(safety.readiness(snapshot), FaultReason::kAllocatorSaturated);
}

TEST(FlyingHandSafety, RejectsStaleStateAtTwentyMilliseconds)
{
  FlyingHandSafety safety(0.02, 0.008, 3);
  SafetySnapshot snapshot = readySnapshot();
  snapshot.odometry_age_s = 0.021;
  EXPECT_EQ(safety.readiness(snapshot), FaultReason::kOdometryStale);

  snapshot = readySnapshot();
  snapshot.controller_output_age_s = 0.021;
  EXPECT_EQ(safety.readiness(snapshot), FaultReason::kControllerOutputStale);

  snapshot = readySnapshot();
  snapshot.imu_age_s = 0.021;
  EXPECT_EQ(safety.readiness(snapshot), FaultReason::kImuStale);

  snapshot = readySnapshot();
  snapshot.arm_state_age_s = 0.021;
  EXPECT_EQ(safety.readiness(snapshot), FaultReason::kArmStateStale);
}

TEST(FlyingHandSafety, LatchesAfterThreeConsecutiveSolverTimeouts)
{
  FlyingHandSafety safety(0.02, 0.008, 3);
  EXPECT_FALSE(safety.recordSolverResult(0.009, true));
  EXPECT_FALSE(safety.faultLatched());
  EXPECT_FALSE(safety.recordSolverResult(0.009, true));
  EXPECT_FALSE(safety.faultLatched());
  EXPECT_FALSE(safety.recordSolverResult(0.009, true));
  EXPECT_TRUE(safety.faultLatched());
  EXPECT_EQ(safety.faultReason(), FaultReason::kSolverTimeout);
}

TEST(FlyingHandSafety, SuccessfulSolveResetsTimeoutSequence)
{
  FlyingHandSafety safety(0.02, 0.008, 3);
  EXPECT_FALSE(safety.recordSolverResult(0.009, true));
  EXPECT_TRUE(safety.recordSolverResult(0.004, true));
  EXPECT_EQ(safety.consecutiveTimeouts(), 0);
  EXPECT_FALSE(safety.recordSolverResult(0.009, true));
  EXPECT_FALSE(safety.faultLatched());
}

TEST(FlyingHandSafety, InvalidSolverOutputFaultsImmediately)
{
  FlyingHandSafety safety(0.02, 0.008, 3);
  EXPECT_FALSE(safety.recordSolverResult(0.004, false));
  EXPECT_TRUE(safety.faultLatched());
  EXPECT_EQ(safety.faultReason(), FaultReason::kSolverInvalid);

  safety.reset();
  EXPECT_FALSE(safety.faultLatched());
  EXPECT_EQ(safety.consecutiveTimeouts(), 0);
}

TEST(ControllerOutput, DetectsNonFiniteBoundaryValues)
{
  ControllerOutput output;
  EXPECT_TRUE(output.allFinite());
  EXPECT_FALSE(output.normalizedCommandValid());
  output.normalized_thrust_frd.z() = 0.1F;
  EXPECT_FALSE(output.normalizedCommandValid());
  output.normalized_thrust_frd = Eigen::Vector3f(0.01F, 0.0F, -0.5F);
  EXPECT_TRUE(output.normalizedCommandValid());
  output.normalized_thrust_frd = Eigen::Vector3f(0.0F, 0.0F, -0.5F);
  EXPECT_TRUE(output.normalizedCommandValid());
  output.arm_position_command_rad[2] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(output.allFinite());
  EXPECT_FALSE(output.normalizedCommandValid());
}

}  // namespace
}  // namespace flying_hand_mode::runtime
