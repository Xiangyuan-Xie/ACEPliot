#include <gtest/gtest.h>

#include <arm_trajectory_commander/arm_trajectory.hpp>

#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
ArmTrajectoryConfig makeArmConfig()
{
  ArmTrajectoryConfig config;
  config.joint_names = {"joint_1", "joint_2"};
  config.segment_durations_s = {1.0, 1.0, 2.0};
  config.positions = {
    0.0, 0.0,
    1.0, 2.0,
    3.0, 4.0,
  };
  config.max_joint_velocity_rad_s = 3.0;
  config.loop_count = 1;
  config.publish_gripper = true;
  config.gripper_joint_name = "joint_5";
  config.gripper_positions = {0.0, 1.0, 0.0};
  config.max_gripper_velocity_rad_s = 1.5;
  return config;
}
}  // namespace

TEST(ArmTrajectory, InterpolatesPositionsAndAutoVelocities)
{
  const ArmTrajectoryProfile profile(makeArmConfig());

  const ArmTrajectorySample first = profile.sample(0.5);
  ASSERT_EQ(first.positions.size(), 2u);
  EXPECT_DOUBLE_EQ(first.positions[0], 0.5);
  EXPECT_DOUBLE_EQ(first.positions[1], 1.0);
  EXPECT_DOUBLE_EQ(first.velocities[0], 1.0);
  EXPECT_DOUBLE_EQ(first.velocities[1], 2.0);
  ASSERT_TRUE(first.gripper.has_value());
  EXPECT_DOUBLE_EQ(first.gripper->position, 0.5);
  EXPECT_DOUBLE_EQ(first.gripper->velocity, 1.0);

  const ArmTrajectorySample second = profile.sample(2.0);
  EXPECT_DOUBLE_EQ(second.positions[0], 2.0);
  EXPECT_DOUBLE_EQ(second.positions[1], 3.0);
  EXPECT_DOUBLE_EQ(second.velocities[0], 1.0);
  EXPECT_DOUBLE_EQ(second.velocities[1], 1.0);
  ASSERT_TRUE(second.gripper.has_value());
  EXPECT_DOUBLE_EQ(second.gripper->position, 0.5);
  EXPECT_DOUBLE_EQ(second.gripper->velocity, -0.5);
}

TEST(ArmTrajectory, HoldsFinalWaypointWhenNotLooping)
{
  const ArmTrajectoryProfile profile(makeArmConfig());

  const ArmTrajectorySample sample = profile.sample(10.0);
  EXPECT_TRUE(sample.finished);
  EXPECT_DOUBLE_EQ(sample.positions[0], 3.0);
  EXPECT_DOUBLE_EQ(sample.positions[1], 4.0);
  EXPECT_DOUBLE_EQ(sample.velocities[0], 0.0);
  EXPECT_DOUBLE_EQ(sample.velocities[1], 0.0);
  ASSERT_TRUE(sample.gripper.has_value());
  EXPECT_DOUBLE_EQ(sample.gripper->position, 0.0);
  EXPECT_DOUBLE_EQ(sample.gripper->velocity, 0.0);
}

TEST(ArmTrajectory, LoopsConfiguredCountBeforeFinishing)
{
  auto config = makeArmConfig();
  config.loop_count = 2;
  const ArmTrajectoryProfile profile(config);

  const ArmTrajectorySample second_loop = profile.sample(profile.trajectoryDurationS() + 0.5);
  EXPECT_FALSE(second_loop.finished);
  EXPECT_DOUBLE_EQ(second_loop.positions[0], 0.5);
  EXPECT_DOUBLE_EQ(second_loop.positions[1], 1.0);
  ASSERT_TRUE(second_loop.gripper.has_value());
  EXPECT_DOUBLE_EQ(second_loop.gripper->position, 0.5);

  const ArmTrajectorySample finished = profile.sample(2.0 * profile.trajectoryDurationS() + 1e-9);
  EXPECT_TRUE(finished.finished);
  EXPECT_DOUBLE_EQ(finished.positions[0], 3.0);
  EXPECT_DOUBLE_EQ(finished.positions[1], 4.0);
  EXPECT_DOUBLE_EQ(finished.velocities[0], 0.0);
  EXPECT_DOUBLE_EQ(finished.velocities[1], 0.0);
  ASSERT_TRUE(finished.gripper.has_value());
  EXPECT_DOUBLE_EQ(finished.gripper->position, 0.0);
  EXPECT_DOUBLE_EQ(finished.gripper->velocity, 0.0);
}

TEST(ArmTrajectory, LoopCountZeroLoopsForever)
{
  auto config = makeArmConfig();
  config.loop_count = 0;
  const ArmTrajectoryProfile profile(config);

  const ArmTrajectorySample sample = profile.sample(3.0 * profile.trajectoryDurationS() + 0.5);
  EXPECT_FALSE(sample.finished);
  EXPECT_DOUBLE_EQ(sample.positions[0], 0.5);
  EXPECT_DOUBLE_EQ(sample.positions[1], 1.0);
  ASSERT_TRUE(sample.gripper.has_value());
  EXPECT_DOUBLE_EQ(sample.gripper->position, 0.5);
}

TEST(ArmTrajectory, RejectsInvalidFlattenedInputs)
{
  auto config = makeArmConfig();
  config.positions.pop_back();
  EXPECT_THROW(ArmTrajectoryProfile{config}, std::invalid_argument);

  auto bad_gripper = makeArmConfig();
  bad_gripper.gripper_positions.pop_back();
  EXPECT_THROW(ArmTrajectoryProfile{bad_gripper}, std::invalid_argument);
}

TEST(ArmTrajectory, RejectsInvalidSegmentDurations)
{
  auto empty = makeArmConfig();
  empty.segment_durations_s.clear();
  EXPECT_THROW(ArmTrajectoryProfile{empty}, std::invalid_argument);

  auto non_positive = makeArmConfig();
  non_positive.segment_durations_s[1] = 0.0;
  EXPECT_THROW(ArmTrajectoryProfile{non_positive}, std::invalid_argument);

  auto wrong_count = makeArmConfig();
  wrong_count.segment_durations_s.pop_back();
  EXPECT_THROW(ArmTrajectoryProfile{wrong_count}, std::invalid_argument);
}

TEST(ArmTrajectory, ReportsConfiguredInitialAndYamlDurations)
{
  const ArmTrajectoryProfile profile(makeArmConfig());

  EXPECT_DOUBLE_EQ(profile.initialTransitionDurationS(), 1.0);
  EXPECT_DOUBLE_EQ(profile.trajectoryDurationS(), 3.0);
  EXPECT_DOUBLE_EQ(profile.totalDurationS(), 4.0);
}

TEST(ArmTrajectory, RejectsJointVelocityAboveLimit)
{
  auto config = makeArmConfig();
  config.max_joint_velocity_rad_s = 0.5;

  EXPECT_THROW(ArmTrajectoryProfile{config}, std::invalid_argument);
}

TEST(ArmTrajectory, RejectsNegativeLoopCount)
{
  auto config = makeArmConfig();
  config.loop_count = -1;

  EXPECT_THROW(ArmTrajectoryProfile{config}, std::invalid_argument);
}

TEST(ArmTrajectory, RejectsGripperVelocityAboveLimit)
{
  auto config = makeArmConfig();
  config.max_joint_velocity_rad_s = 10.0;
  config.max_gripper_velocity_rad_s = 0.25;

  EXPECT_THROW(ArmTrajectoryProfile{config}, std::invalid_argument);
}

TEST(ArmSyncHandshake, StartsWithSyncRequestAndBlocksCommands)
{
  ArmSyncConfig config;
  config.enable_sync_handshake = true;
  const ArmSyncUpdate update = ArmSyncHandshake(config).update(0.0);

  EXPECT_EQ(update.leader_mode, "sync_request");
  EXPECT_FALSE(update.commands_allowed);
  EXPECT_FALSE(update.tracking_started);
}

TEST(ArmSyncHandshake, WaitsForFollowerReadyAndDwellBeforeTracking)
{
  ArmSyncConfig config;
  config.enable_sync_handshake = true;
  config.follower_state_timeout_s = 0.5;
  config.sync_status_timeout_s = 0.5;
  config.ready_dwell_s = 0.2;
  ArmSyncHandshake handshake(config);

  handshake.notifyFollowerState(1.0);
  handshake.notifyFollowerSyncStatus("ready", 1.0);

  ArmSyncUpdate update = handshake.update(1.0);
  EXPECT_EQ(update.leader_mode, "ready");
  EXPECT_FALSE(update.commands_allowed);

  update = handshake.update(1.19);
  EXPECT_EQ(update.leader_mode, "ready");
  EXPECT_FALSE(update.commands_allowed);

  update = handshake.update(1.21);
  EXPECT_EQ(update.leader_mode, "tracking");
  EXPECT_TRUE(update.commands_allowed);
  EXPECT_TRUE(update.tracking_started);

  update = handshake.update(1.22);
  EXPECT_EQ(update.leader_mode, "tracking");
  EXPECT_TRUE(update.commands_allowed);
  EXPECT_FALSE(update.tracking_started);
}

TEST(ArmSyncHandshake, DefaultConfigRequiresFollowerStateBeforeTracking)
{
  ArmSyncConfig config;
  config.enable_sync_handshake = true;
  config.follower_state_timeout_s = 0.5;
  config.sync_status_timeout_s = 0.5;
  config.ready_dwell_s = 0.1;
  ArmSyncHandshake handshake(config);

  handshake.notifyFollowerSyncStatus("ready", 1.0);

  ArmSyncUpdate update = handshake.update(1.0);
  EXPECT_EQ(update.leader_mode, "sync_request");
  EXPECT_FALSE(update.commands_allowed);

  update = handshake.update(1.2);
  EXPECT_EQ(update.leader_mode, "sync_request");
  EXPECT_FALSE(update.commands_allowed);
  EXPECT_FALSE(update.tracking_started);
}

TEST(ArmSyncHandshake, RelaxedConfigAllowsReadyStatusWithoutFollowerState)
{
  ArmSyncConfig config;
  config.enable_sync_handshake = true;
  config.require_follower_state_before_tracking = false;
  config.sync_status_timeout_s = 0.5;
  config.ready_dwell_s = 0.1;
  ArmSyncHandshake handshake(config);

  handshake.notifyFollowerSyncStatus("ready", 1.0);

  ArmSyncUpdate update = handshake.update(1.0);
  EXPECT_EQ(update.leader_mode, "ready");
  EXPECT_FALSE(update.commands_allowed);

  update = handshake.update(1.11);
  EXPECT_EQ(update.leader_mode, "tracking");
  EXPECT_TRUE(update.commands_allowed);
  EXPECT_TRUE(update.tracking_started);
}

TEST(ArmSyncHandshake, RelaxedConfigStillRequiresRecentHealthySyncStatus)
{
  ArmSyncConfig config;
  config.enable_sync_handshake = true;
  config.require_follower_state_before_tracking = false;
  config.sync_status_timeout_s = 0.5;
  config.ready_dwell_s = 0.1;
  ArmSyncHandshake handshake(config);

  handshake.notifyFollowerSyncStatus("ready", 2.0);
  EXPECT_FALSE(handshake.update(2.0).commands_allowed);
  ASSERT_TRUE(handshake.update(2.11).commands_allowed);

  ArmSyncUpdate update = handshake.update(2.6);
  EXPECT_EQ(update.leader_mode, "sync_request");
  EXPECT_FALSE(update.commands_allowed);

  handshake.notifyFollowerSyncStatus("ready", 3.0);
  EXPECT_FALSE(handshake.update(3.0).commands_allowed);
  ASSERT_TRUE(handshake.update(3.11).commands_allowed);

  handshake.notifyFollowerSyncStatus("fault", 3.2);
  update = handshake.update(3.2);
  EXPECT_EQ(update.leader_mode, "sync_request");
  EXPECT_FALSE(update.commands_allowed);
}

TEST(ArmSyncHandshake, LostStatusOrTimeoutReturnsToSyncRequest)
{
  ArmSyncConfig config;
  config.enable_sync_handshake = true;
  config.follower_state_timeout_s = 0.5;
  config.sync_status_timeout_s = 0.5;
  config.ready_dwell_s = 0.1;
  ArmSyncHandshake handshake(config);
  handshake.notifyFollowerState(2.0);
  handshake.notifyFollowerSyncStatus("ready", 2.0);
  EXPECT_FALSE(handshake.update(2.0).commands_allowed);
  ASSERT_TRUE(handshake.update(2.11).commands_allowed);

  handshake.notifyFollowerSyncStatus("lost", 2.2);
  ArmSyncUpdate update = handshake.update(2.2);
  EXPECT_EQ(update.leader_mode, "sync_request");
  EXPECT_FALSE(update.commands_allowed);

  handshake.notifyFollowerState(3.0);
  handshake.notifyFollowerSyncStatus("ready", 3.0);
  EXPECT_FALSE(handshake.update(3.0).commands_allowed);
  ASSERT_TRUE(handshake.update(3.11).commands_allowed);
  update = handshake.update(3.7);
  EXPECT_EQ(update.leader_mode, "sync_request");
  EXPECT_FALSE(update.commands_allowed);
}

TEST(ArmTrajectoryPlayback, TransitionsFromFollowerStateBeforeYamlTimeline)
{
  const ArmTrajectoryProfile profile(makeArmConfig());
  ArmTrajectoryPlayback playback(profile);

  playback.startFromFollowerState(10.0, std::vector<double>{-1.0, 2.0}, 0.25);

  EXPECT_TRUE(playback.active());
  EXPECT_TRUE(playback.usedFollowerGripperState());
  EXPECT_NEAR(playback.transitionDurationS(), 1.0, 1e-9);

  ArmTrajectorySample sample = playback.sample(10.0);
  EXPECT_DOUBLE_EQ(sample.positions[0], -1.0);
  EXPECT_DOUBLE_EQ(sample.positions[1], 2.0);
  ASSERT_TRUE(sample.gripper.has_value());
  EXPECT_DOUBLE_EQ(sample.gripper->position, 0.25);

  sample = playback.sample(10.0 + playback.transitionDurationS() * 0.5);
  EXPECT_NEAR(sample.positions[0], -0.5, 1e-9);
  EXPECT_NEAR(sample.positions[1], 1.0, 1e-9);
  ASSERT_TRUE(sample.gripper.has_value());
  EXPECT_NEAR(sample.gripper->position, 0.125, 1e-9);

  sample = playback.sample(10.0 + playback.transitionDurationS() + 0.5);
  EXPECT_NEAR(sample.positions[0], 0.5, 1e-9);
  EXPECT_NEAR(sample.positions[1], 1.0, 1e-9);
  ASSERT_TRUE(sample.gripper.has_value());
  EXPECT_NEAR(sample.gripper->position, 0.5, 1e-9);
}

TEST(ArmTrajectoryPlayback, RejectsInitialTransitionAboveVelocityLimit)
{
  const ArmTrajectoryProfile profile(makeArmConfig());
  ArmTrajectoryPlayback playback(profile);

  EXPECT_THROW(
    playback.startFromFollowerState(10.0, std::vector<double>{-4.0, 0.0}, std::nullopt),
    std::invalid_argument);
}

TEST(ArmTrajectoryPlayback, HoldsFirstWaypointDuringInitialSegmentWithoutFollowerState)
{
  const ArmTrajectoryProfile profile(makeArmConfig());
  ArmTrajectoryPlayback playback(profile);

  playback.startAt(10.0);

  EXPECT_DOUBLE_EQ(playback.transitionDurationS(), 1.0);

  ArmTrajectorySample sample = playback.sample(10.5);
  EXPECT_DOUBLE_EQ(sample.positions[0], 0.0);
  EXPECT_DOUBLE_EQ(sample.positions[1], 0.0);
  EXPECT_DOUBLE_EQ(sample.velocities[0], 0.0);
  EXPECT_DOUBLE_EQ(sample.velocities[1], 0.0);
  ASSERT_TRUE(sample.gripper.has_value());
  EXPECT_DOUBLE_EQ(sample.gripper->position, 0.0);
  EXPECT_DOUBLE_EQ(sample.gripper->velocity, 0.0);

  sample = playback.sample(11.5);
  EXPECT_DOUBLE_EQ(sample.positions[0], 0.5);
  EXPECT_DOUBLE_EQ(sample.positions[1], 1.0);
  EXPECT_DOUBLE_EQ(sample.velocities[0], 1.0);
  EXPECT_DOUBLE_EQ(sample.velocities[1], 2.0);
}

TEST(ArmTrajectoryPlayback, StartsGripperAtFirstWaypointWhenFollowerGripperMissing)
{
  const ArmTrajectoryProfile profile(makeArmConfig());
  ArmTrajectoryPlayback playback(profile);

  playback.startFromFollowerState(5.0, std::vector<double>{-1.0, 2.0}, std::nullopt);

  EXPECT_FALSE(playback.usedFollowerGripperState());
  const ArmTrajectorySample sample = playback.sample(5.0);
  ASSERT_TRUE(sample.gripper.has_value());
  EXPECT_DOUBLE_EQ(sample.gripper->position, 0.0);
}

TEST(ArmTrajectoryPlayback, MarksNonLoopTrajectoryFinishedAfterTransitionAndTimeline)
{
  const ArmTrajectoryProfile profile(makeArmConfig());
  ArmTrajectoryPlayback playback(profile);
  playback.startFromFollowerState(10.0, std::vector<double>{-1.0, 2.0}, 0.25);

  const ArmTrajectorySample before_finish = playback.sample(
    10.0 + profile.totalDurationS() - 0.01);
  EXPECT_FALSE(before_finish.finished);

  const ArmTrajectorySample finished = playback.sample(
    10.0 + profile.totalDurationS() + 1e-9);
  EXPECT_TRUE(finished.finished);
}
