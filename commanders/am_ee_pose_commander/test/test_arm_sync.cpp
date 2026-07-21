#include <am_ee_pose_commander/arm_sync.hpp>

#include <gtest/gtest.h>

namespace am_ee_pose_commander
{
namespace
{

TEST(ArmSyncHandshake, RequiresFreshStateAndReadyDwell)
{
  ArmSyncHandshake handshake(0.5, 0.5, 0.2);
  EXPECT_EQ(handshake.update(0.0).leader_mode, "sync_request");

  handshake.notifyFollowerState(1.0);
  handshake.notifyFollowerSyncStatus("ready", 1.0);
  EXPECT_EQ(handshake.update(1.0).leader_mode, "ready");
  EXPECT_FALSE(handshake.update(1.1).commands_allowed);
  const auto tracking = handshake.update(1.21);
  EXPECT_EQ(tracking.leader_mode, "tracking");
  EXPECT_TRUE(tracking.commands_allowed);
  EXPECT_TRUE(tracking.tracking_started);
}

TEST(ArmSyncHandshake, FaultReturnsToSyncRequest)
{
  ArmSyncHandshake handshake(0.5, 0.5, 0.0);
  handshake.notifyFollowerState(1.0);
  handshake.notifyFollowerSyncStatus("ready", 1.0);
  handshake.update(1.0);
  ASSERT_TRUE(handshake.update(1.0).commands_allowed);

  handshake.notifyFollowerSyncStatus("fault", 1.1);
  const auto stopped = handshake.update(1.1);
  EXPECT_EQ(stopped.leader_mode, "sync_request");
  EXPECT_FALSE(stopped.commands_allowed);
}

}  // namespace
}  // namespace am_ee_pose_commander
