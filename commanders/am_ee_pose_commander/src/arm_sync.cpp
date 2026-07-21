/****************************************************************************
 * Copyright (c) 2026 Xiangyuan Xie <dragonboat_xxy@163.com>.
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <am_ee_pose_commander/arm_sync.hpp>

#include <stdexcept>

namespace am_ee_pose_commander
{

ArmSyncHandshake::ArmSyncHandshake(
  double follower_state_timeout_s,
  double sync_status_timeout_s,
  double ready_dwell_s)
: follower_state_timeout_s_(follower_state_timeout_s),
  sync_status_timeout_s_(sync_status_timeout_s),
  ready_dwell_s_(ready_dwell_s)
{
  if (follower_state_timeout_s_ <= 0.0 || sync_status_timeout_s_ <= 0.0) {
    throw std::invalid_argument("arm sync timeouts must be positive");
  }
  if (ready_dwell_s_ < 0.0) {
    throw std::invalid_argument("arm sync ready dwell must be non-negative");
  }
}

void ArmSyncHandshake::notifyFollowerState(double now_s)
{
  last_follower_state_s_ = now_s;
}

void ArmSyncHandshake::notifyFollowerSyncStatus(const std::string & status, double now_s)
{
  if (status != "idle" && status != "aligning" && status != "ready" &&
    status != "tracking" && status != "lost" && status != "fault")
  {
    return;
  }
  follower_status_ = status;
  last_sync_status_s_ = now_s;
}

bool ArmSyncHandshake::recent(
  const std::optional<double> & time_s,
  double now_s,
  double timeout_s) const
{
  return time_s.has_value() && now_s >= *time_s && now_s - *time_s <= timeout_s;
}

void ArmSyncHandshake::requestSync()
{
  leader_mode_ = "sync_request";
  ready_since_s_.reset();
}

ArmSyncUpdate ArmSyncHandshake::update(double now_s)
{
  const bool state_recent = recent(last_follower_state_s_, now_s, follower_state_timeout_s_);
  const bool status_recent = recent(last_sync_status_s_, now_s, sync_status_timeout_s_);
  const bool follower_ready = follower_status_ == "ready" || follower_status_ == "tracking";
  const bool follower_faulted = follower_status_ == "lost" || follower_status_ == "fault";
  bool tracking_started = false;

  if (!state_recent || !status_recent || follower_faulted || !follower_ready) {
    requestSync();
  } else if (leader_mode_ == "tracking") {
    return ArmSyncUpdate{leader_mode_, true, false};
  } else if (leader_mode_ != "ready") {
    leader_mode_ = "ready";
    ready_since_s_ = now_s;
  } else if (ready_since_s_.has_value() && now_s - *ready_since_s_ >= ready_dwell_s_) {
    leader_mode_ = "tracking";
    tracking_started = true;
  }

  return ArmSyncUpdate{leader_mode_, leader_mode_ == "tracking", tracking_started};
}

}  // namespace am_ee_pose_commander
