/****************************************************************************
 * Copyright (c) 2026 Xiangyuan Xie <dragonboat_xxy@163.com>.
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#pragma once

#include <optional>
#include <string>

namespace am_ee_pose_commander
{

struct ArmSyncUpdate
{
  std::string leader_mode{"sync_request"};
  bool commands_allowed{false};
  bool tracking_started{false};
};

class ArmSyncHandshake
{
public:
  ArmSyncHandshake(
    double follower_state_timeout_s,
    double sync_status_timeout_s,
    double ready_dwell_s);

  void notifyFollowerState(double now_s);
  void notifyFollowerSyncStatus(const std::string & status, double now_s);
  ArmSyncUpdate update(double now_s);

private:
  bool recent(const std::optional<double> & time_s, double now_s, double timeout_s) const;
  void requestSync();

  double follower_state_timeout_s_{0.5};
  double sync_status_timeout_s_{0.5};
  double ready_dwell_s_{0.2};
  std::string leader_mode_{"sync_request"};
  std::string follower_status_{"idle"};
  std::optional<double> last_follower_state_s_;
  std::optional<double> last_sync_status_s_;
  std::optional<double> ready_since_s_;
};

}  // namespace am_ee_pose_commander
