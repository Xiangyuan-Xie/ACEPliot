#pragma once

#include <optional>
#include <string>
#include <vector>

struct ArmTrajectoryConfig
{
  std::vector<std::string> joint_names;
  std::vector<double> segment_durations_s;
  std::vector<double> positions;
  double max_joint_velocity_rad_s{1.5};
  int loop_count{1};
  bool publish_gripper{false};
  std::string gripper_joint_name{"joint_5"};
  std::vector<double> gripper_positions;
  double max_gripper_velocity_rad_s{1.0};
};

struct GripperTrajectorySample
{
  std::string name;
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
};

struct ArmTrajectorySample
{
  std::vector<std::string> joint_names;
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> efforts;
  std::optional<GripperTrajectorySample> gripper;
  bool finished{false};
};

class ArmTrajectoryProfile
{
public:
  explicit ArmTrajectoryProfile(ArmTrajectoryConfig config);

  ArmTrajectorySample sample(double elapsed_s) const;
  double totalDurationS() const;
  double trajectoryDurationS() const;
  double initialTransitionDurationS() const;
  std::vector<double> firstJointPositions() const;
  std::optional<double> firstGripperPosition() const;
  double maxJointVelocityRadS() const;
  double maxGripperVelocityRadS() const;
  std::size_t jointCount() const;

private:
  ArmTrajectoryConfig config_;
  std::vector<double> cumulative_times_s_;
  double total_duration_s_{0.0};
  double trajectory_duration_s_{0.0};
  double initial_transition_duration_s_{0.0};
  std::size_t joint_count_{0};
};

struct ArmSyncConfig
{
  bool enable_sync_handshake{true};
  double follower_state_timeout_s{0.5};
  double sync_status_timeout_s{0.5};
  double ready_dwell_s{0.2};
  bool auto_start_tracking{true};
  bool require_follower_state_before_tracking{true};
};

struct ArmSyncUpdate
{
  std::string leader_mode{"sync_request"};
  bool commands_allowed{false};
  bool tracking_started{false};
};

class ArmSyncHandshake
{
public:
  explicit ArmSyncHandshake(ArmSyncConfig config);

  void notifyFollowerState(double now_s);
  void notifyFollowerSyncStatus(const std::string & status, double now_s);
  ArmSyncUpdate update(double now_s);

private:
  bool hasRecentFollowerState(double now_s) const;
  bool hasRecentSyncStatus(double now_s) const;
  bool followerReady() const;
  bool followerLost() const;
  void requestSync();

  ArmSyncConfig config_;
  std::string leader_mode_{"sync_request"};
  std::string follower_status_{"idle"};
  std::optional<double> last_follower_state_s_;
  std::optional<double> last_sync_status_s_;
  std::optional<double> ready_since_s_;
  bool has_started_tracking_{false};
};

class ArmTrajectoryPlayback
{
public:
  explicit ArmTrajectoryPlayback(const ArmTrajectoryProfile & profile);

  void startAt(double now_s);
  void startFromFollowerState(
    double now_s,
    const std::vector<double> & follower_positions,
    std::optional<double> follower_gripper_position);
  ArmTrajectorySample sample(double now_s) const;

  bool active() const;
  double transitionDurationS() const;
  bool usedFollowerGripperState() const;

private:
  const ArmTrajectoryProfile & profile_;
  bool active_{false};
  double start_time_s_{0.0};
  double transition_duration_s_{0.0};
  std::vector<double> transition_start_positions_;
  std::vector<double> transition_target_positions_;
  std::optional<double> transition_start_gripper_position_;
  std::optional<double> transition_target_gripper_position_;
  bool used_follower_gripper_state_{false};
};
