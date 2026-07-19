#include <arm_trajectory_commander/arm_trajectory.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace
{
double flatAt(
  const std::vector<double> & values,
  std::size_t waypoint_index,
  std::size_t value_index,
  std::size_t value_count)
{
  return values[waypoint_index * value_count + value_index];
}

double lerp(double a, double b, double ratio)
{
  return a + (b - a) * ratio;
}

void validatePositiveLimit(double value, const char * name)
{
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(name) + " must be positive");
  }
}

struct Segment
{
  std::size_t start_index{0};
  std::size_t end_index{0};
  double ratio{0.0};
  bool hold_final{false};
};

Segment findSegment(const std::vector<double> & times, double elapsed_s)
{
  const double total_duration_s = times.back();
  if (elapsed_s < 0.0) {
    elapsed_s = 0.0;
  }

  if (elapsed_s >= total_duration_s) {
    return Segment{times.size() - 1, times.size() - 1, 0.0, true};
  }

  for (std::size_t i = 0; i + 1 < times.size(); ++i) {
    if (elapsed_s >= times[i] && elapsed_s < times[i + 1]) {
      const double duration_s = times[i + 1] - times[i];
      return Segment{i, i + 1, (elapsed_s - times[i]) / duration_s, false};
    }
  }

  return Segment{0, 0, 0.0, true};
}
}  // namespace

ArmTrajectoryProfile::ArmTrajectoryProfile(ArmTrajectoryConfig config)
: config_(std::move(config)),
  joint_count_(config_.joint_names.size())
{
  if (joint_count_ == 0) {
    throw std::invalid_argument("joint_names must not be empty");
  }
  if (config_.segment_durations_s.empty()) {
    throw std::invalid_argument("segment_durations_s must contain at least one segment");
  }
  if (config_.loop_count < 0) {
    throw std::invalid_argument("loop_count must be non-negative");
  }
  for (double duration_s : config_.segment_durations_s) {
    if (!std::isfinite(duration_s) || duration_s <= 0.0) {
      throw std::invalid_argument("segment_durations_s values must be positive");
    }
  }

  const std::size_t waypoint_count = config_.segment_durations_s.size();
  const std::size_t arm_value_count = waypoint_count * joint_count_;
  if (config_.positions.size() != arm_value_count) {
    throw std::invalid_argument("positions has invalid flattened length");
  }
  initial_transition_duration_s_ = config_.segment_durations_s.front();
  cumulative_times_s_.clear();
  cumulative_times_s_.reserve(waypoint_count);
  cumulative_times_s_.push_back(0.0);
  for (std::size_t segment = 1; segment < config_.segment_durations_s.size(); ++segment) {
    cumulative_times_s_.push_back(
      cumulative_times_s_.back() +
      config_.segment_durations_s[segment]);
  }
  validatePositiveLimit(config_.max_joint_velocity_rad_s, "max_joint_velocity_rad_s");

  for (std::size_t waypoint = 0; waypoint + 1 < waypoint_count; ++waypoint) {
    const double dt_s = cumulative_times_s_[waypoint + 1] - cumulative_times_s_[waypoint];
    for (std::size_t joint = 0; joint < joint_count_; ++joint) {
      const double start_position = flatAt(config_.positions, waypoint, joint, joint_count_);
      const double end_position = flatAt(config_.positions, waypoint + 1, joint, joint_count_);
      const double velocity = std::abs((end_position - start_position) / dt_s);
      if (velocity > config_.max_joint_velocity_rad_s) {
        throw std::invalid_argument("joint velocity exceeds max_joint_velocity_rad_s");
      }
    }
  }

  if (config_.publish_gripper) {
    if (config_.gripper_positions.size() != waypoint_count) {
      throw std::invalid_argument("gripper_positions length must match waypoint count");
    }
    validatePositiveLimit(config_.max_gripper_velocity_rad_s, "max_gripper_velocity_rad_s");
    for (std::size_t waypoint = 0; waypoint + 1 < waypoint_count; ++waypoint) {
      const double dt_s = cumulative_times_s_[waypoint + 1] - cumulative_times_s_[waypoint];
      const double start_position = config_.gripper_positions[waypoint];
      const double end_position = config_.gripper_positions[waypoint + 1];
      const double velocity = std::abs((end_position - start_position) / dt_s);
      if (velocity > config_.max_gripper_velocity_rad_s) {
        throw std::invalid_argument("gripper velocity exceeds max_gripper_velocity_rad_s");
      }
    }
  }

  trajectory_duration_s_ = cumulative_times_s_.back();
  total_duration_s_ = initial_transition_duration_s_ + trajectory_duration_s_;
}

ArmTrajectorySample ArmTrajectoryProfile::sample(double elapsed_s) const
{
  ArmTrajectorySample sample;
  sample.joint_names = config_.joint_names;
  sample.positions.resize(joint_count_, 0.0);
  sample.velocities.resize(joint_count_, 0.0);
  sample.efforts.resize(joint_count_, 0.0);

  sample.finished =
    config_.loop_count > 0 &&
    elapsed_s >= trajectory_duration_s_ * static_cast<double>(config_.loop_count);

  double cycle_elapsed_s = elapsed_s;
  if (sample.finished) {
    cycle_elapsed_s = trajectory_duration_s_;
  } else if (config_.loop_count == 0 || config_.loop_count > 1) {
    cycle_elapsed_s = std::fmod(std::max(0.0, elapsed_s), trajectory_duration_s_);
  }

  const Segment segment = findSegment(cumulative_times_s_, cycle_elapsed_s);

  for (std::size_t joint = 0; joint < joint_count_; ++joint) {
    const double start_position =
      flatAt(config_.positions, segment.start_index, joint, joint_count_);
    const double end_position = flatAt(config_.positions, segment.end_index, joint, joint_count_);
    sample.positions[joint] = segment.hold_final ? start_position :
      lerp(start_position, end_position, segment.ratio);

    if (sample.finished) {
      sample.velocities[joint] = 0.0;
    } else {
      const double dt_s =
        cumulative_times_s_[segment.end_index] - cumulative_times_s_[segment.start_index];
      sample.velocities[joint] = dt_s > 0.0 ? (end_position - start_position) / dt_s : 0.0;
    }
  }

  if (config_.publish_gripper) {
    GripperTrajectorySample gripper;
    gripper.name = config_.gripper_joint_name;
    const double start_position = config_.gripper_positions[segment.start_index];
    const double end_position = config_.gripper_positions[segment.end_index];
    gripper.position = segment.hold_final ? start_position :
      lerp(start_position, end_position, segment.ratio);

    if (sample.finished) {
      gripper.velocity = 0.0;
    } else {
      const double dt_s =
        cumulative_times_s_[segment.end_index] - cumulative_times_s_[segment.start_index];
      gripper.velocity = dt_s > 0.0 ? (end_position - start_position) / dt_s : 0.0;
    }

    sample.gripper = gripper;
  }

  return sample;
}

double ArmTrajectoryProfile::totalDurationS() const
{
  return total_duration_s_;
}

double ArmTrajectoryProfile::trajectoryDurationS() const
{
  return trajectory_duration_s_;
}

double ArmTrajectoryProfile::initialTransitionDurationS() const
{
  return initial_transition_duration_s_;
}

std::vector<double> ArmTrajectoryProfile::firstJointPositions() const
{
  std::vector<double> positions(joint_count_, 0.0);
  for (std::size_t joint = 0; joint < joint_count_; ++joint) {
    positions[joint] = flatAt(config_.positions, 0, joint, joint_count_);
  }
  return positions;
}

std::optional<double> ArmTrajectoryProfile::firstGripperPosition() const
{
  if (!config_.publish_gripper || config_.gripper_positions.empty()) {
    return std::nullopt;
  }
  return config_.gripper_positions.front();
}

double ArmTrajectoryProfile::maxJointVelocityRadS() const
{
  return config_.max_joint_velocity_rad_s;
}

double ArmTrajectoryProfile::maxGripperVelocityRadS() const
{
  return config_.max_gripper_velocity_rad_s;
}

std::size_t ArmTrajectoryProfile::jointCount() const
{
  return joint_count_;
}

ArmSyncHandshake::ArmSyncHandshake(ArmSyncConfig config)
: config_(config)
{
  if (config_.follower_state_timeout_s <= 0.0) {
    throw std::invalid_argument("follower_state_timeout_s must be positive");
  }
  if (config_.sync_status_timeout_s <= 0.0) {
    throw std::invalid_argument("sync_status_timeout_s must be positive");
  }
  if (config_.ready_dwell_s < 0.0) {
    throw std::invalid_argument("ready_dwell_s must be non-negative");
  }
  if (!config_.enable_sync_handshake) {
    leader_mode_ = "tracking";
    has_started_tracking_ = true;
  }
}

void ArmSyncHandshake::notifyFollowerState(double now_s)
{
  last_follower_state_s_ = now_s;
}

void ArmSyncHandshake::notifyFollowerSyncStatus(const std::string & status, double now_s)
{
  if (
    status != "idle" && status != "aligning" && status != "ready" &&
    status != "tracking" && status != "lost" && status != "fault")
  {
    return;
  }
  follower_status_ = status;
  last_sync_status_s_ = now_s;
}

bool ArmSyncHandshake::hasRecentFollowerState(double now_s) const
{
  return last_follower_state_s_.has_value() &&
         now_s - *last_follower_state_s_ <= config_.follower_state_timeout_s;
}

bool ArmSyncHandshake::hasRecentSyncStatus(double now_s) const
{
  return last_sync_status_s_.has_value() &&
         now_s - *last_sync_status_s_ <= config_.sync_status_timeout_s;
}

bool ArmSyncHandshake::followerReady() const
{
  return follower_status_ == "ready" || follower_status_ == "tracking";
}

bool ArmSyncHandshake::followerLost() const
{
  return follower_status_ == "lost" || follower_status_ == "fault";
}

void ArmSyncHandshake::requestSync()
{
  leader_mode_ = "sync_request";
  ready_since_s_.reset();
}

ArmSyncUpdate ArmSyncHandshake::update(double now_s)
{
  if (!config_.enable_sync_handshake) {
    const bool first_update = !has_started_tracking_;
    has_started_tracking_ = true;
    return ArmSyncUpdate{"tracking", true, first_update};
  }

  bool tracking_started = false;
  const bool missing_required_follower_state =
    config_.require_follower_state_before_tracking && !hasRecentFollowerState(now_s);
  if (followerLost() || missing_required_follower_state) {
    requestSync();
  } else if (leader_mode_ == "tracking") {
    if (!hasRecentSyncStatus(now_s)) {
      requestSync();
    }
  } else if (!hasRecentSyncStatus(now_s) || !followerReady()) {
    requestSync();
  } else {
    const bool ready_dwell_complete =
      config_.auto_start_tracking && ready_since_s_.has_value() &&
      now_s - *ready_since_s_ >= config_.ready_dwell_s;
    if (leader_mode_ != "ready") {
      leader_mode_ = "ready";
      ready_since_s_ = now_s;
    } else if (ready_dwell_complete) {
      leader_mode_ = "tracking";
      tracking_started = true;
      has_started_tracking_ = true;
    }
  }

  return ArmSyncUpdate{leader_mode_, leader_mode_ == "tracking", tracking_started};
}

ArmTrajectoryPlayback::ArmTrajectoryPlayback(const ArmTrajectoryProfile & profile)
: profile_(profile)
{
}

void ArmTrajectoryPlayback::startAt(double now_s)
{
  active_ = true;
  start_time_s_ = now_s;
  transition_duration_s_ = profile_.initialTransitionDurationS();
  transition_start_positions_.clear();
  transition_target_positions_.clear();
  transition_start_gripper_position_.reset();
  transition_target_gripper_position_.reset();
  used_follower_gripper_state_ = false;
}

void ArmTrajectoryPlayback::startFromFollowerState(
  double now_s,
  const std::vector<double> & follower_positions,
  std::optional<double> follower_gripper_position)
{
  const std::vector<double> target_positions = profile_.firstJointPositions();
  if (follower_positions.size() != target_positions.size()) {
    throw std::invalid_argument("follower arm state length does not match trajectory joints");
  }

  active_ = true;
  start_time_s_ = now_s;
  transition_start_positions_ = follower_positions;
  transition_target_positions_ = target_positions;
  transition_duration_s_ = profile_.initialTransitionDurationS();
  for (std::size_t i = 0; i < target_positions.size(); ++i) {
    const double velocity =
      std::abs(target_positions[i] - follower_positions[i]) / transition_duration_s_;
    if (velocity > profile_.maxJointVelocityRadS()) {
      throw std::invalid_argument(
              "initial transition joint velocity exceeds max_joint_velocity_rad_s");
    }
  }

  const std::optional<double> target_gripper = profile_.firstGripperPosition();
  transition_start_gripper_position_.reset();
  transition_target_gripper_position_.reset();
  used_follower_gripper_state_ = false;
  if (target_gripper.has_value()) {
    transition_target_gripper_position_ = target_gripper;
    if (follower_gripper_position.has_value()) {
      transition_start_gripper_position_ = follower_gripper_position;
      used_follower_gripper_state_ = true;
      const double velocity = std::abs(*target_gripper - *follower_gripper_position) /
        transition_duration_s_;
      if (velocity > profile_.maxGripperVelocityRadS()) {
        throw std::invalid_argument(
                "initial transition gripper velocity exceeds max_gripper_velocity_rad_s");
      }
    }
  }
}

ArmTrajectorySample ArmTrajectoryPlayback::sample(double now_s) const
{
  if (!active_) {
    return profile_.sample(0.0);
  }

  const double elapsed_s = std::max(0.0, now_s - start_time_s_);
  if (
    transition_duration_s_ > 0.0 && elapsed_s < transition_duration_s_ &&
    !transition_start_positions_.empty())
  {
    ArmTrajectorySample sample;
    sample.joint_names = profile_.sample(0.0).joint_names;
    sample.positions.resize(transition_target_positions_.size(), 0.0);
    sample.velocities.resize(transition_target_positions_.size(), 0.0);
    sample.efforts.resize(transition_target_positions_.size(), 0.0);
    const double ratio = elapsed_s / transition_duration_s_;
    for (std::size_t i = 0; i < transition_target_positions_.size(); ++i) {
      sample.positions[i] = lerp(
        transition_start_positions_[i], transition_target_positions_[i], ratio);
      sample.velocities[i] =
        (transition_target_positions_[i] - transition_start_positions_[i]) /
        transition_duration_s_;
    }

    if (transition_target_gripper_position_.has_value()) {
      GripperTrajectorySample gripper;
      gripper.name = profile_.sample(0.0).gripper->name;
      if (transition_start_gripper_position_.has_value()) {
        gripper.position = lerp(
          *transition_start_gripper_position_, *transition_target_gripper_position_, ratio);
        gripper.velocity =
          (*transition_target_gripper_position_ - *transition_start_gripper_position_) /
          transition_duration_s_;
      } else {
        gripper.position = *transition_target_gripper_position_;
      }
      sample.gripper = gripper;
    }
    return sample;
  }

  if (transition_duration_s_ > 0.0 && elapsed_s < transition_duration_s_) {
    ArmTrajectorySample sample = profile_.sample(0.0);
    for (double & velocity : sample.velocities) {
      velocity = 0.0;
    }
    if (sample.gripper.has_value()) {
      sample.gripper->velocity = 0.0;
    }
    return sample;
  }

  return profile_.sample(elapsed_s - transition_duration_s_);
}

bool ArmTrajectoryPlayback::active() const
{
  return active_;
}

double ArmTrajectoryPlayback::transitionDurationS() const
{
  return transition_duration_s_;
}

bool ArmTrajectoryPlayback::usedFollowerGripperState() const
{
  return used_follower_gripper_state_;
}
