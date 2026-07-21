"""Arm trajectory interpolation, safety validation, and ACETele handshake logic."""

from dataclasses import dataclass
import math
from typing import List, Optional, Sequence, Tuple


@dataclass(frozen=True)
class GripperSample:
    name: str
    position: float
    velocity: float


@dataclass(frozen=True)
class ArmSample:
    joint_names: Tuple[str, ...]
    positions: Tuple[float, ...]
    velocities: Tuple[float, ...]
    efforts: Tuple[float, ...]
    gripper: Optional[GripperSample]
    finished: bool


class ArmTrajectoryProfile:
    """Linear waypoint profile whose first duration is the startup transition."""

    def __init__(
        self,
        joint_names: Sequence[str],
        segment_durations_s: Sequence[float],
        flattened_positions: Sequence[float],
        max_joint_velocity_rad_s: float,
        loop_count: int,
        publish_gripper: bool,
        gripper_joint_name: str,
        gripper_positions: Sequence[float],
        max_gripper_velocity_rad_s: float,
    ) -> None:
        self.joint_names = tuple(str(name) for name in joint_names)
        self.segment_durations_s = tuple(float(value) for value in segment_durations_s)
        self.max_joint_velocity_rad_s = float(max_joint_velocity_rad_s)
        self.loop_count = int(loop_count)
        self.publish_gripper = bool(publish_gripper)
        self.gripper_joint_name = str(gripper_joint_name)
        self.gripper_positions = tuple(float(value) for value in gripper_positions)
        self.max_gripper_velocity_rad_s = float(max_gripper_velocity_rad_s)
        if not self.joint_names:
            raise ValueError("joint_names must not be empty")
        if not self.segment_durations_s:
            raise ValueError("segment_durations_s must not be empty")
        if self.loop_count < 0:
            raise ValueError("loop_count must be non-negative")
        if any(not math.isfinite(value) or value <= 0.0 for value in self.segment_durations_s):
            raise ValueError("segment_durations_s values must be finite and positive")
        if (
            not math.isfinite(self.max_joint_velocity_rad_s)
            or self.max_joint_velocity_rad_s <= 0.0
        ):
            raise ValueError("max_joint_velocity_rad_s must be positive")
        waypoint_count = len(self.segment_durations_s)
        joint_count = len(self.joint_names)
        if len(flattened_positions) != waypoint_count * joint_count:
            raise ValueError("positions has invalid flattened length")
        self.positions = tuple(
            tuple(
                float(flattened_positions[waypoint * joint_count + joint])
                for joint in range(joint_count)
            )
            for waypoint in range(waypoint_count)
        )
        if not all(math.isfinite(value) for waypoint in self.positions for value in waypoint):
            raise ValueError("positions must be finite")
        if self.publish_gripper:
            if len(self.gripper_positions) != waypoint_count:
                raise ValueError("gripper_positions length must match waypoint count")
            if (
                not math.isfinite(self.max_gripper_velocity_rad_s)
                or self.max_gripper_velocity_rad_s <= 0.0
            ):
                raise ValueError("max_gripper_velocity_rad_s must be positive")
        self.initial_transition_duration_s = self.segment_durations_s[0]
        self.waypoint_times_s: List[float] = [0.0]
        for duration in self.segment_durations_s[1:]:
            self.waypoint_times_s.append(self.waypoint_times_s[-1] + duration)
        self.trajectory_duration_s = self.waypoint_times_s[-1]
        if self.trajectory_duration_s == 0.0 and self.loop_count == 0:
            raise ValueError("an infinite arm profile requires at least two waypoints")
        self._validate_segment_speeds()

    def _validate_segment_speeds(self) -> None:
        for index in range(len(self.positions) - 1):
            duration = self.segment_durations_s[index + 1]
            for start, end in zip(self.positions[index], self.positions[index + 1]):
                if abs(end - start) / duration > self.max_joint_velocity_rad_s + 1.0e-9:
                    raise ValueError("joint velocity exceeds max_joint_velocity_rad_s")
            if self.publish_gripper:
                speed = abs(
                    self.gripper_positions[index + 1]
                    - self.gripper_positions[index]
                ) / duration
                if speed > self.max_gripper_velocity_rad_s + 1.0e-9:
                    raise ValueError("gripper velocity exceeds max_gripper_velocity_rad_s")

    def _segment(self, elapsed_s: float):
        if elapsed_s >= self.trajectory_duration_s:
            last = len(self.positions) - 1
            return last, last, 0.0
        for index in range(len(self.waypoint_times_s) - 1):
            start = self.waypoint_times_s[index]
            end = self.waypoint_times_s[index + 1]
            if start <= elapsed_s < end:
                return index, index + 1, (elapsed_s - start) / (end - start)
        return 0, 0, 0.0

    def sample(self, elapsed_s: float) -> ArmSample:
        elapsed_s = max(0.0, float(elapsed_s))
        finite_duration = self.trajectory_duration_s * self.loop_count
        finished = self.loop_count > 0 and elapsed_s >= finite_duration
        if finished:
            cycle_time = self.trajectory_duration_s
        elif self.trajectory_duration_s > 0.0 and (self.loop_count == 0 or self.loop_count > 1):
            cycle_time = elapsed_s % self.trajectory_duration_s
        else:
            cycle_time = elapsed_s
        start_index, end_index, ratio = self._segment(cycle_time)
        duration = (
            self.waypoint_times_s[end_index] - self.waypoint_times_s[start_index]
            if end_index != start_index
            else 0.0
        )
        positions = tuple(
            start + (end - start) * ratio
            for start, end in zip(self.positions[start_index], self.positions[end_index])
        )
        velocities = tuple(
            0.0 if finished or duration <= 0.0 else (end - start) / duration
            for start, end in zip(self.positions[start_index], self.positions[end_index])
        )
        gripper = None
        if self.publish_gripper:
            start = self.gripper_positions[start_index]
            end = self.gripper_positions[end_index]
            gripper = GripperSample(
                self.gripper_joint_name,
                start + (end - start) * ratio,
                0.0 if finished or duration <= 0.0 else (end - start) / duration,
            )
        return ArmSample(
            self.joint_names,
            positions,
            velocities,
            tuple(0.0 for _ in self.joint_names),
            gripper,
            finished,
        )


class ArmTrajectoryPlayback:
    """Prepend a fixed-duration transition from the current follower state."""

    def __init__(self, profile: ArmTrajectoryProfile) -> None:
        self.profile = profile
        self.active = False
        self.start_time_s = 0.0
        self.start_positions: Optional[Tuple[float, ...]] = None
        self.start_gripper: Optional[float] = None

    def start(self, now_s: float, follower_positions=None, follower_gripper=None) -> None:
        self.active = True
        self.start_time_s = now_s
        self.start_positions = None
        self.start_gripper = None
        if follower_positions is not None:
            positions = tuple(float(value) for value in follower_positions)
            if len(positions) != len(self.profile.joint_names):
                raise ValueError("follower arm state length does not match trajectory joints")
            for current, target in zip(positions, self.profile.positions[0]):
                speed = abs(target - current) / self.profile.initial_transition_duration_s
                if speed > self.profile.max_joint_velocity_rad_s + 1.0e-9:
                    raise ValueError("initial transition exceeds max_joint_velocity_rad_s")
            self.start_positions = positions
        if follower_gripper is not None and self.profile.publish_gripper:
            speed = abs(
                self.profile.gripper_positions[0] - follower_gripper
            ) / self.profile.initial_transition_duration_s
            if speed > self.profile.max_gripper_velocity_rad_s + 1.0e-9:
                raise ValueError("initial transition exceeds max_gripper_velocity_rad_s")
            self.start_gripper = float(follower_gripper)

    def sample(self, now_s: float) -> ArmSample:
        if not self.active:
            return self.profile.sample(0.0)
        elapsed = max(0.0, now_s - self.start_time_s)
        transition = self.profile.initial_transition_duration_s
        if elapsed < transition:
            base = self.profile.sample(0.0)
            if self.start_positions is None:
                return ArmSample(
                    base.joint_names,
                    base.positions,
                    tuple(0.0 for _ in base.positions),
                    base.efforts,
                    GripperSample(base.gripper.name, base.gripper.position, 0.0)
                    if base.gripper
                    else None,
                    False,
                )
            ratio = elapsed / transition
            positions = tuple(
                start + (target - start) * ratio
                for start, target in zip(self.start_positions, self.profile.positions[0])
            )
            velocities = tuple(
                (target - start) / transition
                for start, target in zip(self.start_positions, self.profile.positions[0])
            )
            gripper = base.gripper
            if gripper and self.start_gripper is not None:
                target = self.profile.gripper_positions[0]
                gripper = GripperSample(
                    gripper.name,
                    self.start_gripper + (target - self.start_gripper) * ratio,
                    (target - self.start_gripper) / transition,
                )
            return ArmSample(
                base.joint_names, positions, velocities, base.efforts, gripper, False
            )
        return self.profile.sample(elapsed - transition)


class ArmSyncHandshake:
    """Minimal ACETele leader handshake with strict real and relaxed sim guards."""

    FOLLOWER_TIMEOUT_S = 0.5
    STATUS_TIMEOUT_S = 0.5
    READY_DWELL_S = 0.2

    def __init__(self, require_follower_state: bool) -> None:
        self.require_follower_state = require_follower_state
        self.leader_mode = "sync_request"
        self.follower_status = ""
        self.last_follower_state_s: Optional[float] = None
        self.last_status_s: Optional[float] = None
        self.ready_since_s: Optional[float] = None

    def notify_follower_state(self, now_s: float) -> None:
        self.last_follower_state_s = now_s

    def notify_status(self, status: str, now_s: float) -> None:
        if status in {"idle", "aligning", "ready", "tracking", "lost", "fault"}:
            self.follower_status = status
            self.last_status_s = now_s

    def update(self, now_s: float):
        state_recent = (
            self.last_follower_state_s is not None
            and now_s - self.last_follower_state_s <= self.FOLLOWER_TIMEOUT_S
        )
        status_recent = (
            self.last_status_s is not None
            and now_s - self.last_status_s <= self.STATUS_TIMEOUT_S
        )
        ready = self.follower_status in {"ready", "tracking"}
        lost = self.follower_status in {"lost", "fault"}
        tracking_started = False
        if lost or (self.require_follower_state and not state_recent):
            self.leader_mode = "sync_request"
            self.ready_since_s = None
        elif self.leader_mode == "tracking":
            if not status_recent:
                self.leader_mode = "sync_request"
                self.ready_since_s = None
        elif not status_recent or not ready:
            self.leader_mode = "sync_request"
            self.ready_since_s = None
        elif self.leader_mode != "ready":
            self.leader_mode = "ready"
            self.ready_since_s = now_s
        elif self.ready_since_s is not None and now_s - self.ready_since_s >= self.READY_DWELL_S:
            self.leader_mode = "tracking"
            tracking_started = True
        return self.leader_mode, self.leader_mode == "tracking", tracking_started
