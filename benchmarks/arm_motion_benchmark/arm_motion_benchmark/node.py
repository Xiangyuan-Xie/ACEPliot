"""ACETele-compatible arm motion workload node."""

from __future__ import annotations

import math
import sys
from typing import Iterable, Optional, Tuple

from builtin_interfaces.msg import Time
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from arm_motion_benchmark.trajectory import (
    ArmSyncHandshake,
    ArmTrajectoryPlayback,
    ArmTrajectoryProfile,
)


def _status_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def _require_absolute(topic: str, name: str) -> str:
    if not topic.startswith("/"):
        raise ValueError(f"{name} must be an absolute ROS topic")
    return topic


class ArmMotionBenchmark(Node):
    """Run a synchronized arm motion while a separate reporter measures the base."""

    def __init__(self) -> None:
        super().__init__("arm_motion_benchmark")
        self.use_sim_clock = bool(self.declare_parameter("use_sim_clock", False).value)
        self.publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 100.0).value)
        if not math.isfinite(self.publish_rate_hz) or self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be finite and positive")
        self.period_s = 1.0 / self.publish_rate_hz
        self.profile = ArmTrajectoryProfile(
            self.declare_parameter(
                "joint_names", ["joint_1", "joint_2", "joint_3", "joint_4"]
            ).value,
            self.declare_parameter("segment_durations_s", [2.0, 2.0, 2.0]).value,
            self.declare_parameter(
                "positions",
                [
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.2,
                    -0.2,
                    0.15,
                    -0.15,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                ],
            ).value,
            float(self.declare_parameter("max_joint_velocity_rad_s", 1.5).value),
            int(self.declare_parameter("loop_count", 1).value),
            bool(self.declare_parameter("publish_gripper", True).value),
            self.declare_parameter("gripper_joint_name", "joint_5").value,
            self.declare_parameter("gripper_positions", [0.0, 0.5, 0.0]).value,
            float(self.declare_parameter("max_gripper_velocity_rad_s", 1.0).value),
        )
        self.handshake = ArmSyncHandshake(require_follower_state=not self.use_sim_clock)
        self.playback = ArmTrajectoryPlayback(self.profile)
        self._latest_sim_time_s: Optional[float] = None
        self._last_publish_time_s: Optional[float] = None
        self._latest_follower_positions: Optional[Tuple[float, ...]] = None
        self._latest_follower_gripper: Optional[float] = None
        self._status = ""
        self._finished = False

        arm_topic = _require_absolute(
            self.declare_parameter("arm_command_topic", "/ace_leader/arm/command").value,
            "arm_command_topic",
        )
        sync_topic = _require_absolute(
            self.declare_parameter("sync_mode_topic", "/ace_leader/arm/sync_mode").value,
            "sync_mode_topic",
        )
        gripper_topic = _require_absolute(
            self.declare_parameter(
                "gripper_command_topic", "/ace_leader/gripper/command"
            ).value,
            "gripper_command_topic",
        )
        status_topic = _require_absolute(
            self.declare_parameter("status_topic", "/arm_motion_benchmark/status").value,
            "status_topic",
        )
        follower_arm_topic = _require_absolute(
            self.declare_parameter(
                "follower_arm_state_topic", "/ace_follower/arm/state"
            ).value,
            "follower_arm_state_topic",
        )
        follower_status_topic = _require_absolute(
            self.declare_parameter(
                "follower_sync_status_topic", "/ace_follower/arm/sync_status"
            ).value,
            "follower_sync_status_topic",
        )
        follower_gripper_topic = _require_absolute(
            self.declare_parameter(
                "follower_gripper_state_topic", "/ace_follower/gripper/state"
            ).value,
            "follower_gripper_state_topic",
        )
        self.arm_publisher = self.create_publisher(JointState, arm_topic, 10)
        self.sync_publisher = self.create_publisher(String, sync_topic, 10)
        self.gripper_publisher = self.create_publisher(JointState, gripper_topic, 10)
        self.status_publisher = self.create_publisher(String, status_topic, _status_qos())
        self.create_subscription(
            JointState, follower_arm_topic, self._on_follower_arm, qos_profile_sensor_data
        )
        self.create_subscription(
            String,
            follower_status_topic,
            self._on_follower_status,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            JointState,
            follower_gripper_topic,
            self._on_follower_gripper,
            qos_profile_sensor_data,
        )
        self._timer = None
        if self.use_sim_clock:
            clock_topic = _require_absolute(
                self.declare_parameter("sim_clock_topic", "/acesim/clock").value,
                "sim_clock_topic",
            )
            self.create_subscription(
                Clock, clock_topic, self._on_sim_clock, qos_profile_sensor_data
            )
        else:
            self._timer = self.create_timer(self.period_s, self._on_real_timer)
        self._publish_status("waiting")

    def _now_s(self) -> float:
        if self.use_sim_clock and self._latest_sim_time_s is not None:
            return self._latest_sim_time_s
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _stamp(self, now_s: float) -> Time:
        seconds = math.floor(now_s)
        return Time(sec=int(seconds), nanosec=int((now_s - seconds) * 1.0e9))

    def _publish_status(self, status: str) -> None:
        if status == self._status:
            return
        self._status = status
        message = String()
        message.data = status
        self.status_publisher.publish(message)

    def _publish_sync(self, mode: str) -> None:
        message = String()
        message.data = mode
        self.sync_publisher.publish(message)

    def _on_follower_arm(self, message: JointState) -> None:
        if not message.position:
            return
        self._latest_follower_positions = tuple(message.position)
        self.handshake.notify_follower_state(self._now_s())

    def _on_follower_status(self, message: String) -> None:
        self.handshake.notify_status(message.data, self._now_s())

    def _on_follower_gripper(self, message: JointState) -> None:
        if message.position:
            self._latest_follower_gripper = float(message.position[0])

    def _on_sim_clock(self, message: Clock) -> None:
        now_s = float(message.clock.sec) + float(message.clock.nanosec) * 1.0e-9
        if self._latest_sim_time_s is not None and now_s < self._latest_sim_time_s:
            self._reset_after_clock_jump()
        self._latest_sim_time_s = now_s
        if (
            self._last_publish_time_s is None
            or now_s - self._last_publish_time_s >= self.period_s - 1.0e-6
        ):
            self._last_publish_time_s = now_s
            self._safe_tick(now_s)

    def _on_real_timer(self) -> None:
        self._safe_tick(self._now_s())

    def _reset_after_clock_jump(self) -> None:
        self._last_publish_time_s = None
        self.handshake = ArmSyncHandshake(require_follower_state=not self.use_sim_clock)
        self.playback = ArmTrajectoryPlayback(self.profile)
        self._latest_follower_positions = None
        self._latest_follower_gripper = None
        self._finished = False
        self._publish_status("waiting")

    def _safe_tick(self, now_s: float) -> None:
        try:
            self._tick(now_s)
        except Exception as error:  # noqa: BLE001
            self.get_logger().error(f"Arm motion benchmark failed: {error}")
            self._publish_sync("stop")
            self._publish_status("failed")

    def _tick(self, now_s: float) -> None:
        if self._finished:
            self._publish_sync("stop")
            return
        leader_mode, commands_allowed, tracking_started = self.handshake.update(now_s)
        self._publish_sync(leader_mode)
        if not commands_allowed:
            self._publish_status("waiting")
            return
        if tracking_started or not self.playback.active:
            self.playback.start(
                now_s,
                self._latest_follower_positions,
                self._latest_follower_gripper,
            )
            if self._latest_follower_positions is None:
                self.get_logger().warning(
                    "Follower arm state unavailable; starting at the first YAML waypoint."
                )
        self._publish_status("running")
        sample = self.playback.sample(now_s)
        stamp = self._stamp(now_s)
        arm = JointState()
        arm.header.stamp = stamp
        arm.name = list(sample.joint_names)
        arm.position = list(sample.positions)
        arm.velocity = list(sample.velocities)
        arm.effort = list(sample.efforts)
        self.arm_publisher.publish(arm)
        if sample.gripper is not None:
            gripper = JointState()
            gripper.header.stamp = stamp
            gripper.name = [sample.gripper.name]
            gripper.position = [sample.gripper.position]
            gripper.velocity = [sample.gripper.velocity]
            gripper.effort = [0.0]
            self.gripper_publisher.publish(gripper)
        if sample.finished:
            self._finished = True
            self._publish_sync("stop")
            self._publish_status("finished")
            self.get_logger().info("Arm motion completed; waiting for report finalization.")


def main(args: Iterable[str] | None = None) -> None:
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = ArmMotionBenchmark()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:  # noqa: BLE001
        logger = node.get_logger() if node is not None else rclpy.logging.get_logger(
            "arm_motion_benchmark"
        )
        logger.fatal(str(error))
        exit_code = 1
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except (Exception, KeyboardInterrupt):  # noqa: BLE001
                pass
        if rclpy.ok():
            rclpy.shutdown()
    if exit_code:
        sys.exit(exit_code)


if __name__ == "__main__":
    main()
