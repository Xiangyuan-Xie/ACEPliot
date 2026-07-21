"""PX4 velocity benchmark workload node."""

from __future__ import annotations

import math
import sys
from typing import Iterable, Optional, Sequence, Tuple

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

from velocity_tracking_benchmark.profile import PoseVelocityEstimator, VelocityProfile


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


def _stamp_s(message, fallback_s: float) -> float:
    stamp = message.header.stamp
    value = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
    return value if value > 0.0 else fallback_s


def _nwu_to_enu(position: Sequence[float]) -> Tuple[float, float, float]:
    return (-float(position[1]), float(position[0]), float(position[2]))


class VelocityTrackingBenchmark(Node):
    """Publish a stage velocity profile and measured feedback streams."""

    def __init__(self) -> None:
        super().__init__("velocity_tracking_benchmark")
        self.use_sim_clock = bool(self.declare_parameter("use_sim_clock", False).value)
        self.publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 50.0).value)
        if not math.isfinite(self.publish_rate_hz) or self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be finite and positive")
        self.period_s = 1.0 / self.publish_rate_hz
        self.profile = VelocityProfile(
            self.declare_parameter("profile.durations_s", [2.0, 2.0, 2.0]).value,
            self.declare_parameter("profile.vx_m_s", [0.25, 0.0, 0.0]).value,
            self.declare_parameter("profile.vy_m_s", [0.0, 0.25, 0.0]).value,
            self.declare_parameter("profile.vz_m_s", [0.0, 0.0, 0.0]).value,
            self.declare_parameter("profile.yaw_rate_rad_s", [0.0, 0.0, 0.0]).value,
            self.declare_parameter("profile.loop", False).value,
            float(self.declare_parameter("profile.max_linear_speed_m_s", 2.0).value),
            float(self.declare_parameter("profile.max_yaw_rate_rad_s", 1.0).value),
        )
        self.estimator = PoseVelocityEstimator(
            float(self.declare_parameter("velocity_estimator.min_dt_s", 0.002).value),
            float(self.declare_parameter("velocity_estimator.max_dt_s", 0.5).value),
            float(self.declare_parameter("velocity_estimator.low_pass_alpha", 0.5).value),
        )
        self.frame_id = self.declare_parameter("velocity_frame_id", "world").value
        self._latest_sim_time_s: Optional[float] = None
        self._last_publish_time_s: Optional[float] = None
        self._start_time_s: Optional[float] = None
        self._last_measured_velocity: Optional[Tuple[float, float, float]] = None
        self._status = ""

        offboard_topic = _require_absolute(
            self.declare_parameter(
                "offboard_control_mode_topic", "/fmu/in/offboard_control_mode"
            ).value,
            "offboard_control_mode_topic",
        )
        setpoint_topic = _require_absolute(
            self.declare_parameter(
                "trajectory_setpoint_topic", "/fmu/in/trajectory_setpoint"
            ).value,
            "trajectory_setpoint_topic",
        )
        command_topic = _require_absolute(
            self.declare_parameter(
                "command_velocity_topic",
                "/velocity_tracking_benchmark/command_velocity",
            ).value,
            "command_velocity_topic",
        )
        measured_topic = _require_absolute(
            self.declare_parameter(
                "measured_velocity_topic",
                "/velocity_tracking_benchmark/measured_velocity",
            ).value,
            "measured_velocity_topic",
        )
        error_topic = _require_absolute(
            self.declare_parameter(
                "velocity_error_topic", "/velocity_tracking_benchmark/velocity_error"
            ).value,
            "velocity_error_topic",
        )
        status_topic = _require_absolute(
            self.declare_parameter(
                "status_topic", "/velocity_tracking_benchmark/status"
            ).value,
            "status_topic",
        )
        self.offboard_publisher = self.create_publisher(
            OffboardControlMode, offboard_topic, 10
        )
        self.setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, setpoint_topic, 10
        )
        self.command_publisher = self.create_publisher(TwistStamped, command_topic, 10)
        self.measured_publisher = self.create_publisher(TwistStamped, measured_topic, 10)
        self.error_publisher = self.create_publisher(TwistStamped, error_topic, 10)
        self.status_publisher = self.create_publisher(String, status_topic, _status_qos())

        measurement_source = self.declare_parameter(
            "measurement_source", "pose_stamped"
        ).value
        if measurement_source == "pose_stamped":
            topic = _require_absolute(
                self.declare_parameter("mocap_pose_topic", "/xxy/pose").value,
                "mocap_pose_topic",
            )
            self.create_subscription(
                PoseStamped, topic, self._on_pose, qos_profile_sensor_data
            )
        elif measurement_source == "odometry_pose":
            topic = _require_absolute(
                self.declare_parameter(
                    "measured_odometry_topic", "/acesim/vehicle/odometry"
                ).value,
                "measured_odometry_topic",
            )
            self.create_subscription(
                Odometry, topic, self._on_odometry, qos_profile_sensor_data
            )
        else:
            raise ValueError(f"unsupported measurement_source: {measurement_source}")

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

    def _stamp_message(self, stamp_s: float):
        seconds = math.floor(stamp_s)
        from builtin_interfaces.msg import Time

        return Time(sec=int(seconds), nanosec=int((stamp_s - seconds) * 1.0e9))

    def _publish_status(self, status: str) -> None:
        if status == self._status:
            return
        self._status = status
        message = String()
        message.data = status
        self.status_publisher.publish(message)

    def _on_pose(self, message: PoseStamped) -> None:
        self._update_measurement(
            _stamp_s(message, self._now_s()),
            (message.pose.position.x, message.pose.position.y, message.pose.position.z),
        )

    def _on_odometry(self, message: Odometry) -> None:
        position = _nwu_to_enu(
            (
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                message.pose.pose.position.z,
            )
        )
        self._update_measurement(_stamp_s(message, self._now_s()), position)

    def _update_measurement(self, stamp_s: float, position) -> None:
        velocity = self.estimator.update(stamp_s, position)
        if velocity is None:
            return
        self._last_measured_velocity = velocity
        self.measured_publisher.publish(self._twist(stamp_s, velocity, 0.0))

    def _on_sim_clock(self, message: Clock) -> None:
        now_s = float(message.clock.sec) + float(message.clock.nanosec) * 1.0e-9
        if self._latest_sim_time_s is not None and now_s < self._latest_sim_time_s:
            self._reset_timing()
        self._latest_sim_time_s = now_s
        if (
            self._last_publish_time_s is None
            or now_s - self._last_publish_time_s >= self.period_s - 1.0e-6
        ):
            self._last_publish_time_s = now_s
            self._publish_command(now_s)

    def _on_real_timer(self) -> None:
        self._publish_command(self._now_s())

    def _reset_timing(self) -> None:
        self._last_publish_time_s = None
        self._start_time_s = None
        self.estimator.reset()
        self._publish_status("waiting")

    def _twist(self, stamp_s: float, velocity, yaw_rate: float) -> TwistStamped:
        message = TwistStamped()
        message.header.stamp = self._stamp_message(stamp_s)
        message.header.frame_id = self.frame_id
        (
            message.twist.linear.x,
            message.twist.linear.y,
            message.twist.linear.z,
        ) = velocity
        message.twist.angular.z = yaw_rate
        return message

    def _publish_command(self, now_s: float) -> None:
        if self._start_time_s is None:
            self._start_time_s = now_s
        command = self.profile.sample(now_s - self._start_time_s)
        if not command.finished:
            self._publish_status("running")
        timestamp_us = max(0, int(now_s * 1.0e6))
        mode = OffboardControlMode()
        mode.timestamp = timestamp_us
        mode.position = False
        mode.velocity = True
        mode.acceleration = False
        mode.attitude = False
        mode.body_rate = False
        if hasattr(mode, "thrust_and_torque"):
            mode.thrust_and_torque = False
        if hasattr(mode, "direct_actuator"):
            mode.direct_actuator = False
        self.offboard_publisher.publish(mode)

        setpoint = TrajectorySetpoint()
        setpoint.timestamp = timestamp_us
        setpoint.position = [math.nan, math.nan, math.nan]
        setpoint.velocity = [
            command.linear_enu_m_s[1],
            command.linear_enu_m_s[0],
            -command.linear_enu_m_s[2],
        ]
        setpoint.acceleration = [math.nan, math.nan, math.nan]
        setpoint.jerk = [math.nan, math.nan, math.nan]
        setpoint.yaw = math.nan
        setpoint.yawspeed = -command.yaw_rate_enu_rad_s
        self.setpoint_publisher.publish(setpoint)
        self.command_publisher.publish(
            self._twist(
                now_s, command.linear_enu_m_s, command.yaw_rate_enu_rad_s
            )
        )
        if self._last_measured_velocity is not None:
            error = tuple(
                desired - measured
                for desired, measured in zip(
                    command.linear_enu_m_s, self._last_measured_velocity
                )
            )
            self.error_publisher.publish(self._twist(now_s, error, 0.0))
        if command.finished:
            self._publish_status("finished")


def main(args: Iterable[str] | None = None) -> None:
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = VelocityTrackingBenchmark()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:  # noqa: BLE001
        logger = node.get_logger() if node is not None else rclpy.logging.get_logger(
            "velocity_tracking_benchmark"
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
