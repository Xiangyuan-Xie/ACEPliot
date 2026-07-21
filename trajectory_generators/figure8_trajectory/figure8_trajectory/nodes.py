"""ROS 2 nodes that publish PX4 and end-effector Figure-8 references."""

from __future__ import annotations

import math
import sys
from typing import Callable, Iterable, Optional, Sequence, Tuple, Type

from geometry_msgs.msg import PoseArray, PoseStamped, TwistStamped
from nav_msgs.msg import Path
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleOdometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

from figure8_trajectory.profile import Figure8Parameters, Figure8Profile, TWO_PI


PREVIEW_OFFSETS_S = (0.0, 0.02, 0.04, 0.06, 1.0)


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


def _time_to_s(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _yaw_from_quaternion_wxyz(quaternion: Sequence[float]) -> float:
    w, x, y, z = (float(value) for value in quaternion)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    return (math.cos(0.5 * yaw), 0.0, 0.0, math.sin(0.5 * yaw))


def _ned_to_enu(vector: Sequence[float]) -> Tuple[float, float, float]:
    return (float(vector[1]), float(vector[0]), -float(vector[2]))


def _enu_to_ned(vector: Sequence[float]) -> list[float]:
    return [float(vector[1]), float(vector[0]), -float(vector[2])]


class _ClockDrivenNode(Node):
    """Drive callbacks from ACESim clock in sim and a ROS timer on hardware."""

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)
        self.use_sim_clock = bool(self.declare_parameter("use_sim_clock", False).value)
        self.publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 50.0).value)
        if not math.isfinite(self.publish_rate_hz) or self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be finite and positive")
        self.period_s = 1.0 / self.publish_rate_hz
        self._last_drive_s: Optional[float] = None
        self._drive_callback: Optional[Callable[[float, object], None]] = None
        self._timer = None
        self._clock_subscription = None

    def start_driver(self, callback: Callable[[float, object], None]) -> None:
        self._drive_callback = callback
        if self.use_sim_clock:
            topic = _require_absolute(
                self.declare_parameter("sim_clock_topic", "/acesim/clock").value,
                "sim_clock_topic",
            )
            self._clock_subscription = self.create_subscription(
                Clock, topic, self._on_sim_clock, qos_profile_sensor_data
            )
        else:
            self._timer = self.create_timer(self.period_s, self._on_real_timer)

    def _on_sim_clock(self, message: Clock) -> None:
        self._drive(_time_to_s(message.clock), message.clock)

    def _on_real_timer(self) -> None:
        now = self.get_clock().now()
        self._drive(now.nanoseconds * 1.0e-9, now.to_msg())

    def _drive(self, now_s: float, stamp) -> None:
        if self._drive_callback is None:
            return
        if self._last_drive_s is not None and now_s < self._last_drive_s:
            self._last_drive_s = None
            self.on_time_reset()
        if self._last_drive_s is not None and now_s - self._last_drive_s < self.period_s - 1.0e-6:
            return
        self._last_drive_s = now_s
        self._drive_callback(now_s, stamp)

    def on_time_reset(self) -> None:
        """Reset trajectory timing after a simulation clock rollback."""


class _Figure8NodeBase(_ClockDrivenNode):
    def __init__(self, node_name: str, status_default: str) -> None:
        super().__init__(node_name)
        parameters = Figure8Parameters(
            period_s=float(self.declare_parameter("period_s", 10.0).value),
            amplitude_x_m=float(self.declare_parameter("amplitude_x_m", 1.0).value),
            amplitude_y_m=float(self.declare_parameter("amplitude_y_m", 0.5).value),
            max_linear_speed_m_s=float(
                self.declare_parameter("max_linear_speed_m_s", 1.0).value
            ),
            transition_time_s=float(
                self.declare_parameter("transition_time_s", 1.0).value
            ),
            loops_to_run=int(self.declare_parameter("loops_to_run", 0).value),
        )
        self.profile = Figure8Profile(parameters)
        status_topic = _require_absolute(
            self.declare_parameter("status_topic", status_default).value, "status_topic"
        )
        self.status_publisher = self.create_publisher(String, status_topic, _status_qos())
        self._status = ""
        self._start_time_s: Optional[float] = None
        self._finished_ticks = 0
        self.publish_status("waiting")

    def publish_status(self, status: str) -> None:
        if status == self._status:
            return
        self._status = status
        message = String()
        message.data = status
        self.status_publisher.publish(message)

    def elapsed_s(self, now_s: float) -> float:
        if self._start_time_s is None:
            self._start_time_s = now_s
        return max(0.0, now_s - self._start_time_s)

    def finish_or_continue(self, finished: bool) -> None:
        if not finished:
            self.publish_status("running")
            return
        self.publish_status("finished")
        self._finished_ticks += 1
        if self._finished_ticks >= 3 and rclpy.ok():
            rclpy.shutdown()

    def on_time_reset(self) -> None:
        self._start_time_s = None
        self._finished_ticks = 0
        self.publish_status("waiting")


class Px4Figure8Trajectory(_Figure8NodeBase):
    """Publish a horizontal position Figure-8 directly to PX4 Offboard topics."""

    def __init__(self) -> None:
        super().__init__("px4_figure8_trajectory", "/figure8_trajectory/px4/status")
        self.vehicle_odometry_topic = _require_absolute(
            self.declare_parameter(
                "vehicle_odometry_topic", "/fmu/out/vehicle_odometry"
            ).value,
            "vehicle_odometry_topic",
        )
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
        reference_pose_topic = _require_absolute(
            self.declare_parameter(
                "reference_pose_topic", "/figure8_trajectory/px4/reference_pose"
            ).value,
            "reference_pose_topic",
        )
        reference_velocity_topic = _require_absolute(
            self.declare_parameter(
                "reference_velocity_topic", "/figure8_trajectory/px4/reference_velocity"
            ).value,
            "reference_velocity_topic",
        )
        path_topic = _require_absolute(
            self.declare_parameter(
                "path_topic", "/figure8_trajectory/px4/path"
            ).value,
            "path_topic",
        )
        self.publish_path = bool(self.declare_parameter("publish_path", True).value)
        self.offboard_publisher = self.create_publisher(
            OffboardControlMode, offboard_topic, 10
        )
        self.setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, setpoint_topic, 10
        )
        self.reference_pose_publisher = self.create_publisher(
            PoseStamped, reference_pose_topic, 10
        )
        self.reference_velocity_publisher = self.create_publisher(
            TwistStamped, reference_velocity_topic, 10
        )
        self.path_publisher = self.create_publisher(Path, path_topic, 1)
        self.create_subscription(
            VehicleOdometry,
            self.vehicle_odometry_topic,
            self._on_vehicle_odometry,
            qos_profile_sensor_data,
        )
        self._latest_origin: Optional[Tuple[Tuple[float, float, float], float]] = None
        self._origin: Optional[Tuple[float, float, float]] = None
        self._yaw_enu = 0.0
        self._path_published = False
        self.start_driver(self._publish_at)
        self.get_logger().info(
            "PX4 Figure-8 ready: period=%.3f s, amplitude=(%.3f, %.3f) m, max_speed=%.3f m/s."
            % (
                self.profile.parameters.period_s,
                self.profile.parameters.amplitude_x_m,
                self.profile.parameters.amplitude_y_m,
                self.profile.theoretical_max_speed_m_s,
            )
        )

    def _on_vehicle_odometry(self, message: VehicleOdometry) -> None:
        pose_frame_ned = int(getattr(VehicleOdometry, "POSE_FRAME_NED", 1))
        if int(message.pose_frame) != pose_frame_ned:
            return
        position = _ned_to_enu(message.position)
        yaw_ned = _yaw_from_quaternion_wxyz(message.q)
        self._latest_origin = (position, _wrap_pi(math.pi / 2.0 - yaw_ned))

    def _publish_at(self, now_s: float, stamp) -> None:
        if self._origin is None:
            if self._latest_origin is None:
                return
            self._origin, self._yaw_enu = self._latest_origin
            self._start_time_s = now_s
        sample = self.profile.sample(self.elapsed_s(now_s))
        position = (
            self._origin[0] + sample.position_xy_m[0],
            self._origin[1] + sample.position_xy_m[1],
            self._origin[2],
        )
        velocity = (sample.velocity_xy_m_s[0], sample.velocity_xy_m_s[1], 0.0)
        acceleration = (
            sample.acceleration_xy_m_s2[0],
            sample.acceleration_xy_m_s2[1],
            0.0,
        )
        timestamp_us = max(0, int(now_s * 1.0e6))

        mode = OffboardControlMode()
        mode.timestamp = timestamp_us
        mode.position = True
        mode.velocity = False
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
        setpoint.position = _enu_to_ned(position)
        setpoint.velocity = _enu_to_ned(velocity)
        setpoint.acceleration = _enu_to_ned(acceleration)
        setpoint.jerk = [math.nan, math.nan, math.nan]
        setpoint.yaw = _wrap_pi(math.pi / 2.0 - self._yaw_enu)
        setpoint.yawspeed = 0.0
        self.setpoint_publisher.publish(setpoint)
        self._publish_reference(stamp, position, velocity)
        if self.publish_path and not self._path_published:
            self.path_publisher.publish(self._make_path(stamp))
            self._path_published = True
        self.finish_or_continue(sample.finished)

    def _publish_reference(self, stamp, position, velocity) -> None:
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "world"
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = position
        quaternion = _yaw_to_quaternion(self._yaw_enu)
        (
            pose.pose.orientation.w,
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
        ) = quaternion
        self.reference_pose_publisher.publish(pose)
        twist = TwistStamped()
        twist.header = pose.header
        twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z = velocity
        self.reference_velocity_publisher.publish(twist)

    def _make_path(self, stamp) -> Path:
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = "world"
        for index in range(201):
            phase = TWO_PI * index / 200.0
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = (
                self._origin[0]
                + self.profile.parameters.amplitude_x_m * math.sin(phase)
            )
            pose.pose.position.y = (
                self._origin[1]
                + self.profile.parameters.amplitude_y_m * math.sin(2.0 * phase)
            )
            pose.pose.position.z = self._origin[2]
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        return path

    def on_time_reset(self) -> None:
        super().on_time_reset()
        self._origin = None
        self._path_published = False


class EeFigure8Trajectory(_Figure8NodeBase):
    """Publish the five-pose AM EE preview along a horizontal Figure-8."""

    def __init__(self) -> None:
        super().__init__("ee_figure8_trajectory", "/figure8_trajectory/ee/status")
        self.target_frame_id = self.declare_parameter("target_frame_id", "world").value
        current_topic = _require_absolute(
            self.declare_parameter(
                "current_ee_pose_topic", "/am_ee_pose/current_ee_pose"
            ).value,
            "current_ee_pose_topic",
        )
        preview_topic = _require_absolute(
            self.declare_parameter(
                "target_preview_topic", "/am_ee_pose/trajectory_preview"
            ).value,
            "target_preview_topic",
        )
        reference_pose_topic = _require_absolute(
            self.declare_parameter(
                "reference_pose_topic", "/figure8_trajectory/ee/reference_pose"
            ).value,
            "reference_pose_topic",
        )
        reference_velocity_topic = _require_absolute(
            self.declare_parameter(
                "reference_velocity_topic", "/figure8_trajectory/ee/reference_velocity"
            ).value,
            "reference_velocity_topic",
        )
        self.preview_publisher = self.create_publisher(PoseArray, preview_topic, 10)
        self.reference_pose_publisher = self.create_publisher(
            PoseStamped, reference_pose_topic, 10
        )
        self.reference_velocity_publisher = self.create_publisher(
            TwistStamped, reference_velocity_topic, 10
        )
        self.create_subscription(
            PoseStamped, current_topic, self._on_current_pose, qos_profile_sensor_data
        )
        self._origin_position: Optional[Tuple[float, float, float]] = None
        self._origin_quaternion: Optional[Tuple[float, float, float, float]] = None
        self.start_driver(self._publish_at)

    def _on_current_pose(self, message: PoseStamped) -> None:
        if self._origin_position is not None or message.header.frame_id != self.target_frame_id:
            return
        quaternion = (
            message.pose.orientation.w,
            message.pose.orientation.x,
            message.pose.orientation.y,
            message.pose.orientation.z,
        )
        norm = math.sqrt(sum(value * value for value in quaternion))
        if norm < 1.0e-6:
            return
        self._origin_position = (
            message.pose.position.x,
            message.pose.position.y,
            message.pose.position.z,
        )
        self._origin_quaternion = tuple(value / norm for value in quaternion)

    def _sample_pose(self, elapsed_s: float):
        sample = self.profile.sample(elapsed_s)
        position = (
            self._origin_position[0] + sample.position_xy_m[0],
            self._origin_position[1] + sample.position_xy_m[1],
            self._origin_position[2],
        )
        velocity = (sample.velocity_xy_m_s[0], sample.velocity_xy_m_s[1], 0.0)
        return sample, position, velocity

    def _publish_at(self, now_s: float, stamp) -> None:
        if self._origin_position is None or self._origin_quaternion is None:
            return
        elapsed = self.elapsed_s(now_s)
        current, position, velocity = self._sample_pose(elapsed)
        preview = PoseArray()
        preview.header.stamp = stamp
        preview.header.frame_id = self.target_frame_id
        for offset in PREVIEW_OFFSETS_S:
            _, preview_position, _ = self._sample_pose(elapsed + offset)
            pose = PoseStamped().pose
            pose.position.x, pose.position.y, pose.position.z = preview_position
            (
                pose.orientation.w,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
            ) = self._origin_quaternion
            preview.poses.append(pose)
        self.preview_publisher.publish(preview)

        reference_pose = PoseStamped()
        reference_pose.header = preview.header
        reference_pose.pose = preview.poses[0]
        self.reference_pose_publisher.publish(reference_pose)
        reference_velocity = TwistStamped()
        reference_velocity.header = preview.header
        (
            reference_velocity.twist.linear.x,
            reference_velocity.twist.linear.y,
            reference_velocity.twist.linear.z,
        ) = velocity
        self.reference_velocity_publisher.publish(reference_velocity)
        self.finish_or_continue(current.finished)

    def on_time_reset(self) -> None:
        super().on_time_reset()
        self._origin_position = None
        self._origin_quaternion = None


def _run(node_type: Type[Node], args: Iterable[str] | None = None) -> None:
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = node_type()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:  # noqa: BLE001
        logger = (
            node.get_logger()
            if node is not None
            else rclpy.logging.get_logger(node_type.__name__)
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


def px4_main(args: Iterable[str] | None = None) -> None:
    _run(Px4Figure8Trajectory, args)


def ee_main(args: Iterable[str] | None = None) -> None:
    _run(EeFigure8Trajectory, args)
