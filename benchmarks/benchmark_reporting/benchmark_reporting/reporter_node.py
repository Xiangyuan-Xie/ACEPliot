"""ROS node that records benchmark streams and writes reports on completion."""

from __future__ import annotations

import sys
import threading
import time
from typing import Dict, Iterable, List, Sequence, Tuple

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
import numpy as np
from px4_msgs.msg import VehicleOdometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

from benchmark_reporting.metrics import (
    align_linear,
    base_drift_metrics,
    derive_velocity,
    interpolate_at,
    interpolate_quaternions_at,
    quaternion_angular_errors,
    scalar_error_metrics,
    tracking_metrics,
)
from benchmark_reporting.plots import (
    plot_aerial_overview_bundle,
    plot_base_overview_bundle,
    plot_pose_overview_bundle,
    plot_scalar_error_bundle,
    plot_trajectory_bundle,
    plot_vector_error_bundle,
    plot_velocity_overview_bundle,
    plot_velocity_tracking_bundle,
)
from benchmark_reporting.report import ReportWriter, artifact_names


VALID_REPORT_TYPES = {"velocity", "base", "aerial", "px4_figure8", "ee_figure8"}


def _status_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def _header_time_s(message, fallback_s: float) -> float:
    header = getattr(message, "header", None)
    if header is None:
        return fallback_s
    stamp = header.stamp
    value = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
    return value if value > 0.0 else fallback_s


def _nwu_to_enu(position: Sequence[float]) -> Tuple[float, float, float]:
    return (-float(position[1]), float(position[0]), float(position[2]))


def _ned_to_enu(vector: Sequence[float]) -> Tuple[float, float, float]:
    return (float(vector[1]), float(vector[0]), -float(vector[2]))


def _rotate_vector_wxyz(
    quaternion: Sequence[float], vector: Sequence[float]
) -> Tuple[float, float, float]:
    values = np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm(values)
    if not np.isfinite(norm) or norm < 1.0e-12:
        raise ValueError("vehicle quaternion is invalid")
    values /= norm
    scalar = values[0]
    axis = values[1:]
    source = np.asarray(vector, dtype=float)
    rotated = (
        2.0 * np.dot(axis, source) * axis
        + (scalar * scalar - np.dot(axis, axis)) * source
        + 2.0 * scalar * np.cross(axis, source)
    )
    return tuple(float(value) for value in rotated)


class BenchmarkReporter(Node):
    """Collect and report one configured benchmark domain."""

    def __init__(self) -> None:
        super().__init__("benchmark_reporter")
        self.report_type = self.declare_parameter("report_type", "velocity").value
        if self.report_type not in VALID_REPORT_TYPES:
            raise ValueError(f"unsupported report_type: {self.report_type}")
        self.benchmark_name = self.declare_parameter(
            "benchmark_name", self.report_type
        ).value
        self.benchmark_package = self.declare_parameter(
            "benchmark_package", "benchmark_reporting"
        ).value
        self.max_gap_s = float(
            self.declare_parameter("max_alignment_gap_s", 0.1).value
        )
        self.use_sim_clock = bool(
            self.declare_parameter("use_sim_clock", False).value
        )
        self._latest_sim_time_s = None
        if self.use_sim_clock:
            clock_topic = self.declare_parameter(
                "sim_clock_topic", "/acesim/clock"
            ).value
            if not clock_topic.startswith("/"):
                raise ValueError("sim_clock_topic must be an absolute ROS topic")
            self.create_subscription(
                Clock, clock_topic, self._on_sim_clock, qos_profile_sensor_data
            )
        self.writer = ReportWriter(
            self.benchmark_name,
            self.declare_parameter("output_dir", "").value,
            package_name=self.benchmark_package,
        )

        self._finalized = False
        self._exit_code = 0
        self._finalize_lock = threading.Lock()
        self._finish_timer: threading.Timer | None = None
        self._statuses: Dict[str, str] = {}
        self._velocity_active = False
        self._base_active = False
        self._pose_active = False

        self.command_times: List[float] = []
        self.command_velocity: List[Tuple[float, float, float]] = []
        self.measured_times: List[float] = []
        self.measured_velocity: List[Tuple[float, float, float]] = []
        self.base_times: List[float] = []
        self.base_positions: List[Tuple[float, float, float]] = []
        self.reference_pose_times: List[float] = []
        self.reference_positions: List[Tuple[float, float, float]] = []
        self.reference_quaternions: List[Tuple[float, float, float, float]] = []
        self.reference_velocity_times: List[float] = []
        self.reference_velocities: List[Tuple[float, float, float]] = []
        self.actual_pose_times: List[float] = []
        self.actual_positions: List[Tuple[float, float, float]] = []
        self.actual_quaternions: List[Tuple[float, float, float, float]] = []
        self.actual_velocity_times: List[float] = []
        self.actual_velocities: List[Tuple[float, float, float]] = []

        if self.report_type in {"velocity", "aerial"}:
            self._configure_velocity()
        if self.report_type in {"base", "aerial"}:
            self._configure_base()
        if self.report_type in {"px4_figure8", "ee_figure8"}:
            self._configure_pose_tracking()

        self.get_logger().info(
            f"Recording {self.report_type} benchmark artifacts in '{self.writer.output_dir}'."
        )

    def _now_s(self) -> float:
        if self.use_sim_clock and self._latest_sim_time_s is not None:
            return self._latest_sim_time_s
        now = self.get_clock().now().nanoseconds * 1.0e-9
        return now if now > 0.0 else time.monotonic()

    def _on_sim_clock(self, message: Clock) -> None:
        self._latest_sim_time_s = (
            float(message.clock.sec) + float(message.clock.nanosec) * 1.0e-9
        )

    def _message_time_s(self, message) -> float:
        if self.use_sim_clock and self._latest_sim_time_s is not None:
            return self._latest_sim_time_s
        return _header_time_s(message, self._now_s())

    def _subscribe_status(self, key: str, parameter: str, default: str) -> None:
        topic = self.declare_parameter(parameter, default).value
        if not topic.startswith("/"):
            raise ValueError(f"{parameter} must be an absolute ROS topic")
        self._statuses[key] = "waiting"
        self.create_subscription(
            String,
            topic,
            lambda message, status_key=key: self._on_status(status_key, message.data),
            _status_qos(),
        )

    def _configure_velocity(self) -> None:
        command_topic = self.declare_parameter(
            "command_velocity_topic", "/velocity_tracking_benchmark/command_velocity"
        ).value
        measured_topic = self.declare_parameter(
            "measured_velocity_topic", "/velocity_tracking_benchmark/measured_velocity"
        ).value
        self.create_subscription(
            TwistStamped, command_topic, self._on_command_velocity, qos_profile_sensor_data
        )
        self.create_subscription(
            TwistStamped, measured_topic, self._on_measured_velocity, qos_profile_sensor_data
        )
        self._subscribe_status(
            "velocity",
            "velocity_status_topic",
            "/velocity_tracking_benchmark/status",
        )
        if self.report_type == "velocity":
            self._configure_vehicle_position(active_with="velocity")

    def _configure_base(self) -> None:
        self._subscribe_status(
            "arm", "arm_status_topic", "/arm_motion_benchmark/status"
        )
        self._configure_vehicle_position(active_with="base")

    def _configure_vehicle_position(self, active_with: str) -> None:
        source = self.declare_parameter("vehicle_position_source", "pose_stamped").value
        if source == "pose_stamped":
            topic = self.declare_parameter("vehicle_pose_topic", "/xxy/pose").value
            self.create_subscription(
                PoseStamped,
                topic,
                lambda message: self._on_vehicle_pose(message, active_with),
                qos_profile_sensor_data,
            )
        elif source == "odometry_pose":
            topic = self.declare_parameter(
                "vehicle_odometry_topic", "/acesim/vehicle/odometry"
            ).value
            self.create_subscription(
                Odometry,
                topic,
                lambda message: self._on_vehicle_odometry(message, active_with),
                qos_profile_sensor_data,
            )
        else:
            raise ValueError(f"unsupported vehicle_position_source: {source}")

    def _configure_pose_tracking(self) -> None:
        prefix = (
            "/figure8_trajectory/px4"
            if self.report_type == "px4_figure8"
            else "/figure8_trajectory/ee"
        )
        reference_pose_topic = self.declare_parameter(
            "reference_pose_topic", f"{prefix}/reference_pose"
        ).value
        reference_velocity_topic = self.declare_parameter(
            "reference_velocity_topic", f"{prefix}/reference_velocity"
        ).value
        self.create_subscription(
            PoseStamped,
            reference_pose_topic,
            self._on_reference_pose,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            TwistStamped,
            reference_velocity_topic,
            self._on_reference_velocity,
            qos_profile_sensor_data,
        )
        self._subscribe_status("figure8", "figure8_status_topic", f"{prefix}/status")
        if self.report_type == "px4_figure8":
            topic = self.declare_parameter(
                "actual_vehicle_odometry_topic", "/fmu/out/vehicle_odometry"
            ).value
            self.create_subscription(
                VehicleOdometry,
                topic,
                self._on_px4_vehicle_odometry,
                qos_profile_sensor_data,
            )
        else:
            topic = self.declare_parameter(
                "actual_ee_pose_topic", "/am_ee_pose/current_ee_pose"
            ).value
            self.create_subscription(
                PoseStamped, topic, self._on_actual_ee_pose, qos_profile_sensor_data
            )

    def _on_status(self, key: str, status: str) -> None:
        if status not in {"waiting", "running", "finished", "failed"}:
            return
        self._statuses[key] = status
        if key == "velocity":
            self._velocity_active = status == "running"
        elif key == "arm":
            self._base_active = status == "running"
        elif key == "figure8":
            self._pose_active = status == "running"

        if status == "failed":
            self._schedule_finalize("invalid", delay_s=0.0)
            return
        if all(value == "finished" for value in self._statuses.values()):
            self._schedule_finalize("complete", delay_s=0.2)

    def _schedule_finalize(self, status: str, delay_s: float) -> None:
        if self._finalized or self._finish_timer is not None:
            return
        self._finish_timer = threading.Timer(delay_s, self._finish_and_shutdown, (status,))
        self._finish_timer.daemon = True
        self._finish_timer.start()

    def _finish_and_shutdown(self, status: str) -> None:
        self.finalize(status)
        if rclpy.ok():
            rclpy.shutdown()

    def _append_sample(self, times: List[float], values: list, stamp: float, value) -> None:
        if times and stamp < times[-1]:
            times.clear()
            values.clear()
        times.append(stamp)
        values.append(value)

    def _on_command_velocity(self, message: TwistStamped) -> None:
        if not self._velocity_active:
            return
        stamp = self._message_time_s(message)
        value = (message.twist.linear.x, message.twist.linear.y, message.twist.linear.z)
        self._append_sample(self.command_times, self.command_velocity, stamp, value)

    def _on_measured_velocity(self, message: TwistStamped) -> None:
        if not self._velocity_active:
            return
        stamp = self._message_time_s(message)
        value = (message.twist.linear.x, message.twist.linear.y, message.twist.linear.z)
        self._append_sample(self.measured_times, self.measured_velocity, stamp, value)

    def _record_base_position(
        self, stamp: float, position: Tuple[float, float, float], active_with: str
    ) -> None:
        active = self._base_active if active_with == "base" else self._velocity_active
        if not active:
            return
        self._append_sample(self.base_times, self.base_positions, stamp, position)

    def _on_vehicle_pose(self, message: PoseStamped, active_with: str) -> None:
        position = (
            message.pose.position.x,
            message.pose.position.y,
            message.pose.position.z,
        )
        self._record_base_position(
            self._message_time_s(message), position, active_with
        )

    def _on_vehicle_odometry(self, message: Odometry, active_with: str) -> None:
        raw = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
            message.pose.pose.position.z,
        )
        self._record_base_position(
            self._message_time_s(message), _nwu_to_enu(raw), active_with
        )

    def _on_reference_pose(self, message: PoseStamped) -> None:
        if not self._pose_active:
            return
        stamp = self._message_time_s(message)
        position = (
            message.pose.position.x,
            message.pose.position.y,
            message.pose.position.z,
        )
        quaternion = (
            message.pose.orientation.w,
            message.pose.orientation.x,
            message.pose.orientation.y,
            message.pose.orientation.z,
        )
        if self.reference_pose_times and stamp < self.reference_pose_times[-1]:
            self.reference_quaternions.clear()
        self._append_sample(
            self.reference_pose_times, self.reference_positions, stamp, position
        )
        self.reference_quaternions.append(quaternion)

    def _on_reference_velocity(self, message: TwistStamped) -> None:
        if not self._pose_active:
            return
        stamp = self._message_time_s(message)
        velocity = (
            message.twist.linear.x,
            message.twist.linear.y,
            message.twist.linear.z,
        )
        self._append_sample(
            self.reference_velocity_times, self.reference_velocities, stamp, velocity
        )

    def _record_actual_pose(
        self,
        stamp: float,
        position: Tuple[float, float, float],
        quaternion: Tuple[float, float, float, float],
    ) -> None:
        if self.actual_pose_times and stamp < self.actual_pose_times[-1]:
            self.actual_quaternions.clear()
        self._append_sample(self.actual_pose_times, self.actual_positions, stamp, position)
        self.actual_quaternions.append(quaternion)

    def _on_actual_ee_pose(self, message: PoseStamped) -> None:
        if self._statuses.get("figure8") in {"finished", "failed"}:
            return
        self._record_actual_pose(
            self._message_time_s(message),
            (
                message.pose.position.x,
                message.pose.position.y,
                message.pose.position.z,
            ),
            (
                message.pose.orientation.w,
                message.pose.orientation.x,
                message.pose.orientation.y,
                message.pose.orientation.z,
            ),
        )

    def _on_px4_vehicle_odometry(self, message: VehicleOdometry) -> None:
        if self._statuses.get("figure8") in {"finished", "failed"}:
            return
        pose_frame_ned = int(getattr(VehicleOdometry, "POSE_FRAME_NED", 1))
        if int(message.pose_frame) != pose_frame_ned:
            return
        stamp = self._now_s()
        position = _ned_to_enu(message.position)
        self._record_actual_pose(stamp, position, (1.0, 0.0, 0.0, 0.0))
        velocity_frame_ned = int(getattr(VehicleOdometry, "VELOCITY_FRAME_NED", 1))
        if int(message.velocity_frame) == velocity_frame_ned:
            velocity_enu = _ned_to_enu(message.velocity)
        elif int(message.velocity_frame) == int(
            getattr(VehicleOdometry, "VELOCITY_FRAME_BODY_FRD", 3)
        ):
            velocity_enu = _ned_to_enu(
                _rotate_vector_wxyz(message.q, message.velocity)
            )
        else:
            return
        self._append_sample(
            self.actual_velocity_times,
            self.actual_velocities,
            stamp,
            velocity_enu,
        )

    def _valid_window(self, times: np.ndarray) -> bool:
        return len(times) >= 2 and float(times[-1] - times[0]) > 0.0

    def _velocity_report(self, prefix: str) -> Tuple[Dict[str, object], List, bool]:
        aligned = align_linear(
            self.command_times,
            self.command_velocity,
            self.measured_times,
            self.measured_velocity,
            self.max_gap_s,
        )
        if not self._valid_window(aligned.times_s):
            return {"valid_samples": len(aligned.times_s)}, [], False
        error = aligned.actual - aligned.reference
        rows = []
        for stamp, reference, actual, sample_error in zip(
            aligned.times_s, aligned.reference, aligned.actual, error
        ):
            rows.append(
                {
                    "time_s": float(stamp - aligned.times_s[0]),
                    "reference_vx_m_s": reference[0],
                    "reference_vy_m_s": reference[1],
                    "reference_vz_m_s": reference[2],
                    "actual_vx_m_s": actual[0],
                    "actual_vy_m_s": actual[1],
                    "actual_vz_m_s": actual[2],
                    "error_vx_m_s": sample_error[0],
                    "error_vy_m_s": sample_error[1],
                    "error_vz_m_s": sample_error[2],
                }
            )
        artifacts = [self.writer.write_csv(f"{prefix}samples.csv", rows)]
        artifacts.extend(
            plot_velocity_tracking_bundle(
                self.writer.output_dir,
                f"{prefix}tracking",
                aligned.times_s,
                aligned.reference,
                aligned.actual,
            )
        )
        artifacts.extend(
            plot_vector_error_bundle(
                self.writer.output_dir,
                f"{prefix}error",
                aligned.times_s,
                error,
                "Velocity error (m s$^{-1}$)",
            )
        )
        positions = None
        if self.base_positions:
            positions = np.asarray(self.base_positions, dtype=float)
            artifacts.extend(
                plot_trajectory_bundle(
                    self.writer.output_dir,
                    f"{prefix}trajectory",
                    positions,
                    origin=positions[0],
                )
            )
        metrics = tracking_metrics(aligned.reference, aligned.actual, ("x", "y", "z"))
        metrics.update(
            {
                "valid_samples": len(aligned.times_s),
                "duration_s": float(aligned.times_s[-1] - aligned.times_s[0]),
            }
        )
        if not prefix:
            artifacts.extend(
                plot_velocity_overview_bundle(
                    self.writer.output_dir,
                    "overview",
                    aligned.times_s,
                    aligned.reference,
                    aligned.actual,
                    positions,
                )
            )
        return metrics, artifacts, True

    def _base_report(self, prefix: str) -> Tuple[Dict[str, object], List, bool]:
        times = np.asarray(self.base_times, dtype=float)
        positions = np.asarray(self.base_positions, dtype=float)
        if not self._valid_window(times) or positions.shape != (len(times), 3):
            return {"valid_samples": len(times)}, [], False
        reference = positions[0]
        drift = positions - reference
        rows = []
        for stamp, position, sample_drift in zip(times, positions, drift):
            rows.append(
                {
                    "time_s": float(stamp - times[0]),
                    "x_enu_m": position[0],
                    "y_enu_m": position[1],
                    "z_enu_m": position[2],
                    "drift_x_m": sample_drift[0],
                    "drift_y_m": sample_drift[1],
                    "drift_z_m": sample_drift[2],
                    "drift_xy_m": float(np.linalg.norm(sample_drift[:2])),
                    "drift_xyz_m": float(np.linalg.norm(sample_drift)),
                }
            )
        artifacts = [self.writer.write_csv(f"{prefix}samples.csv", rows)]
        artifacts.extend(
            plot_trajectory_bundle(
                self.writer.output_dir,
                f"{prefix}trajectory",
                positions,
                origin=reference,
            )
        )
        artifacts.extend(
            plot_vector_error_bundle(
                self.writer.output_dir,
                f"{prefix}error",
                times,
                drift,
                "Base drift (m)",
            )
        )
        metrics = base_drift_metrics(positions)
        metrics.update(
            {"valid_samples": len(times), "duration_s": float(times[-1] - times[0])}
        )
        if not prefix:
            artifacts.extend(
                plot_base_overview_bundle(
                    self.writer.output_dir,
                    "overview",
                    times,
                    positions,
                )
            )
        return metrics, artifacts, True

    def _pose_report(self) -> Tuple[Dict[str, object], List, bool]:
        aligned = align_linear(
            self.reference_pose_times,
            self.reference_positions,
            self.actual_pose_times,
            self.actual_positions,
            self.max_gap_s,
        )
        if not self._valid_window(aligned.times_s):
            return {"valid_samples": len(aligned.times_s)}, [], False
        error = aligned.actual - aligned.reference
        rows = []
        for stamp, reference, actual, sample_error in zip(
            aligned.times_s, aligned.reference, aligned.actual, error
        ):
            rows.append(
                {
                    "time_s": float(stamp - aligned.times_s[0]),
                    "reference_x_m": reference[0],
                    "reference_y_m": reference[1],
                    "reference_z_m": reference[2],
                    "actual_x_m": actual[0],
                    "actual_y_m": actual[1],
                    "actual_z_m": actual[2],
                    "error_x_m": sample_error[0],
                    "error_y_m": sample_error[1],
                    "error_z_m": sample_error[2],
                }
            )
        artifacts = [self.writer.write_csv("position_samples.csv", rows)]
        artifacts.extend(
            plot_trajectory_bundle(
                self.writer.output_dir,
                "position_trajectory",
                aligned.actual,
                reference=aligned.reference,
            )
        )
        artifacts.extend(
            plot_vector_error_bundle(
                self.writer.output_dir,
                "position_error",
                aligned.times_s,
                error,
                "Position error (m)",
            )
        )
        metrics: Dict[str, object] = {
            "position": tracking_metrics(
                aligned.reference, aligned.actual, ("x", "y", "z")
            ),
            "valid_samples": len(aligned.times_s),
            "duration_s": float(aligned.times_s[-1] - aligned.times_s[0]),
        }

        reference_velocity, reference_valid = interpolate_at(
            aligned.times_s,
            self.reference_velocity_times,
            self.reference_velocities,
            self.max_gap_s,
        )
        if self.report_type == "px4_figure8":
            actual_velocity, actual_valid = interpolate_at(
                aligned.times_s,
                self.actual_velocity_times,
                self.actual_velocities,
                self.max_gap_s,
            )
        else:
            actual_velocity = derive_velocity(aligned.times_s, aligned.actual)
            actual_valid = np.ones(len(aligned.times_s), dtype=bool)
        velocity_valid = reference_valid & actual_valid
        velocity_times = None
        velocity_error = None
        if np.count_nonzero(velocity_valid) >= 2:
            velocity_times = aligned.times_s[velocity_valid]
            desired_velocity = reference_velocity[velocity_valid]
            measured_velocity = actual_velocity[velocity_valid]
            velocity_error = measured_velocity - desired_velocity
            velocity_rows = []
            for stamp, desired, measured, sample_error in zip(
                velocity_times, desired_velocity, measured_velocity, velocity_error
            ):
                velocity_rows.append(
                    {
                        "time_s": float(stamp - velocity_times[0]),
                        "reference_vx_m_s": desired[0],
                        "reference_vy_m_s": desired[1],
                        "reference_vz_m_s": desired[2],
                        "actual_vx_m_s": measured[0],
                        "actual_vy_m_s": measured[1],
                        "actual_vz_m_s": measured[2],
                        "error_vx_m_s": sample_error[0],
                        "error_vy_m_s": sample_error[1],
                        "error_vz_m_s": sample_error[2],
                    }
                )
            artifacts.append(self.writer.write_csv("velocity_samples.csv", velocity_rows))
            artifacts.extend(
                plot_velocity_tracking_bundle(
                    self.writer.output_dir,
                    "velocity_tracking",
                    velocity_times,
                    desired_velocity,
                    measured_velocity,
                )
            )
            artifacts.extend(
                plot_vector_error_bundle(
                    self.writer.output_dir,
                    "velocity_error",
                    velocity_times,
                    velocity_error,
                    "Velocity error (m s$^{-1}$)",
                )
            )
            metrics["velocity"] = tracking_metrics(
                desired_velocity, measured_velocity, ("x", "y", "z")
            )

        orientation_times = None
        angular_errors = None
        if self.report_type == "ee_figure8":
            desired_quaternion, desired_valid = interpolate_quaternions_at(
                aligned.times_s,
                self.reference_pose_times,
                self.reference_quaternions,
                self.max_gap_s,
            )
            actual_quaternion, actual_valid = interpolate_quaternions_at(
                aligned.times_s,
                self.actual_pose_times,
                self.actual_quaternions,
                self.max_gap_s,
            )
            quaternion_valid = desired_valid & actual_valid
            if np.count_nonzero(quaternion_valid) >= 2:
                orientation_times = aligned.times_s[quaternion_valid]
                angular_errors = quaternion_angular_errors(
                    desired_quaternion[quaternion_valid],
                    actual_quaternion[quaternion_valid],
                )
                metrics["orientation_angle_rad"] = scalar_error_metrics(angular_errors)
                artifacts.extend(
                    plot_scalar_error_bundle(
                        self.writer.output_dir,
                        "orientation_error",
                        orientation_times,
                        angular_errors,
                        "Orientation error (rad)",
                    )
                )
        artifacts.extend(
            plot_pose_overview_bundle(
                self.writer.output_dir,
                "overview",
                aligned.times_s,
                aligned.reference,
                aligned.actual,
                velocity_times,
                velocity_error,
                orientation_times,
                angular_errors,
            )
        )
        return metrics, artifacts, True

    def finalize(self, requested_status: str) -> None:
        with self._finalize_lock:
            if self._finalized:
                return
            self._finalized = True
            artifacts: List = []
            valid = False
            metrics: Dict[str, object]
            if self.report_type == "velocity":
                metrics, artifacts, valid = self._velocity_report("")
            elif self.report_type == "base":
                metrics, artifacts, valid = self._base_report("")
            elif self.report_type == "aerial":
                velocity_metrics, velocity_artifacts, velocity_valid = self._velocity_report(
                    "velocity_"
                )
                base_metrics, base_artifacts, base_valid = self._base_report("base_")
                metrics = {"velocity": velocity_metrics, "base": base_metrics}
                artifacts = velocity_artifacts + base_artifacts
                valid = velocity_valid and base_valid
                if valid:
                    aligned_velocity = align_linear(
                        self.command_times,
                        self.command_velocity,
                        self.measured_times,
                        self.measured_velocity,
                        self.max_gap_s,
                    )
                    artifacts.extend(
                        plot_aerial_overview_bundle(
                            self.writer.output_dir,
                            "overview",
                            aligned_velocity.times_s,
                            aligned_velocity.reference,
                            aligned_velocity.actual,
                            self.base_times,
                            self.base_positions,
                        )
                    )
            else:
                metrics, artifacts, valid = self._pose_report()

            status = requested_status
            if requested_status == "complete" and not valid:
                status = "invalid"
            if status == "invalid":
                self._exit_code = 2
            summary = {
                "schema_version": 1,
                "benchmark": self.benchmark_name,
                "report_type": self.report_type,
                "status": status,
                "metrics": metrics,
                "artifacts": artifact_names(artifacts),
            }
            summary_path = self.writer.write_json("summary.json", summary)
            message = f"Benchmark report status={status} written to '{summary_path}'."
            if rclpy.ok():
                self.get_logger().info(message)
            else:
                print(message, flush=True)


def main(args: Iterable[str] | None = None) -> None:
    rclpy.init(args=args)
    node: BenchmarkReporter | None = None
    exit_code = 0
    try:
        node = BenchmarkReporter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:  # noqa: BLE001
        if node is not None:
            node.get_logger().error(f"Benchmark reporter failed: {error}")
            node.finalize("invalid")
            exit_code = 2
        else:
            raise
    finally:
        if node is not None:
            node.finalize("incomplete")
            exit_code = max(exit_code, node._exit_code)
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
