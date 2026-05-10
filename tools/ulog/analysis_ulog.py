#!/usr/bin/env python3

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import numpy as np
from pyulog import ULog

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from utils import (
    attitude_frd_to_ned_to_flu_to_enu_quat,
    finite_rows,
    interpolate_columns,
    ned_to_enu_positions,
    quat_wxyz_to_euler,
    quat_wxyz_to_xyzw,
    relative_seconds,
    rotate_vectors_by_quat_wxyz,
)

DEFAULT_OUTPUT_DIR = "analysis_outputs"
DEFAULT_MAN_DEADZONE = 0.1
DEFAULT_MPC_HOLD_MAX_XY = 0.8
DEFAULT_MPC_HOLD_MAX_Z = 0.6
DEFAULT_SERIES_WIDTH = 2.0
FLAG_SERIES_WIDTH = 2.4
VEHICLE_BODY_AXIS_LENGTH = 0.28
VEHICLE_BODY_AXIS_RADIUS = 0.012

AXIS_COLORS = {
    "x": [230, 85, 85, 255],
    "y": [70, 175, 95, 255],
    "z": [70, 120, 230, 255],
    "roll": [230, 85, 85, 255],
    "pitch": [70, 175, 95, 255],
    "yaw": [70, 120, 230, 255],
}

CHANNEL_COLORS = (
    [230, 85, 85, 255],
    [70, 175, 95, 255],
    [70, 120, 230, 255],
    [215, 155, 55, 255],
    [180, 95, 220, 255],
    [55, 170, 180, 255],
)

FLAG_PALETTE = (
    [37, 99, 235, 255],
    [5, 150, 105, 255],
    [217, 119, 6, 255],
    [220, 38, 38, 255],
    [124, 58, 237, 255],
    [8, 145, 178, 255],
    [202, 138, 4, 255],
    [219, 39, 119, 255],
    [75, 85, 99, 255],
    [22, 163, 74, 255],
)

FLAG_COLORS = {
    "/am/status/vehicle_status/nav_state/value": [75, 85, 99, 255],
    "/am/status/vehicle_status/nav_state/is_manual": [100, 116, 139, 255],
    "/am/status/vehicle_status/nav_state/is_posctl": [37, 99, 235, 255],
    "/am/status/vehicle_status/nav_state/is_am_position": [5, 150, 105, 255],
    "/am/status/vehicle_status/nav_state/is_am_offboard": [124, 58, 237, 255],
    "/am/status/vehicle_status/nav_state/is_am_test": [217, 119, 6, 255],
    "/am/status/am_pos_control/module_running": [5, 150, 105, 255],
    "/am/status/am_pos_control/manual_control_available": [37, 99, 235, 255],
    "/am/status/am_pos_control/arm_state_valid": [22, 163, 74, 255],
    "/am/status/am_pos_control/trajectory_setpoint_valid": [8, 145, 178, 255],
    "/am/status/am_pos_control/offboard_control_mode_fresh": [217, 119, 6, 255],
    "/am/status/am_pos_control/offboard_control_mode_supported": [124, 58, 237, 255],
    "/am/status/am_pos_control/am_position_available": [14, 165, 233, 255],
    "/am/status/am_pos_control/am_offboard_available": [168, 85, 247, 255],
    "/am/status/offboard_control_mode/position": [37, 99, 235, 255],
    "/am/status/offboard_control_mode/velocity": [5, 150, 105, 255],
    "/am/status/offboard_control_mode/acceleration": [217, 119, 6, 255],
    "/am/status/offboard_control_mode/attitude": [124, 58, 237, 255],
    "/am/status/offboard_control_mode/body_rate": [219, 39, 119, 255],
    "/am/status/offboard_control_mode/actuator": [8, 145, 178, 255],
    "/am/status/vehicle_status/arming_state": [37, 99, 235, 255],
    "/am/status/vehicle_status/failsafe": [220, 38, 38, 255],
    "/am/status/vehicle_status/pre_flight_checks_pass": [22, 163, 74, 255],
    "/am/policy/failure_flags": [127, 29, 29, 255],
    "/am/policy/failure_flags/vehicle_state_invalid": [220, 38, 38, 255],
    "/am/policy/failure_flags/arm_state_invalid": [217, 119, 6, 255],
    "/am/policy/failure_flags/am_setpoint_invalid": [202, 138, 4, 255],
    "/am/policy/failure_flags/am_inference_failed": [219, 39, 119, 255],
    "/am/policy/degraded_flags": [120, 113, 108, 255],
    "/am/policy/degraded_flags/local_xy_invalid": [234, 88, 12, 255],
    "/am/policy/degraded_flags/local_vxy_invalid": [14, 116, 144, 255],
    "/am/policy/degraded_flags/setpoint_defaulted": [124, 58, 237, 255],
    "/am/policy/manual_hover_activation/manual_xy_hold": [37, 99, 235, 255],
    "/am/policy/manual_hover_activation/manual_z_hold": [5, 150, 105, 255],
    "/am/policy/manual_hover_activation/manual_hover_active": [220, 38, 38, 255],
}

NAV_STATE_NAMES = {
    0: "MANUAL",
    1: "ALTCTL",
    2: "POSCTL",
    3: "AUTO_MISSION",
    4: "AUTO_LOITER",
    5: "AUTO_RTL",
    7: "AM_POSITION",
    9: "AM_OFFBOARD",
    10: "ACRO",
    11: "AM_TEST",
    14: "OFFBOARD",
    15: "STAB",
    17: "AUTO_TAKEOFF",
    18: "AUTO_LAND",
    19: "AUTO_FOLLOW_TARGET",
    23: "EXTERNAL1",
    24: "EXTERNAL2",
    25: "EXTERNAL3",
    26: "EXTERNAL4",
    27: "EXTERNAL5",
    28: "EXTERNAL6",
    29: "EXTERNAL7",
    30: "EXTERNAL8",
}

AM_EXPECTED_NAV_STATES = {
    0: "MANUAL",
    2: "POSCTL",
    7: "AM_POSITION",
    9: "AM_OFFBOARD",
    11: "AM_TEST",
}

AM_DEGRADED_FLAGS = {
    1: "LOCAL_XY_INVALID",
    2: "LOCAL_VXY_INVALID",
    4: "SETPOINT_DEFAULTED",
}

AM_FAILURE_FLAGS = {
    1: "VEHICLE_STATE_INVALID",
    2: "ARM_STATE_INVALID",
    4: "AM_SETPOINT_INVALID",
    8: "AM_INFERENCE_FAILED",
}


@dataclass(frozen=True)
class RerunExportResult:
    rrd_path: Path
    topic_count: int
    field_count: int
    sample_count: int
    am_view_count: int
    opened_viewer: bool


class RerunUnavailableError(ImportError):
    def __init__(self) -> None:
        super().__init__("Rerun is not installed. Install with: python3 -m pip install rerun-sdk")


@dataclass(frozen=True)
class TimeSeriesGroup:
    name: str
    origin: str
    contents: tuple[str, ...]
    axis_y: tuple[float, float] | None = None


@dataclass(frozen=True)
class SceneEntity:
    path: str
    label: str
    child_frame: str | None = None
    parent_frame: str | None = None


SCENE_ENTITIES = {
    "vehicle_path": SceneEntity("/am/scene/vehicle/path", "vehicle_actual_path"),
    "vehicle_frame": SceneEntity("/am/scene/vehicle/frame", "vehicle_flu", "vehicle_flu", "world_enu"),
    "vehicle_body_axes": SceneEntity("/am/scene/vehicle/body_axes", "vehicle_body_axes"),
    "vehicle_body_axis_x": SceneEntity("/am/scene/vehicle/body_axes/x", "vehicle_body_axis_x"),
    "vehicle_body_axis_y": SceneEntity("/am/scene/vehicle/body_axes/y", "vehicle_body_axis_y"),
    "vehicle_body_axis_z": SceneEntity("/am/scene/vehicle/body_axes/z", "vehicle_body_axis_z"),
}

SERIES_LABELS = {
    "/am/state/local_position/position_ned/x": "actual_pos_x",
    "/am/state/local_position/position_ned/y": "actual_pos_y",
    "/am/state/local_position/position_ned/z": "actual_pos_z",
    "/am/setpoint/trajectory/position_ned/x": "setpoint_pos_x",
    "/am/setpoint/trajectory/position_ned/y": "setpoint_pos_y",
    "/am/setpoint/trajectory/position_ned/z": "setpoint_pos_z",
    "/am/state/local_position/velocity_ned/x": "actual_vx",
    "/am/state/local_position/velocity_ned/y": "actual_vy",
    "/am/state/local_position/velocity_ned/z": "actual_vz",
    "/am/setpoint/trajectory/velocity_ned/x": "setpoint_vx",
    "/am/setpoint/trajectory/velocity_ned/y": "setpoint_vy",
    "/am/setpoint/trajectory/velocity_ned/z": "setpoint_vz",
    "/am/state/attitude/roll": "actual_roll",
    "/am/state/attitude/pitch": "actual_pitch",
    "/am/state/attitude/yaw": "actual_yaw",
    "/am/setpoint/attitude/roll": "setpoint_roll",
    "/am/setpoint/attitude/pitch": "setpoint_pitch",
    "/am/setpoint/attitude/yaw": "setpoint_yaw",
    "/am/setpoint/trajectory/yaw": "setpoint_yaw",
    "/am/state/angular_velocity/body_rates_frd/x": "actual_roll_rate",
    "/am/state/angular_velocity/body_rates_frd/y": "actual_pitch_rate",
    "/am/state/angular_velocity/body_rates_frd/z": "actual_yaw_rate",
    "/am/control/rates_setpoint/roll": "setpoint_roll_rate",
    "/am/control/rates_setpoint/pitch": "setpoint_pitch_rate",
    "/am/control/rates_setpoint/yaw": "setpoint_yaw_rate",
    "/am/policy/manual_hover_activation/manual_xy_hold": "manual_xy_hold",
    "/am/policy/manual_hover_activation/manual_z_hold": "manual_z_hold",
    "/am/policy/manual_hover_activation/manual_hover_active": "manual_hover_active",
}


def resolve_path(path: Path) -> Path:
    return path.expanduser().resolve()


def _safe_output_stem(log_path: Path) -> str:
    date_part = log_path.parent.name if log_path.parent.name else "ulog"
    return f"{date_part}_{log_path.stem}".replace(os.sep, "_")


def _default_rerun_path(log_path: Path, output_dir: Path) -> Path:
    return output_dir / f"{_safe_output_stem(log_path)}_all_data.rrd"


def _rerun_entity_path(topic_name: str, multi_id: int, field_name: str) -> str:
    return f"/ulog/raw/{topic_name}/instance_{multi_id}/{field_name}"


def _path_group(root: str, names: Sequence[str]) -> tuple[str, ...]:
    return tuple(f"{root}/{name}" for name in names)


def build_time_series_groups() -> tuple[TimeSeriesGroup, ...]:
    return (
        TimeSeriesGroup(
            name="Nav State",
            origin="/am/status/vehicle_status/nav_state",
            contents=_path_group(
                "/am/status/vehicle_status/nav_state",
                ("value", "is_posctl", "is_am_position", "is_am_offboard", "is_am_test"),
            ),
        ),
        TimeSeriesGroup(
            name="Motor Outputs 0-3",
            origin="/am/control/actuator_motors/motor",
            contents=_path_group("/am/control/actuator_motors/motor", tuple(str(idx) for idx in range(4))),
            axis_y=(-0.1, 1.1),
        ),
        TimeSeriesGroup(
            name="Policy Raw Action",
            origin="/am/policy/raw_action",
            contents=_path_group("/am/policy/raw_action", tuple(str(idx) for idx in range(4))),
            axis_y=(-1.1, 1.1),
        ),
        TimeSeriesGroup(
            name="Policy Mapped Action",
            origin="/am/policy/mapped_action",
            contents=_path_group("/am/policy/mapped_action", tuple(str(idx) for idx in range(4))),
            axis_y=(-0.1, 1.1),
        ),
        TimeSeriesGroup(
            name="Position NED",
            origin="/am/state/local_position/position_ned",
            contents=_path_group(
                "/am/state/local_position/position_ned",
                ("x", "y", "z"),
            ),
        ),
        TimeSeriesGroup(
            name="Linear Velocity NED",
            origin="/am/state/local_position/velocity_ned",
            contents=_path_group(
                "/am/state/local_position/velocity_ned",
                ("x", "y", "z"),
            ),
        ),
        TimeSeriesGroup(
            name="Acceleration Setpoint NED",
            origin="/am/setpoint/trajectory/acceleration_ned",
            contents=_path_group("/am/setpoint/trajectory/acceleration_ned", ("x", "y", "z")),
        ),
        TimeSeriesGroup(
            name="Attitude RPY",
            origin="/am/state/attitude",
            contents=_path_group("/am/state/attitude", ("roll", "pitch", "yaw")),
        ),
        TimeSeriesGroup(
            name="Angular Velocity FRD",
            origin="/am/state/angular_velocity/body_rates_frd",
            contents=_path_group("/am/state/angular_velocity/body_rates_frd", ("x", "y", "z")),
        ),
        TimeSeriesGroup(
            name="Angular Acceleration FRD",
            origin="/am/state/angular_velocity/body_rate_derivative_frd",
            contents=_path_group("/am/state/angular_velocity/body_rate_derivative_frd", ("x", "y", "z")),
        ),
        TimeSeriesGroup(
            name="Thrust Setpoint Body",
            origin="/am/control/thrust_setpoint/body",
            contents=_path_group("/am/control/thrust_setpoint/body", ("x", "y", "z")),
        ),
        TimeSeriesGroup(
            name="Rates Setpoint",
            origin="/am/control/rates_setpoint",
            contents=_path_group("/am/control/rates_setpoint", ("roll", "pitch", "yaw")),
        ),
        TimeSeriesGroup(
            name="Rates Thrust Body",
            origin="/am/control/rates_setpoint/thrust_body",
            contents=_path_group("/am/control/rates_setpoint/thrust_body", ("x", "y", "z")),
        ),
        TimeSeriesGroup(
            name="Policy Motor Control 0-3",
            origin="/am/policy/motor_control",
            contents=_path_group("/am/policy/motor_control", tuple(str(idx) for idx in range(4))),
            axis_y=(-0.1, 1.1),
        ),
        TimeSeriesGroup(
            name="Position Error Body",
            origin="/am/policy/observation/position_error_body",
            contents=_path_group("/am/policy/observation/position_error_body", ("x", "y", "z")),
        ),
        TimeSeriesGroup(
            name="Linear Velocity Error Body",
            origin="/am/policy/observation/linear_velocity_error_body",
            contents=_path_group("/am/policy/observation/linear_velocity_error_body", ("x", "y", "z")),
        ),
        TimeSeriesGroup(
            name="Angular Velocity Error Body",
            origin="/am/policy/observation/angular_velocity_error_body",
            contents=_path_group("/am/policy/observation/angular_velocity_error_body", ("x", "y", "z")),
        ),
        TimeSeriesGroup(
            name="Projected Gravity Body",
            origin="/am/policy/observation/projected_gravity_body",
            contents=_path_group("/am/policy/observation/projected_gravity_body", ("x", "y", "z")),
        ),
        TimeSeriesGroup(
            name="Arm Position",
            origin="/am/policy/observation/arm_position",
            contents=_path_group("/am/policy/observation/arm_position", tuple(str(idx) for idx in range(5))),
        ),
        TimeSeriesGroup(
            name="Arm Velocity",
            origin="/am/policy/observation/arm_velocity",
            contents=_path_group("/am/policy/observation/arm_velocity", tuple(str(idx) for idx in range(5))),
        ),
        TimeSeriesGroup(
            name="Previous Action",
            origin="/am/policy/observation/previous_action",
            contents=_path_group("/am/policy/observation/previous_action", tuple(str(idx) for idx in range(4))),
            axis_y=(-1.1, 1.1),
        ),
        TimeSeriesGroup(
            name="Manual Hover Activation",
            origin="/am/policy/manual_hover_activation",
            contents=_path_group(
                "/am/policy/manual_hover_activation",
                ("manual_xy_hold", "manual_z_hold", "manual_hover_active"),
            ),
            axis_y=(-0.1, 1.1),
        ),
        TimeSeriesGroup(
            name="AM Availability",
            origin="/am/status/am_pos_control",
            contents=_path_group(
                "/am/status/am_pos_control",
                (
                    "module_running",
                    "manual_control_available",
                    "arm_state_valid",
                    "trajectory_setpoint_valid",
                    "offboard_control_mode_fresh",
                    "offboard_control_mode_supported",
                    "am_position_available",
                    "am_offboard_available",
                ),
            ),
            axis_y=(-0.1, 1.1),
        ),
        TimeSeriesGroup(
            name="Offboard Control Mode",
            origin="/am/status/offboard_control_mode",
            contents=_path_group(
                "/am/status/offboard_control_mode",
                ("position", "velocity", "acceleration", "attitude", "body_rate", "actuator"),
            ),
            axis_y=(-0.1, 1.1),
        ),
        TimeSeriesGroup(
            name="Vehicle Flags",
            origin="/am/status/vehicle_status",
            contents=_path_group(
                "/am/status/vehicle_status",
                ("arming_state", "failsafe", "pre_flight_checks_pass"),
            ),
        ),
        TimeSeriesGroup(
            name="Policy Failure Flags",
            origin="/am/policy/failure_flags",
            contents=("/am/policy/failure_flags",)
            + _path_group(
                "/am/policy/failure_flags",
                (
                    "vehicle_state_invalid",
                    "arm_state_invalid",
                    "am_setpoint_invalid",
                    "am_inference_failed",
                ),
            ),
        ),
        TimeSeriesGroup(
            name="Policy Degraded Flags",
            origin="/am/policy/degraded_flags",
            contents=("/am/policy/degraded_flags",)
            + _path_group(
                "/am/policy/degraded_flags",
                ("local_xy_invalid", "local_vxy_invalid", "setpoint_defaulted"),
            ),
        ),
    )


def build_am_blueprint():
    import rerun.blueprint as rrb
    import rerun.blueprint.archetypes as rrb_archetypes

    groups = {group.name: group for group in build_time_series_groups()}
    plot_background = rrb_archetypes.PlotBackground(color=[248, 250, 252, 255], show_grid=True)

    def series_view(name: str, view_name: str | None = None):
        group = groups[name]
        axis_y = rrb.ScalarAxis(range=group.axis_y) if group.axis_y is not None else None
        return rrb.TimeSeriesView(
            origin=group.origin,
            contents=list(group.contents),
            name=view_name or group.name,
            axis_y=axis_y,
            plot_legend=rrb.PlotLegend("RightTop", visible=True),
            background=plot_background,
        )

    overview = rrb.Horizontal(
        rrb.Spatial3DView(
            origin="/am/scene",
            contents=[
                "/am/scene/world",
                "/am/scene/vehicle/**",
            ],
            name="Flight Scene",
            background=rrb.Background([248, 250, 252, 255]),
            line_grid=rrb.LineGrid3D(visible=True, spacing=1.0, stroke_width=1.0, color=[203, 213, 225, 255]),
        ),
        rrb.Vertical(
            series_view("Nav State"),
            series_view("Motor Outputs 0-3"),
            series_view("Policy Mapped Action"),
            series_view("Linear Velocity NED"),
            series_view("Angular Velocity FRD"),
            name="Overview Signals",
        ),
        name="Overview",
        column_shares=[2.0, 1.3],
    )

    state = rrb.Grid(
        series_view("Position NED"),
        series_view("Linear Velocity NED"),
        series_view("Attitude RPY"),
        series_view("Angular Velocity FRD"),
        series_view("Angular Acceleration FRD"),
        name="State",
        grid_columns=2,
    )

    control = rrb.Grid(
        series_view("Motor Outputs 0-3", "Motor Output"),
        series_view("Policy Raw Action"),
        series_view("Policy Mapped Action"),
        series_view("Policy Motor Control 0-3"),
        series_view("Angular Velocity FRD", "Rates Actual"),
        series_view("Rates Setpoint"),
        series_view("Thrust Setpoint Body"),
        series_view("Rates Thrust Body"),
        name="Control",
        grid_columns=2,
    )

    policy = rrb.Grid(
        series_view("Position Error Body"),
        series_view("Linear Velocity Error Body"),
        series_view("Angular Velocity Error Body"),
        series_view("Projected Gravity Body"),
        series_view("Arm Position"),
        series_view("Arm Velocity"),
        series_view("Previous Action"),
        series_view("Manual Hover Activation"),
        name="AM Policy",
        grid_columns=2,
    )

    status = rrb.Grid(
        series_view("Nav State"),
        series_view("AM Availability"),
        series_view("Offboard Control Mode"),
        series_view("Vehicle Flags"),
        series_view("Policy Failure Flags"),
        series_view("Policy Degraded Flags"),
        name="Status",
        grid_columns=2,
    )

    return rrb.Blueprint(
        rrb.Tabs(overview, state, control, policy, status, active_tab=0, name="PX4 AM ULog"),
        rrb.SelectionPanel(state="collapsed"),
        rrb.BlueprintPanel(state="collapsed"),
        rrb.TimePanel(expanded=True, timeline="time_s"),
        collapse_panels=False,
    )


def _optional_topic(ulog: ULog, name: str):
    for dataset in ulog.data_list:
        if dataset.name == name:
            return dataset

    return None


def _stack_dataset_fields(dataset, fields: Sequence[str]) -> np.ndarray:
    return np.column_stack([np.asarray(dataset.data[field], dtype=np.float64) for field in fields])


def _am_policy_observation_names() -> list[str]:
    names: list[str] = []
    names.extend(f"position_error_body/{axis}" for axis in ("x", "y", "z"))
    names.extend(f"attitude_error_dcm/r{row}c{col}" for row in range(3) for col in range(3))
    names.extend(f"projected_gravity_body/{axis}" for axis in ("x", "y", "z"))
    names.extend(f"linear_velocity_error_body/{axis}" for axis in ("x", "y", "z"))
    names.extend(f"angular_velocity_error_body/{axis}" for axis in ("x", "y", "z"))
    names.extend(f"arm_position/{idx}" for idx in range(5))
    names.extend(f"arm_velocity/{idx}" for idx in range(5))
    names.extend(f"previous_action/{idx}" for idx in range(4))
    return names


def _flag_names(value: int, mapping: dict[int, str]) -> str:
    if value == 0:
        return "NONE"

    names = [name for bit, name in mapping.items() if value & bit]
    unknown = value & ~sum(mapping.keys())
    if unknown:
        names.append(f"UNKNOWN_0x{unknown:x}")
    return "|".join(names)


def export_rerun_visualization(
    log_path: Path,
    output_dir: Path,
    rrd_path: Path | None = None,
    open_viewer: bool = True,
) -> RerunExportResult:
    try:
        import rerun as rr
    except ImportError as exc:
        raise RerunUnavailableError() from exc

    log_path = log_path.expanduser().resolve()
    output_dir = output_dir.expanduser().resolve()

    if not log_path.exists():
        raise FileNotFoundError(f"Log file does not exist: {log_path}")

    output_dir.mkdir(parents=True, exist_ok=True)
    rrd_path = _default_rerun_path(log_path, output_dir) if rrd_path is None else rrd_path.expanduser().resolve()
    rrd_path.parent.mkdir(parents=True, exist_ok=True)

    ulog = ULog(str(log_path))
    recording = rr.RecordingStream("px4_am_ulog")
    _save_recording_with_blueprint(recording, rrd_path, build_am_blueprint())

    recording.log("/am/scene/world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    field_count, sample_count = _export_raw_fields(rr, recording, ulog)
    am_view_count = _export_am_views(rr, recording, ulog)
    opened_viewer = _open_rerun_viewer(rrd_path) if open_viewer else False

    return RerunExportResult(
        rrd_path=rrd_path,
        topic_count=len(ulog.data_list),
        field_count=field_count,
        sample_count=sample_count,
        am_view_count=am_view_count,
        opened_viewer=opened_viewer,
    )


def _save_recording_with_blueprint(recording, rrd_path: Path, blueprint) -> None:
    recording.save(str(rrd_path), default_blueprint=blueprint)
    recording.send_blueprint(blueprint, make_active=True, make_default=True)


def _export_raw_fields(rr, recording, ulog: ULog) -> tuple[int, int]:
    field_count = 0
    sample_count = 0

    for dataset in ulog.data_list:
        multi_id = int(getattr(dataset, "multi_id", 0))
        if "timestamp" not in dataset.data:
            continue

        times_s = relative_seconds(dataset.data["timestamp"], ulog.start_timestamp)

        for field_name, values in dataset.data.items():
            values_array = np.asarray(values)
            if values_array.ndim != 1 or len(values_array) != len(times_s):
                continue

            _send_rerun_scalar_columns(
                rr=rr,
                recording=recording,
                entity_path=_rerun_entity_path(dataset.name, multi_id, field_name),
                times_s=times_s,
                values=values_array,
            )
            field_count += 1
            sample_count += len(values_array)

    return field_count, sample_count


def _export_am_views(rr, recording, ulog: ULog) -> int:
    count = 0
    exporters = (
        _export_am_scene,
        _export_am_status,
        _export_am_state,
        _export_am_setpoint,
        _export_am_control,
        _export_am_policy,
        _export_am_test,
    )

    for exporter in exporters:
        count += exporter(rr, recording, ulog)

    return count


def _series_color(name: str) -> list[int]:
    if name in FLAG_COLORS:
        return FLAG_COLORS[name]

    leaf = name.rsplit("/", 1)[-1]
    if leaf in AXIS_COLORS:
        return AXIS_COLORS[leaf]
    if leaf.isdigit():
        return CHANNEL_COLORS[int(leaf) % len(CHANNEL_COLORS)]

    if name.startswith("/am/status/"):
        return FLAG_PALETTE[sum(ord(char) for char in name) % len(FLAG_PALETTE)]
    if name.startswith("/am/policy/failure_flags") or name.startswith("/am/policy/degraded_flags"):
        return FLAG_PALETTE[sum(ord(char) for char in name) % len(FLAG_PALETTE)]

    return [180, 180, 180, 255]


def _series_width(name: str) -> float:
    if name in FLAG_COLORS or name.startswith("/am/status/"):
        return FLAG_SERIES_WIDTH
    return DEFAULT_SERIES_WIDTH


def _series_name(name: str) -> str:
    if name in SERIES_LABELS:
        return SERIES_LABELS[name]

    parts = [part for part in name.split("/") if part]
    normalized_name = "/" + "/".join(parts)
    if normalized_name in SERIES_LABELS:
        return SERIES_LABELS[normalized_name]

    if len(parts) >= 2 and parts[-1] in {"x", "y", "z"}:
        return f"{parts[-2]}_{parts[-1]}"
    if len(parts) >= 2 and parts[-1].isdigit():
        return f"{parts[-2]}_{parts[-1]}"
    return parts[-1] if parts else name


def _ulog_parameter(ulog: ULog, name: str, default: float) -> float:
    value = getattr(ulog, "initial_parameters", {}).get(name, default)
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _manual_hover_activation_fields(ulog: ULog) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    manual = _optional_topic(ulog, "manual_control_setpoint")
    local_position = _optional_topic(ulog, "vehicle_local_position")
    required_manual = ("timestamp", "valid", "pitch", "roll", "throttle")
    required_position = ("timestamp", "vx", "vy", "vz")

    if manual is None or local_position is None:
        return np.array([], dtype=np.float64), {}
    if not all(field in manual.data for field in required_manual):
        return np.array([], dtype=np.float64), {}
    if not all(field in local_position.data for field in required_position):
        return np.array([], dtype=np.float64), {}

    times_s = relative_seconds(manual.data["timestamp"], ulog.start_timestamp)
    pos_times_s = relative_seconds(local_position.data["timestamp"], ulog.start_timestamp)
    velocities = _stack_dataset_fields(local_position, ("vx", "vy", "vz"))
    velocities_at_manual = interpolate_columns(pos_times_s, velocities, times_s)

    valid = np.asarray(manual.data["valid"]).astype(bool)
    pitch = np.asarray(manual.data["pitch"], dtype=np.float64)
    roll = np.asarray(manual.data["roll"], dtype=np.float64)
    throttle = np.asarray(manual.data["throttle"], dtype=np.float64)
    vx = velocities_at_manual[:, 0]
    vy = velocities_at_manual[:, 1]
    vz = velocities_at_manual[:, 2]

    deadzone = _ulog_parameter(ulog, "MAN_DEADZONE", DEFAULT_MAN_DEADZONE)
    hold_max_xy = _ulog_parameter(ulog, "MPC_HOLD_MAX_XY", DEFAULT_MPC_HOLD_MAX_XY)
    hold_max_z = _ulog_parameter(ulog, "MPC_HOLD_MAX_Z", DEFAULT_MPC_HOLD_MAX_Z)

    finite_xy = np.isfinite(pitch) & np.isfinite(roll) & np.isfinite(vx) & np.isfinite(vy)
    finite_z = np.isfinite(throttle) & np.isfinite(vz)
    centered_xy = (np.abs(pitch) <= deadzone) & (np.abs(roll) <= deadzone)
    centered_z = np.abs(throttle) <= deadzone
    stopped_xy = (hold_max_xy <= 0.0) | (np.sqrt(vx * vx + vy * vy) < hold_max_xy)
    stopped_z = (hold_max_z <= 0.0) | (np.abs(vz) < hold_max_z)

    manual_xy_hold = (valid & finite_xy & centered_xy & stopped_xy).astype(np.uint8)
    manual_z_hold = (valid & finite_z & centered_z & stopped_z).astype(np.uint8)
    manual_hover_active = ((manual_xy_hold != 0) & (manual_z_hold != 0)).astype(np.uint8)

    return times_s, {
        "manual_xy_hold": manual_xy_hold,
        "manual_z_hold": manual_z_hold,
        "manual_hover_active": manual_hover_active,
    }


def _send_rerun_scalar_columns(
    *,
    rr,
    recording,
    entity_path: str,
    times_s: np.ndarray,
    values: np.ndarray,
    series_name: str | None = None,
    color: list[int] | None = None,
) -> None:
    values = np.asarray(values)
    times_s = np.asarray(times_s, dtype=np.float64)
    if len(times_s) == 0 or len(values) == 0:
        return

    if series_name is not None or color is not None:
        recording.log(
            entity_path,
            rr.SeriesLines(
                names=series_name or _series_name(entity_path),
                colors=color or _series_color(entity_path),
                widths=_series_width(entity_path),
            ),
            static=True,
        )

    scalars = rr.Scalars.columns(scalars=values)
    time_column = rr.TimeColumn("time_s", duration=times_s)
    recording.send_columns(entity_path, indexes=[time_column], columns=scalars)


def _send_grouped_scalars(
    *,
    rr,
    recording,
    root: str,
    times_s: np.ndarray,
    fields: dict[str, np.ndarray],
) -> int:
    count = 0
    for name, values in fields.items():
        values = np.asarray(values)
        if values.ndim != 1 or len(values) != len(times_s):
            continue
        entity_path = f"{root}/{name}"
        _send_rerun_scalar_columns(
            rr=rr,
            recording=recording,
            entity_path=entity_path,
            times_s=times_s,
            values=values,
            series_name=_series_name(entity_path),
            color=_series_color(entity_path),
        )
        count += 1
    return count


def _send_rerun_transform_columns(
    *,
    rr,
    recording,
    entity_path: str,
    times_s: np.ndarray,
    translations: np.ndarray,
    quats_wxyz: np.ndarray,
    child_frame: str | None = None,
    parent_frame: str | None = None,
) -> None:
    mask = finite_rows(times_s, translations, quats_wxyz)
    if not np.any(mask):
        return

    row_count = int(np.count_nonzero(mask))
    columns = rr.Transform3D.columns(
        translation=np.asarray(translations, dtype=np.float64)[mask],
        quaternion=quat_wxyz_to_xyzw(quats_wxyz[mask]),
        child_frame=np.full(row_count, child_frame) if child_frame is not None else None,
        parent_frame=np.full(row_count, parent_frame) if parent_frame is not None else None,
    )
    time_column = rr.TimeColumn("time_s", duration=np.asarray(times_s, dtype=np.float64)[mask])
    recording.send_columns(entity_path, indexes=[time_column], columns=columns)


def _send_rerun_arrows_columns(
    *,
    rr,
    recording,
    entity_path: str,
    times_s: np.ndarray,
    origins: np.ndarray,
    vectors: np.ndarray,
    color: list[int],
    radius: float,
    label: str | None = None,
) -> None:
    mask = finite_rows(times_s, origins, vectors)
    if not np.any(mask):
        return

    row_count = int(np.count_nonzero(mask))
    colors = np.tile(np.asarray(color, dtype=np.uint8), (row_count, 1))
    columns = rr.Arrows3D.columns(
        origins=np.asarray(origins, dtype=np.float64)[mask],
        vectors=np.asarray(vectors, dtype=np.float64)[mask],
        colors=colors,
        radii=np.full(row_count, radius),
        labels=np.full(row_count, label) if label is not None else None,
        show_labels=np.full(row_count, True) if label is not None else None,
    )
    time_column = rr.TimeColumn("time_s", duration=np.asarray(times_s, dtype=np.float64)[mask])
    recording.send_columns(entity_path, indexes=[time_column], columns=columns)


def _send_vehicle_body_axis_columns(
    *,
    rr,
    recording,
    times_s: np.ndarray,
    origins: np.ndarray,
    quats_wxyz: np.ndarray,
) -> int:
    count = 0
    for axis, unit_vector in (
        ("x", np.array([1.0, 0.0, 0.0], dtype=np.float64)),
        ("y", np.array([0.0, 1.0, 0.0], dtype=np.float64)),
        ("z", np.array([0.0, 0.0, 1.0], dtype=np.float64)),
    ):
        vectors = rotate_vectors_by_quat_wxyz(quats_wxyz, unit_vector) * VEHICLE_BODY_AXIS_LENGTH
        _send_rerun_arrows_columns(
            rr=rr,
            recording=recording,
            entity_path=SCENE_ENTITIES[f"vehicle_body_axis_{axis}"].path,
            times_s=times_s,
            origins=origins,
            vectors=vectors,
            color=AXIS_COLORS[axis],
            radius=VEHICLE_BODY_AXIS_RADIUS,
            label=axis.upper(),
        )
        count += 1

    return count


def _export_line_strip(
    rr,
    recording,
    entity_path: str,
    positions: np.ndarray,
    color: list[int],
    radius: float,
    label: str,
    show_labels: bool = False,
) -> bool:
    mask = finite_rows(positions)
    if not np.any(mask):
        return False

    recording.log(
        entity_path,
        rr.LineStrips3D(
            [np.asarray(positions, dtype=np.float64)[mask]],
            colors=[color],
            radii=[radius],
            labels=[label],
            show_labels=show_labels,
        ),
        static=True,
    )
    return True


def _export_am_scene(rr, recording, ulog: ULog) -> int:
    count = 0
    local_position = _optional_topic(ulog, "vehicle_local_position")
    vehicle_attitude = _optional_topic(ulog, "vehicle_attitude")

    if local_position is not None and all(field in local_position.data for field in ("timestamp", "x", "y", "z")):
        pos_times = relative_seconds(local_position.data["timestamp"], ulog.start_timestamp)
        positions = ned_to_enu_positions(local_position.data["x"], local_position.data["y"], local_position.data["z"])
        if _export_line_strip(
            rr,
            recording,
            SCENE_ENTITIES["vehicle_path"].path,
            positions,
            [20, 140, 255, 255],
            0.008,
            SCENE_ENTITIES["vehicle_path"].label,
        ):
            count += 1

        if vehicle_attitude is not None and all(field in vehicle_attitude.data for field in ("timestamp", "q[0]", "q[1]", "q[2]", "q[3]")):
            att_times = relative_seconds(vehicle_attitude.data["timestamp"], ulog.start_timestamp)
            att_q = _stack_dataset_fields(vehicle_attitude, ("q[0]", "q[1]", "q[2]", "q[3]"))
            scene_times = att_times
            scene_positions = interpolate_columns(pos_times, positions, scene_times)
            scene_quats = attitude_frd_to_ned_to_flu_to_enu_quat(att_q)
            _send_rerun_transform_columns(
                rr=rr,
                recording=recording,
                entity_path=SCENE_ENTITIES["vehicle_frame"].path,
                times_s=scene_times,
                translations=scene_positions,
                quats_wxyz=scene_quats,
                child_frame=SCENE_ENTITIES["vehicle_frame"].child_frame,
                parent_frame=SCENE_ENTITIES["vehicle_frame"].parent_frame,
            )
            count += 1
        else:
            scene_times = pos_times
            scene_positions = positions
            scene_quats = np.tile(np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64), (len(scene_times), 1))
            _send_rerun_transform_columns(
                rr=rr,
                recording=recording,
                entity_path=SCENE_ENTITIES["vehicle_frame"].path,
                times_s=scene_times,
                translations=scene_positions,
                quats_wxyz=scene_quats,
                child_frame=SCENE_ENTITIES["vehicle_frame"].child_frame,
                parent_frame=SCENE_ENTITIES["vehicle_frame"].parent_frame,
            )
            count += 1

        count += _send_vehicle_body_axis_columns(
            rr=rr,
            recording=recording,
            times_s=scene_times,
            origins=scene_positions,
            quats_wxyz=scene_quats,
        )

    return count


def _export_am_state(rr, recording, ulog: ULog) -> int:
    count = 0
    local_position = _optional_topic(ulog, "vehicle_local_position")
    if local_position is not None:
        times_s = relative_seconds(local_position.data["timestamp"], ulog.start_timestamp)
        fields = {}
        for source, target in (
            ("x", "position_ned/x"),
            ("y", "position_ned/y"),
            ("z", "position_ned/z"),
            ("vx", "velocity_ned/x"),
            ("vy", "velocity_ned/y"),
            ("vz", "velocity_ned/z"),
            ("heading", "heading_ned"),
            ("eph", "accuracy/eph"),
            ("epv", "accuracy/epv"),
            ("xy_valid", "validity/xy_valid"),
            ("z_valid", "validity/z_valid"),
            ("v_xy_valid", "validity/v_xy_valid"),
            ("v_z_valid", "validity/v_z_valid"),
        ):
            if source in local_position.data:
                fields[target] = local_position.data[source]
        count += _send_grouped_scalars(rr=rr, recording=recording, root="/am/state/local_position", times_s=times_s, fields=fields)

    vehicle_attitude = _optional_topic(ulog, "vehicle_attitude")
    if vehicle_attitude is not None and all(field in vehicle_attitude.data for field in ("q[0]", "q[1]", "q[2]", "q[3]")):
        times_s = relative_seconds(vehicle_attitude.data["timestamp"], ulog.start_timestamp)
        q = _stack_dataset_fields(vehicle_attitude, ("q[0]", "q[1]", "q[2]", "q[3]"))
        euler = quat_wxyz_to_euler(q)
        count += _send_grouped_scalars(
            rr=rr,
            recording=recording,
            root="/am/state/attitude",
            times_s=times_s,
            fields={
                "roll": euler[:, 0],
                "pitch": euler[:, 1],
                "yaw": euler[:, 2],
                "q/w": q[:, 0],
                "q/x": q[:, 1],
                "q/y": q[:, 2],
                "q/z": q[:, 3],
            },
        )

    angular_velocity = _optional_topic(ulog, "vehicle_angular_velocity")
    if angular_velocity is not None:
        times_s = relative_seconds(angular_velocity.data["timestamp"], ulog.start_timestamp)
        fields = {
            target: angular_velocity.data[source]
            for source, target in (
                ("xyz[0]", "body_rates_frd/x"),
                ("xyz[1]", "body_rates_frd/y"),
                ("xyz[2]", "body_rates_frd/z"),
                ("xyz_derivative[0]", "body_rate_derivative_frd/x"),
                ("xyz_derivative[1]", "body_rate_derivative_frd/y"),
                ("xyz_derivative[2]", "body_rate_derivative_frd/z"),
            )
            if source in angular_velocity.data
        }
        count += _send_grouped_scalars(rr=rr, recording=recording, root="/am/state/angular_velocity", times_s=times_s, fields=fields)

    return count


def _export_am_setpoint(rr, recording, ulog: ULog) -> int:
    count = 0
    trajectory_setpoint = _optional_topic(ulog, "trajectory_setpoint")
    if trajectory_setpoint is not None:
        times_s = relative_seconds(trajectory_setpoint.data["timestamp"], ulog.start_timestamp)
        fields = {}
        for prefix in ("position", "velocity", "acceleration", "jerk"):
            for idx, axis in enumerate(("x", "y", "z")):
                source = f"{prefix}[{idx}]"
                if source in trajectory_setpoint.data:
                    fields[f"{prefix}_ned/{axis}"] = trajectory_setpoint.data[source]
        for source in ("yaw", "yawspeed"):
            if source in trajectory_setpoint.data:
                fields[source] = trajectory_setpoint.data[source]

        count += _send_grouped_scalars(rr=rr, recording=recording, root="/am/setpoint/trajectory", times_s=times_s, fields=fields)

    attitude_setpoint = _optional_topic(ulog, "vehicle_attitude_setpoint")
    if attitude_setpoint is not None and all(field in attitude_setpoint.data for field in ("q_d[0]", "q_d[1]", "q_d[2]", "q_d[3]")):
        times_s = relative_seconds(attitude_setpoint.data["timestamp"], ulog.start_timestamp)
        q = _stack_dataset_fields(attitude_setpoint, ("q_d[0]", "q_d[1]", "q_d[2]", "q_d[3]"))
        euler = quat_wxyz_to_euler(q)
        count += _send_grouped_scalars(
            rr=rr,
            recording=recording,
            root="/am/setpoint/attitude",
            times_s=times_s,
            fields={
                "roll": euler[:, 0],
                "pitch": euler[:, 1],
                "yaw": euler[:, 2],
            },
        )

    return count


def _export_am_control(rr, recording, ulog: ULog) -> int:
    count = 0
    rates = _optional_topic(ulog, "vehicle_rates_setpoint")
    if rates is not None:
        times_s = relative_seconds(rates.data["timestamp"], ulog.start_timestamp)
        fields = {name: rates.data[name] for name in ("roll", "pitch", "yaw", "reset_integral") if name in rates.data}
        for idx, axis in enumerate(("x", "y", "z")):
            source = f"thrust_body[{idx}]"
            if source in rates.data:
                fields[f"thrust_body/{axis}"] = rates.data[source]
        count += _send_grouped_scalars(rr=rr, recording=recording, root="/am/control/rates_setpoint", times_s=times_s, fields=fields)

    thrust = _optional_topic(ulog, "vehicle_thrust_setpoint")
    if thrust is not None:
        times_s = relative_seconds(thrust.data["timestamp"], ulog.start_timestamp)
        fields = {f"body/{axis}": thrust.data[f"xyz[{idx}]"] for idx, axis in enumerate(("x", "y", "z")) if f"xyz[{idx}]" in thrust.data}
        count += _send_grouped_scalars(rr=rr, recording=recording, root="/am/control/thrust_setpoint", times_s=times_s, fields=fields)

    motors = _optional_topic(ulog, "actuator_motors")
    if motors is not None:
        times_s = relative_seconds(motors.data["timestamp"], ulog.start_timestamp)
        fields = {f"motor/{idx}": motors.data[f"control[{idx}]"] for idx in range(12) if f"control[{idx}]" in motors.data}
        if "reversible_flags" in motors.data:
            fields["reversible_flags"] = motors.data["reversible_flags"]
        count += _send_grouped_scalars(rr=rr, recording=recording, root="/am/control/actuator_motors", times_s=times_s, fields=fields)

    manual = _optional_topic(ulog, "manual_control_setpoint")
    if manual is not None:
        times_s = relative_seconds(manual.data["timestamp"], ulog.start_timestamp)
        fields = {name: manual.data[name] for name in ("roll", "pitch", "yaw", "throttle", "valid", "sticks_moving") if name in manual.data}
        count += _send_grouped_scalars(rr=rr, recording=recording, root="/am/manual", times_s=times_s, fields=fields)

    return count


def _export_am_status(rr, recording, ulog: ULog) -> int:
    count = 0
    vehicle_status = _optional_topic(ulog, "vehicle_status")
    if vehicle_status is not None:
        times_s = relative_seconds(vehicle_status.data["timestamp"], ulog.start_timestamp)
        fields = {}
        if "nav_state" in vehicle_status.data:
            nav_state = np.asarray(vehicle_status.data["nav_state"])
            fields["nav_state/value"] = nav_state
            for value, name in AM_EXPECTED_NAV_STATES.items():
                fields[f"nav_state/is_{name.lower()}"] = (nav_state == value).astype(np.uint8)
        for source in ("arming_state", "failsafe", "pre_flight_checks_pass"):
            if source in vehicle_status.data:
                fields[source] = vehicle_status.data[source]
        count += _send_grouped_scalars(rr=rr, recording=recording, root="/am/status/vehicle_status", times_s=times_s, fields=fields)

    for topic_name, root in (
        ("vehicle_control_mode", "/am/status/vehicle_control_mode"),
        ("vehicle_land_detected", "/am/status/land_detected"),
        ("takeoff_status", "/am/status/takeoff"),
        ("am_pos_control_status", "/am/status/am_pos_control"),
        ("offboard_control_mode", "/am/status/offboard_control_mode"),
    ):
        dataset = _optional_topic(ulog, topic_name)
        if dataset is None:
            continue
        times_s = relative_seconds(dataset.data["timestamp"], ulog.start_timestamp)
        fields = {name: values for name, values in dataset.data.items() if name != "timestamp" and np.asarray(values).ndim == 1}
        count += _send_grouped_scalars(rr=rr, recording=recording, root=root, times_s=times_s, fields=fields)

    return count


def _export_am_policy(rr, recording, ulog: ULog) -> int:
    count = 0
    policy = _optional_topic(ulog, "am_policy_observation")
    if policy is not None:
        times_s = relative_seconds(policy.data["timestamp"], ulog.start_timestamp)
        fields = {}

        for idx, name in enumerate(_am_policy_observation_names()):
            source = f"observation[{idx}]"
            if source in policy.data:
                fields[f"observation/{name}"] = policy.data[source]

        for prefix, field_count in (("raw_action", 4), ("mapped_action", 4), ("motor_control", 12)):
            for idx in range(field_count):
                source = f"{prefix}[{idx}]"
                if source in policy.data:
                    fields[f"{prefix}/{idx}"] = policy.data[source]

        for source, mapping in (("failure_flags", AM_FAILURE_FLAGS), ("degraded_flags", AM_DEGRADED_FLAGS)):
            if source in policy.data:
                values = np.asarray(policy.data[source])
                fields[source] = values
                for bit, name in mapping.items():
                    fields[f"{source}/{name.lower()}"] = ((values.astype(np.uint32) & bit) != 0).astype(np.uint8)

        count += _send_grouped_scalars(rr=rr, recording=recording, root="/am/policy", times_s=times_s, fields=fields)

    hover_times_s, hover_fields = _manual_hover_activation_fields(ulog)
    count += _send_grouped_scalars(
        rr=rr,
        recording=recording,
        root="/am/policy/manual_hover_activation",
        times_s=hover_times_s,
        fields=hover_fields,
    )
    return count


def _export_am_test(rr, recording, ulog: ULog) -> int:
    count = 0
    for topic_name, root in (("am_test_status", "/am/test/status"), ("am_test_result", "/am/test/result")):
        dataset = _optional_topic(ulog, topic_name)
        if dataset is None:
            continue
        times_s = relative_seconds(dataset.data["timestamp"], ulog.start_timestamp)
        fields = {}
        for name, values in dataset.data.items():
            if name not in ("timestamp", "timestamp_sample") and np.asarray(values).ndim == 1:
                fields[name.replace("[", "/").replace("]", "")] = values
        for source, mapping in (("failure_flags", AM_FAILURE_FLAGS), ("degraded_flags", AM_DEGRADED_FLAGS)):
            if source in dataset.data:
                values = np.asarray(dataset.data[source])
                for bit, name in mapping.items():
                    fields[f"{source}/{name.lower()}"] = ((values.astype(np.uint32) & bit) != 0).astype(np.uint8)
        count += _send_grouped_scalars(rr=rr, recording=recording, root=root, times_s=times_s, fields=fields)
    return count


def _open_rerun_viewer(rrd_path: Path) -> bool:
    rerun_cli = shutil.which("rerun")
    if rerun_cli is None:
        return False

    subprocess.Popen(
        [rerun_cli, str(rrd_path)],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )
    return True


def select_log_file(initial_dir: Path) -> Path | None:
    try:
        import tkinter as tk
        from tkinter import filedialog
    except ImportError as exc:
        raise RuntimeError("tkinter is required for interactive log selection.") from exc

    root = tk.Tk()
    root.withdraw()
    root.update()
    try:
        filename = filedialog.askopenfilename(
            title="Select PX4 ULog",
            initialdir=str(initial_dir.expanduser()),
            filetypes=(("PX4 ULog files", "*.ulg"), ("All files", "*")),
        )
    finally:
        root.destroy()

    return Path(filename).expanduser().resolve() if filename else None


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Export PX4 AM ULog data to a Rerun .rrd visualization.")
    parser.add_argument("log", nargs="?", type=Path, help="Export this .ulg directly. Omit to open a file picker.")
    parser.add_argument("--output-dir", type=Path, default=Path.cwd() / DEFAULT_OUTPUT_DIR, help="Output directory. Default: ./analysis_outputs")
    parser.add_argument("--rerun-output", type=Path, help="Path for the Rerun .rrd file. Default: <output-dir>/<log-stem>_all_data.rrd")
    parser.add_argument("--rerun", action="store_true", help="Deprecated no-op: Rerun export is now the only output mode.")
    parser.add_argument("--rerun-only", action="store_true", help="Deprecated no-op: Rerun export is now the only output mode.")

    rerun_open_group = parser.add_mutually_exclusive_group()
    rerun_open_group.add_argument("--rerun-open", dest="rerun_open", action="store_true", default=True, help="Open the generated .rrd in Rerun Viewer after export. Default.")
    rerun_open_group.add_argument("--no-rerun-open", dest="rerun_open", action="store_false", help="Do not open Rerun Viewer after export.")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    output_dir = resolve_path(args.output_dir)
    rerun_output = resolve_path(args.rerun_output) if args.rerun_output is not None else None

    if args.log is not None:
        log_path = resolve_path(args.log)
    else:
        log_path = select_log_file(Path.home())
        if log_path is None:
            print("No log selected.")
            return 0

    result = export_rerun_visualization(
        log_path,
        output_dir,
        rrd_path=rerun_output,
        open_viewer=args.rerun_open,
    )
    _print_rerun_result(result)
    return 0


def _print_rerun_result(result: RerunExportResult) -> None:
    print(f"Rerun: {result.rrd_path}")
    print(
        "Raw export: "
        f"{result.topic_count} topics, "
        f"{result.field_count} fields, "
        f"{result.sample_count} samples"
    )
    print(f"AM views: {result.am_view_count} structured views")
    if result.opened_viewer:
        print("Rerun Viewer: opened")
    else:
        print("Rerun Viewer: not opened; run `rerun <path>` manually if desired.")


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except RerunUnavailableError as exc:
        print(str(exc), file=sys.stderr)
        raise SystemExit(1) from None
