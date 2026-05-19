import importlib.util
import pathlib
import sys
import tempfile
import unittest
import contextlib
import io
import types
from unittest import mock

import numpy as np


REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
MODULE_PATH = REPO_ROOT / "tools" / "ulog" / "analysis_ulog.py"


def load_module():
    spec = importlib.util.spec_from_file_location("analysis_ulog", MODULE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load module from {MODULE_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def blueprint_names(part):
    names = []
    name = getattr(part, "name", None)
    if name:
        names.append(name)
    for child in getattr(part, "contents", ()):
        names.extend(blueprint_names(child))
    return names


def find_blueprint_part(part, name):
    if getattr(part, "name", None) == name:
        return part
    for child in getattr(part, "contents", ()):
        found = find_blueprint_part(child, name)
        if found is not None:
            return found
    return None


def blueprint_contents(part):
    contents = []
    for child in getattr(part, "contents", ()):
        if isinstance(child, str):
            contents.append(child)
        else:
            contents.extend(blueprint_contents(child))
    return contents


def group_by_name(groups, name):
    for group in groups:
        if group.name == name:
            return group
    raise AssertionError(f"Missing group: {name}")


class RecordingSpy:
    def __init__(self):
        self.logged = []
        self.sent = []
        self.saved = []
        self.blueprints = []

    def log(self, entity_path, *args, **kwargs):
        self.logged.append((entity_path, args, kwargs))

    def send_columns(self, entity_path, *args, **kwargs):
        self.sent.append((entity_path, args, kwargs))

    def save(self, *args, **kwargs):
        self.saved.append((args, kwargs))

    def send_blueprint(self, *args, **kwargs):
        self.blueprints.append((args, kwargs))


class AnalysisUlogTests(unittest.TestCase):
    def test_no_log_argument_opens_file_picker_and_cancel_exits_cleanly(self):
        module = load_module()

        with mock.patch.object(module, "select_log_file", return_value=None) as select_log_file:
            with contextlib.redirect_stdout(io.StringIO()):
                exit_code = module.main([])

        self.assertEqual(exit_code, 0)
        select_log_file.assert_called_once()

    def test_positional_log_argument_bypasses_file_picker(self):
        module = load_module()

        with tempfile.TemporaryDirectory() as tmpdir:
            fake_log = pathlib.Path(tmpdir) / "sample.ulg"
            fake_log.write_bytes(b"not a real ulog")
            expected = module.RerunExportResult(fake_log.with_suffix(".rrd"), 1, 2, 3, 4, False)

            with mock.patch.object(module, "select_log_file") as select_log_file:
                with mock.patch.object(module, "export_rerun_visualization", return_value=expected) as export:
                    with contextlib.redirect_stdout(io.StringIO()):
                        exit_code = module.main([str(fake_log), "--no-rerun-open"])

        self.assertEqual(exit_code, 0)
        select_log_file.assert_not_called()
        export.assert_called_once()
        self.assertEqual(export.call_args.args[0], fake_log.resolve())

    def test_blueprint_contains_main_viewport_tabs(self):
        module = load_module()

        blueprint = module.build_am_blueprint()
        names = blueprint_names(blueprint.root_container)

        for expected_name in ("Overview", "State", "Control", "AM Policy", "Status"):
            self.assertIn(expected_name, names)
        self.assertNotIn("State & Setpoint", names)

        overview = find_blueprint_part(blueprint.root_container, "Overview")
        self.assertIsNotNone(overview)
        self.assertIn("Linear Velocity NED", blueprint_names(overview))
        self.assertNotIn("/am/scene/setpoint/**", blueprint_contents(overview))

    def test_state_blueprint_is_actual_state_only(self):
        module = load_module()

        blueprint = module.build_am_blueprint()
        state = find_blueprint_part(blueprint.root_container, "State")
        self.assertIsNotNone(state)
        names = blueprint_names(state)
        contents = blueprint_contents(state)

        for expected_name in (
            "Position NED",
            "Linear Velocity NED",
            "Attitude RPY",
            "Angular Velocity FRD",
            "Angular Acceleration FRD",
        ):
            self.assertIn(expected_name, names)

        self.assertNotIn("Acceleration Setpoint NED", names)
        self.assertFalse(any(path.startswith("/am/setpoint/") for path in contents))
        self.assertFalse(any(path.startswith("/am/control/rates_setpoint/") for path in contents))

    def test_control_blueprint_splits_outputs_into_separate_charts(self):
        module = load_module()

        blueprint = module.build_am_blueprint()
        control = find_blueprint_part(blueprint.root_container, "Control")
        self.assertIsNotNone(control)
        names = blueprint_names(control)

        for expected_name in (
            "Motor Output",
            "Policy Raw Action",
            "Policy Mapped Action",
            "Policy Motor Control 0-3",
            "Rates Actual",
            "Rates Setpoint",
            "Thrust Setpoint Body",
            "Rates Thrust Body",
        ):
            self.assertIn(expected_name, names)
        self.assertNotIn("Policy Output", names)
        self.assertNotIn("Thrust & Rates", names)

        thrust = find_blueprint_part(control, "Thrust Setpoint Body")
        self.assertIsNotNone(thrust)
        self.assertEqual(
            tuple(thrust.contents),
            (
                "/am/control/thrust_setpoint/body/x",
                "/am/control/thrust_setpoint/body/y",
                "/am/control/thrust_setpoint/body/z",
            ),
        )

        rates_setpoint = find_blueprint_part(control, "Rates Setpoint")
        self.assertIsNotNone(rates_setpoint)
        self.assertEqual(
            tuple(rates_setpoint.contents),
            (
                "/am/control/rates_setpoint/roll",
                "/am/control/rates_setpoint/pitch",
                "/am/control/rates_setpoint/yaw",
            ),
        )

    def test_blueprint_can_be_saved_as_default_rerun_layout(self):
        module = load_module()
        import rerun as rr

        with tempfile.TemporaryDirectory() as tmpdir:
            rrd_path = pathlib.Path(tmpdir) / "layout.rrd"
            recording = rr.RecordingStream("test_px4_am_ulog")

            recording.save(str(rrd_path), default_blueprint=module.build_am_blueprint())
            recording.flush(timeout_sec=5.0)
            self.assertTrue(rrd_path.is_file())

    def test_recording_sends_blueprint_as_active_and_default_layout(self):
        module = load_module()

        recording = RecordingSpy()
        blueprint = module.build_am_blueprint()
        module._save_recording_with_blueprint(recording, pathlib.Path("/tmp/test-layout.rrd"), blueprint)

        self.assertEqual(len(recording.saved), 1)
        self.assertEqual(len(recording.blueprints), 1)
        _, kwargs = recording.blueprints[0]
        self.assertTrue(kwargs["make_active"])
        self.assertTrue(kwargs["make_default"])

    def test_blueprint_key_groups_reference_actual_entities(self):
        module = load_module()

        groups = module.build_time_series_groups()
        all_contents = "\n".join(
            "\n".join(group.contents)
            for group in groups
        )

        for expected in (
            "/am/control/actuator_motors/motor/0",
            "/am/control/actuator_motors/motor/1",
            "/am/control/actuator_motors/motor/2",
            "/am/control/actuator_motors/motor/3",
            "/am/policy/raw_action/0",
            "/am/policy/mapped_action/0",
            "/am/state/local_position/velocity_ned/x",
            "/am/state/local_position/velocity_ned/y",
            "/am/state/local_position/velocity_ned/z",
            "/am/state/angular_velocity/body_rates_frd/x",
            "/am/state/angular_velocity/body_rates_frd/y",
            "/am/state/angular_velocity/body_rates_frd/z",
            "/am/policy/manual_hover_activation/manual_xy_hold",
            "/am/policy/manual_hover_activation/manual_z_hold",
            "/am/policy/manual_hover_activation/manual_hover_active",
        ):
            self.assertIn(expected, all_contents)

        for group_name, expected_paths in {
            "Position NED": (
                "/am/state/local_position/position_ned/x",
            ),
            "Linear Velocity NED": (
                "/am/state/local_position/velocity_ned/x",
            ),
            "Attitude RPY": (
                "/am/state/attitude/yaw",
            ),
            "Angular Velocity FRD": (
                "/am/state/angular_velocity/body_rates_frd/x",
            ),
        }.items():
            contents = group_by_name(groups, group_name).contents
            for expected_path in expected_paths:
                self.assertIn(expected_path, contents)
            self.assertFalse(any(path.startswith("/am/setpoint/") for path in contents))
            self.assertFalse(any(path.startswith("/am/control/rates_setpoint/") for path in contents))

    def test_series_labels_distinguish_actual_and_setpoint_values(self):
        module = load_module()

        for path, expected_label in {
            "/am/state/local_position/velocity_ned/x": "actual_vx",
            "/am/setpoint/trajectory/velocity_ned/x": "setpoint_vx",
            "/am/state/angular_velocity/body_rates_frd/x": "actual_roll_rate",
            "/am/control/rates_setpoint/roll": "setpoint_roll_rate",
            "/am/state/local_position/position_ned/x": "actual_pos_x",
            "/am/setpoint/trajectory/position_ned/x": "setpoint_pos_x",
            "/am/state/attitude/yaw": "actual_yaw",
            "/am/setpoint/attitude/yaw": "setpoint_yaw",
            "/am/setpoint/trajectory/yaw": "setpoint_yaw",
        }.items():
            self.assertEqual(module._series_name(path), expected_label)

    def test_am_policy_observation_names_match_30_dim_policy(self):
        module = load_module()

        names = module._am_policy_observation_names()

        self.assertEqual(len(names), 30)
        self.assertEqual(names[21:26], [f"arm_position/{idx}" for idx in range(5)])
        self.assertEqual(names[26:30], [f"previous_action/{idx}" for idx in range(4)])
        self.assertFalse(any(name.startswith("arm_velocity/") for name in names))

    def test_scene_entity_metadata_names_actual_frame_and_body_axes(self):
        module = load_module()

        self.assertEqual(module.SCENE_ENTITIES["vehicle_path"].label, "vehicle_actual_path")
        self.assertEqual(module.SCENE_ENTITIES["vehicle_frame"].child_frame, "vehicle_flu")
        self.assertEqual(module.SCENE_ENTITIES["vehicle_frame"].parent_frame, "world_enu")
        self.assertEqual(module.SCENE_ENTITIES["vehicle_body_axes"].path, "/am/scene/vehicle/body_axes")
        for axis in ("x", "y", "z"):
            self.assertEqual(
                module.SCENE_ENTITIES[f"vehicle_body_axis_{axis}"].path,
                f"/am/scene/vehicle/body_axes/{axis}",
            )
        self.assertNotIn("vehicle_position", module.SCENE_ENTITIES)
        self.assertNotIn("setpoint_path", module.SCENE_ENTITIES)
        self.assertNotIn("setpoint_frame", module.SCENE_ENTITIES)

    def test_am_scene_exports_robot_position_as_moving_vehicle_frame(self):
        module = load_module()
        import rerun as rr

        start = 1_000_000
        local_position = types.SimpleNamespace(
            name="vehicle_local_position",
            data={
                "timestamp": np.array([1_000_000, 2_000_000], dtype=np.uint64),
                "x": np.array([0.0, 1.0], dtype=np.float32),
                "y": np.array([0.0, 2.0], dtype=np.float32),
                "z": np.array([0.0, -1.0], dtype=np.float32),
            },
        )
        ulog = types.SimpleNamespace(start_timestamp=start, data_list=[local_position])
        recording = RecordingSpy()

        module._export_am_scene(rr, recording, ulog)

        sent_paths = [path for path, _, _ in recording.sent]
        logged_paths = [path for path, _, _ in recording.logged]
        self.assertIn(module.SCENE_ENTITIES["vehicle_frame"].path, sent_paths)
        for axis in ("x", "y", "z"):
            self.assertIn(module.SCENE_ENTITIES[f"vehicle_body_axis_{axis}"].path, sent_paths)
        self.assertNotIn(module.SCENE_ENTITIES["vehicle_body_axes"].path, logged_paths)
        self.assertNotIn("/am/scene/vehicle/current_position", sent_paths)

    def test_vehicle_body_axis_arrows_have_xyz_labels(self):
        module = load_module()

        class FakeArrows3D:
            calls = []

            @staticmethod
            def columns(**kwargs):
                FakeArrows3D.calls.append(kwargs)
                return kwargs

        fake_rr = types.SimpleNamespace(
            Arrows3D=FakeArrows3D,
            TimeColumn=lambda *args, **kwargs: (args, kwargs),
        )
        recording = RecordingSpy()

        module._send_vehicle_body_axis_columns(
            rr=fake_rr,
            recording=recording,
            times_s=np.array([0.0, 1.0], dtype=np.float64),
            origins=np.array([[0.0, 0.0, 0.0], [1.0, 2.0, 3.0]], dtype=np.float64),
            quats_wxyz=np.array([[1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]], dtype=np.float64),
        )

        self.assertEqual([list(call["labels"]) for call in FakeArrows3D.calls], [["X", "X"], ["Y", "Y"], ["Z", "Z"]])
        self.assertEqual([list(call["show_labels"]) for call in FakeArrows3D.calls], [[True, True], [True, True], [True, True]])

    def test_vehicle_body_axes_are_compact_scene_markers(self):
        module = load_module()

        self.assertLessEqual(module.VEHICLE_BODY_AXIS_LENGTH, 0.35)
        self.assertLessEqual(module.VEHICLE_BODY_AXIS_RADIUS, 0.02)

    def test_status_and_policy_flag_colors_are_distinct(self):
        module = load_module()

        colors = {
            tuple(module._series_color(path))
            for path in (
                "/am/status/am_pos_control/module_running",
                "/am/status/am_pos_control/manual_control_available",
                "/am/status/am_pos_control/arm_state_valid",
                "/am/status/offboard_control_mode/position",
                "/am/policy/failure_flags/vehicle_state_invalid",
                "/am/policy/degraded_flags/setpoint_defaulted",
            )
        }

        self.assertGreater(len(colors), 3)
        self.assertNotEqual(module._series_color("/am/policy/failure_flags/vehicle_state_invalid"), module._series_color("/am/policy/failure_flags/arm_state_invalid"))
        self.assertNotEqual(module._series_color("/am/policy/degraded_flags/local_xy_invalid"), module._series_color("/am/policy/degraded_flags/setpoint_defaulted"))

    def test_manual_hover_activation_fields_match_px4_position_hold_logic(self):
        module = load_module()

        start = 1_000_000
        timestamps = np.array([1_000_000, 2_000_000, 3_000_000, 4_000_000, 5_000_000], dtype=np.uint64)
        manual = types.SimpleNamespace(
            name="manual_control_setpoint",
            data={
                "timestamp": timestamps,
                "valid": np.array([1, 1, 1, 1, 0], dtype=np.uint8),
                "pitch": np.array([0.0, 0.2, 0.0, 0.0, 0.0], dtype=np.float32),
                "roll": np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32),
                "throttle": np.array([0.0, 0.0, 0.2, 0.0, 0.0], dtype=np.float32),
            },
        )
        local_position = types.SimpleNamespace(
            name="vehicle_local_position",
            data={
                "timestamp": timestamps,
                "vx": np.array([0.1, 0.1, 0.1, 1.0, 0.1], dtype=np.float32),
                "vy": np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32),
                "vz": np.array([0.1, 0.1, 0.1, 0.1, 0.1], dtype=np.float32),
            },
        )
        ulog = types.SimpleNamespace(
            start_timestamp=start,
            data_list=[manual, local_position],
            initial_parameters={
                "MAN_DEADZONE": 0.1,
                "MPC_HOLD_MAX_XY": 0.8,
                "MPC_HOLD_MAX_Z": 0.6,
            },
        )

        times_s, fields = module._manual_hover_activation_fields(ulog)

        np.testing.assert_allclose(times_s, np.array([0.0, 1.0, 2.0, 3.0, 4.0]))
        np.testing.assert_array_equal(fields["manual_xy_hold"], np.array([1, 0, 1, 0, 0], dtype=np.uint8))
        np.testing.assert_array_equal(fields["manual_z_hold"], np.array([1, 1, 0, 1, 0], dtype=np.uint8))
        np.testing.assert_array_equal(fields["manual_hover_active"], np.array([1, 0, 0, 0, 0], dtype=np.uint8))


if __name__ == "__main__":
    unittest.main()
