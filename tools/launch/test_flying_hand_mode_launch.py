import pathlib
import re
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
EXTERNAL_MODES = REPO_ROOT / "px4_external_modes"
QUAD = EXTERNAL_MODES / "flying_hand_quadrotor_mode"
FULL = EXTERNAL_MODES / "flying_hand_fully_actuated_mode"
COMMON = EXTERNAL_MODES / "flying_hand_control_common"


def yaml_array(text, parameter):
    match = re.search(
        rf"^\s*{re.escape(parameter)}:\s*\[([^\]]*)\]\s*$",
        text,
        re.MULTILINE,
    )
    if match is None:
        raise AssertionError(f"missing YAML array {parameter}")
    return [item.strip() for item in match.group(1).split(",") if item.strip()]


class FlyingHandModeLaunchTests(unittest.TestCase):
    def test_modes_have_distinct_package_and_launch_names(self):
        self.assertFalse((EXTERNAL_MODES / "flying_hand_mode").exists())
        expected = {
            QUAD: [
                "sim_flying_hand_quadrotor.launch.py",
                "real_flying_hand_quadrotor_shadow.launch.py",
                "real_flying_hand_quadrotor.launch.py",
            ],
            FULL: [
                "real_flying_hand_fully_actuated_shadow.launch.py",
                "real_flying_hand_fully_actuated.launch.py",
            ],
        }
        for package, launches in expected.items():
            with self.subTest(package=package.name):
                self.assertTrue((package / "package.xml").is_file())
                for launch in launches:
                    self.assertTrue((package / "launch" / launch).is_file())
        self.assertTrue((COMMON / "package.xml").is_file())

    def test_quadrotor_topics_use_distinct_prefix(self):
        config_text = "\n".join(
            path.read_text(encoding="utf-8") for path in (QUAD / "config").glob("*.yaml")
        )
        self.assertIn("/flying_hand_quadrotor/ee_pose_setpoint", config_text)
        self.assertIn("/flying_hand_quadrotor/status", config_text)
        self.assertNotIn("/flying_hand/", config_text)

    def test_fully_actuated_shadow_is_safe_by_default(self):
        config = (
            FULL / "config" / "real_flying_hand_fully_actuated_shadow.yaml"
        ).read_text(encoding="utf-8")
        self.assertRegex(config, re.compile(r"^\s*closed_loop:\s*false$", re.MULTILINE))
        self.assertRegex(
            config,
            re.compile(r"^\s*calibration_confirmed:\s*false$", re.MULTILINE),
        )
        for topic in (
            "ee_pose_setpoint",
            "ee_pose",
            "wrench_nominal",
            "wrench_adaptive",
            "wrench_applied",
            "status",
        ):
            self.assertIn(f"/flying_hand_fully_actuated/{topic}", config)

    def test_fully_actuated_rotor_parameter_lengths_are_exact(self):
        config = (
            FULL / "config" / "real_flying_hand_fully_actuated_shadow.yaml"
        ).read_text(encoding="utf-8")
        self.assertEqual(len(yaml_array(config, "rotor.position_frd_m")), 18)
        self.assertEqual(len(yaml_array(config, "rotor.axis_frd")), 18)
        for parameter in (
            "rotor.moment_ratio_m",
            "rotor.minimum_thrust_n",
            "rotor.maximum_thrust_n",
            "rotor.thrust_curve_kappa",
        ):
            self.assertEqual(len(yaml_array(config, parameter)), 6)

    def test_closed_loop_launch_requires_explicit_config_file(self):
        launch = (
            FULL / "launch" / "real_flying_hand_fully_actuated.launch.py"
        ).read_text(encoding="utf-8")
        self.assertIn('DeclareLaunchArgument(\n                "config_file"', launch)
        self.assertNotIn("default_value", launch)
        self.assertIn("parameters=[config_file]", launch)

    def test_common_runtime_enforces_calibration_and_allocator_feedback(self):
        source = (COMMON / "src" / "external_mode.cpp").read_text(encoding="utf-8")
        self.assertIn("calibration_confirmed=true", source)
        self.assertIn("px4_msgs::msg::ControlAllocatorStatus", source)
        self.assertIn("kAllocatorStatusMissing", source)
        self.assertIn("kAllocatorUnachieved", source)
        self.assertIn("kAllocatorSaturated", source)
        self.assertRegex(
            source,
            re.compile(r"if\s*\(\s*!closed_loop_\s*\|\|\s*shutting_down_\.load\(\)\s*\)"),
        )
        self.assertGreaterEqual(
            len(re.findall(r"if\s*\(\s*!closed_loop_\s*\)\s*\{", source)),
            2,
        )

    def test_generated_solver_matches_paper_dimensions_and_rk4(self):
        header = (
            FULL
            / "generated"
            / "flying_hand_fully_actuated_solver"
            / "acados_solver_flying_hand_fully_actuated.h"
        ).read_text(encoding="utf-8")
        generator = (FULL / "tools" / "generate_solver.py").read_text(encoding="utf-8")
        self.assertIn("FLYING_HAND_FULLY_ACTUATED_NX     17", header)
        self.assertIn("FLYING_HAND_FULLY_ACTUATED_NU     10", header)
        self.assertIn("FLYING_HAND_FULLY_ACTUATED_N      100", header)
        self.assertIn("FLYING_HAND_FULLY_ACTUATED_NH0    10", header)
        self.assertIn("ocp.solver_options.tf = 2.5", generator)
        self.assertIn("ocp.solver_options.sim_method_num_stages = 4", generator)

    def test_generator_keeps_paper_weights_and_marks_stability_term(self):
        generator = (FULL / "tools" / "generate_solver.py").read_text(
            encoding="utf-8"
        )
        expected_weights = {
            "PAPER_POSITION_WEIGHT": "12.0",
            "PAPER_ORIENTATION_WEIGHT": "10.0",
            "PAPER_VELOCITY_WEIGHT": "0.1",
            "PAPER_ARM_POSITION_WEIGHT": "0.1",
            "PAPER_FORCE_WEIGHT": "0.03",
            "PAPER_MOMENT_WEIGHT": "0.1",
            "IMPLEMENTATION_ARM_COMMAND_WEIGHT": "0.03",
        }
        for name, value in expected_weights.items():
            self.assertRegex(
                generator,
                re.compile(rf"^{name} = {re.escape(value)}$", re.MULTILINE),
            )

    def test_fully_actuated_mode_uses_px4_wrench_not_direct_actuators(self):
        source_text = "\n".join(
            path.read_text(encoding="utf-8") for path in (FULL / "src").glob("*.cpp")
        )
        self.assertIn("normalized_thrust_frd", source_text)
        self.assertIn("normalized_torque_frd", source_text)
        self.assertNotIn("DirectActuatorsSetpointType", source_text)
        self.assertNotIn("ActuatorMotors", source_text)

    def test_fully_actuated_launch_does_not_start_acesim(self):
        launch_text = "\n".join(
            path.read_text(encoding="utf-8") for path in (FULL / "launch").glob("*.py")
        )
        self.assertNotIn("acesim", launch_text.lower())
        self.assertNotIn("IncludeLaunchDescription", launch_text)


if __name__ == "__main__":
    unittest.main()
