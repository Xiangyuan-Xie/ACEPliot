import pathlib
import re
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
PACKAGE = REPO_ROOT / "commanders" / "am_ee_pose_commander"
FIGURE8_PACKAGE = REPO_ROOT / "trajectory_generators" / "figure8_trajectory"


class AmEePoseCommanderLaunchTests(unittest.TestCase):
    def test_commander_is_not_a_px4_external_mode(self):
        source = (PACKAGE / "src" / "am_ee_pose_commander_node.cpp").read_text(
            encoding="utf-8"
        )
        self.assertIn('rclcpp::Node("am_ee_pose_commander")', source)
        self.assertNotIn("NodeWithMode", source)
        self.assertNotIn("DirectActuatorsSetpointType", source)
        self.assertFalse(
            (REPO_ROOT / "px4_external_modes" / "am_ee_pose_mode").exists()
        )

    def test_launches_require_only_the_upper_model(self):
        for launch_path in (PACKAGE / "launch").glob("*.launch.py"):
            launch = launch_path.read_text(encoding="utf-8")
            with self.subTest(launch=launch_path.name):
                self.assertIn("upper_model_path", launch)
                self.assertNotIn("low_level_model_path", launch)

    def test_configs_connect_px4_am_offboard_and_acetele(self):
        expected_topics = (
            "/fmu/out/vehicle_odometry",
            "/fmu/in/offboard_control_mode",
            "/fmu/in/trajectory_setpoint",
            "/ace_follower/arm/state",
            "/ace_follower/gripper/state",
            "/ace_follower/arm/sync_status",
            "/ace_leader/arm/command",
            "/ace_leader/gripper/command",
            "/ace_leader/arm/sync_mode",
            "/am_ee_pose/trajectory_preview",
        )
        for config_path in (PACKAGE / "config").glob("*.yaml"):
            config = config_path.read_text(encoding="utf-8")
            with self.subTest(config=config_path.name):
                for topic in expected_topics:
                    self.assertIn(topic, config)

    def test_real_launch_does_not_start_agent_by_default(self):
        launch = (
            PACKAGE / "launch" / "real_am_ee_pose_commander.launch.py"
        ).read_text(encoding="utf-8")
        self.assertRegex(
            launch,
            re.compile(
                r'DeclareLaunchArgument\(\s*"start_micro_xrce_agent",\s*default_value="false"',
                re.MULTILINE,
            ),
        )
        self.assertIn("condition=IfCondition(start_agent)", launch)

    def test_figure8_package_exposes_two_trajectory_entries(self):
        package_xml = (FIGURE8_PACKAGE / "package.xml").read_text(encoding="utf-8")
        launch_names = {
            path.name for path in (FIGURE8_PACKAGE / "launch").glob("*.launch.py")
        }

        self.assertIn("<name>figure8_trajectory</name>", package_xml)
        self.assertIn("<build_type>ament_python</build_type>", package_xml)
        self.assertEqual(
            launch_names,
            {
                "px4_figure8_trajectory.launch.py",
                "ee_figure8_trajectory.launch.py",
            },
        )

    def test_figure8_ee_entry_publishes_preview_without_px4_topics(self):
        source = (
            FIGURE8_PACKAGE / "figure8_trajectory" / "nodes.py"
        ).read_text(encoding="utf-8")
        config_matches = [
            path
            for path in (FIGURE8_PACKAGE / "config").glob("*.yaml")
            if "/am_ee_pose/trajectory_preview" in path.read_text(encoding="utf-8")
        ]
        self.assertEqual(len(config_matches), 1)

        config = config_matches[0].read_text(encoding="utf-8")
        self.assertIn("PREVIEW_OFFSETS_S = (0.0, 0.02, 0.04, 0.06, 1.0)", source)
        self.assertIn("/am_ee_pose/trajectory_preview", config)
        self.assertIn("/acesim/clock", config)
        self.assertNotIn("/fmu/in/", config)


if __name__ == "__main__":
    unittest.main()
