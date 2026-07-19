import pathlib
import re
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]


COMMANDER_PACKAGES = {
    "px4_velocity_commander": {
        "launches": [
            "sim_px4_velocity_commander.launch.py",
            "real_px4_velocity_commander.launch.py",
        ],
        "configs": [
            "sim_px4_velocity_commander.yaml",
            "real_px4_velocity_commander.yaml",
        ],
        "node_sources": ["src/velocity_commander_node.cpp"],
        "feedback_topics": [
            "/px4_velocity_commander/command_velocity",
            "/px4_velocity_commander/measured_velocity",
            "/px4_velocity_commander/velocity_error",
        ],
    },
    "arm_trajectory_commander": {
        "launches": [
            "sim_arm_trajectory_commander.launch.py",
            "real_arm_trajectory_commander.launch.py",
        ],
        "configs": [
            "sim_arm_trajectory_commander.yaml",
            "real_arm_trajectory_commander.yaml",
        ],
        "node_sources": ["src/arm_trajectory_commander_node.cpp"],
        "feedback_topics": [],
    },
    "aerial_manipulator_commander": {
        "launches": [
            "sim_aerial_manipulator_commander.launch.py",
            "real_aerial_manipulator_commander.launch.py",
        ],
        "configs": [],
        "node_sources": [],
        "feedback_topics": [],
    },
}


ARM_INTERNAL_PARAMETERS = [
    "enable_sync_handshake",
    "auto_start_tracking",
    "require_follower_state_before_tracking",
    "follower_state_timeout_s",
    "sync_status_timeout_s",
    "ready_dwell_s",
    "allow_wall_time_without_clock",
]


OLD_PACKAGE_NAMES = [
    "_".join(["px4", "pid", "velocity", "profile"]),
    "_".join(["am", "position", "velocity", "profile"]),
    "_".join(["acetele", "leader", "arm", "trajectory"]),
]


class CommanderLaunchTests(unittest.TestCase):
    def package_root(self, package_name):
        return REPO_ROOT / "trajectory_generators" / package_name

    def test_commander_packages_exist_with_expected_launch_files(self):
        for package_name, package in COMMANDER_PACKAGES.items():
            with self.subTest(package=package_name):
                self.assertTrue((self.package_root(package_name) / "package.xml").is_file())
                launch_dir = self.package_root(package_name) / "launch"
                for launch_name in package["launches"]:
                    self.assertTrue(
                        (launch_dir / launch_name).is_file(),
                        f"missing launch file: {launch_dir / launch_name}",
                    )

    def test_sim_and_real_configs_set_explicit_time_mode(self):
        for package_name, package in COMMANDER_PACKAGES.items():
            config_dir = self.package_root(package_name) / "config"
            for config_name in package["configs"]:
                with self.subTest(package=package_name, config=config_name):
                    config_path = config_dir / config_name
                    self.assertTrue(config_path.is_file(), f"missing config file: {config_path}")
                    content = config_path.read_text(encoding="utf-8")
                    expected_value = "true" if config_name.startswith("sim_") else "false"
                    self.assertRegex(
                        content,
                        re.compile(rf"^\s*use_sim_time:\s*{expected_value}\s*$", re.MULTILINE),
                        f"{config_path} must set use_sim_time: {expected_value}",
                    )

    def test_velocity_commander_uses_canonical_feedback_topics(self):
        package = COMMANDER_PACKAGES["px4_velocity_commander"]
        config_text = "\n".join(
            (self.package_root("px4_velocity_commander") / "config" / name).read_text(
                encoding="utf-8")
            for name in package["configs"]
        )

        for topic in package["feedback_topics"]:
            with self.subTest(topic=topic):
                self.assertIn(topic, config_text)

        self.assertIn("offboard_control_mode_topic: /fmu/in/offboard_control_mode", config_text)
        self.assertIn("trajectory_setpoint_topic: /fmu/in/trajectory_setpoint", config_text)

    def test_commander_nodes_use_steady_timers_with_sim_clock_fallback(self):
        for package_name, package in COMMANDER_PACKAGES.items():
            for source_name in package["node_sources"]:
                with self.subTest(package=package_name, source=source_name):
                    source_path = self.package_root(package_name) / source_name
                    content = source_path.read_text(encoding="utf-8")
                    self.assertNotIn("create_wall_timer", content)
                    self.assertRegex(content, re.compile(r"rclcpp::create_timer\s*\("))
                    self.assertIn("RCL_STEADY_TIME", content)
                    self.assertIn("ClockFallback", content)
                    self.assertIn("this->get_clock()", content)

    def test_aerial_manipulator_launches_include_both_commanders(self):
        launch_dir = self.package_root("aerial_manipulator_commander") / "launch"
        for launch_name in COMMANDER_PACKAGES["aerial_manipulator_commander"]["launches"]:
            with self.subTest(launch=launch_name):
                content = (launch_dir / launch_name).read_text(encoding="utf-8")
                self.assertIn("px4_velocity_commander", content)
                self.assertIn("arm_trajectory_commander", content)
                self.assertIn("GroupAction", content)
                self.assertIn("scoped=True", content)
                self.assertIn("IncludeLaunchDescription", content)

    def test_arm_commander_configs_use_velocity_limits_not_empty_optional_arrays(self):
        config_dir = self.package_root("arm_trajectory_commander") / "config"
        for config_name in COMMANDER_PACKAGES["arm_trajectory_commander"]["configs"]:
            with self.subTest(config=config_name):
                content = (config_dir / config_name).read_text(encoding="utf-8")
                self.assertIn("segment_durations_s:", content)
                self.assertIn("segment_durations_s: [2.0, 2.0, 2.0, 2.0]", content)
                self.assertNotIn("waypoint_times_s:", content)
                self.assertIn("max_joint_velocity_rad_s:", content)
                self.assertIn("max_gripper_velocity_rad_s:", content)
                self.assertIn("loop_count:", content)
                self.assertNotIn("loop:", content)
                self.assertNotIn("velocities: []", content)
                self.assertNotIn("efforts: []", content)
                self.assertNotIn("gripper_velocities: []", content)
                self.assertNotIn("gripper_efforts: []", content)
                self.assertNotIn("sync_mode: tracking", content)
                self.assertIn("follower_arm_state_topic: /ace_follower/arm/state", content)
                self.assertIn("follower_sync_status_topic: /ace_follower/arm/sync_status", content)
                self.assertIn("follower_gripper_state_topic: /ace_follower/gripper/state", content)
                self.assertRegex(
                    content,
                    re.compile(r"^\s*vehicle_position_source:\s*\w+\s*$", re.MULTILINE),
                )
                topic_matches = re.findall(
                    r"^\s*vehicle_(?:pose|odometry)_topic:\s*(\S+)\s*$",
                    content,
                    re.MULTILINE,
                )
                self.assertGreaterEqual(len(topic_matches), 1)
                for topic in topic_matches:
                    self.assertTrue(topic.startswith("/"), f"{topic} must be absolute")

    def test_arm_commander_configs_hide_internal_handshake_parameters(self):
        config_dir = self.package_root("arm_trajectory_commander") / "config"
        for config_name in COMMANDER_PACKAGES["arm_trajectory_commander"]["configs"]:
            with self.subTest(config=config_name):
                content = (config_dir / config_name).read_text(encoding="utf-8")
                for parameter in ARM_INTERNAL_PARAMETERS:
                    self.assertNotIn(f"{parameter}:", content)

    def test_arm_commander_derives_follower_state_guard_from_time_mode(self):
        source = (
            self.package_root("arm_trajectory_commander")
            / "src"
            / "arm_trajectory_commander_node.cpp"
        ).read_text(encoding="utf-8")

        self.assertIn(
            'declare_parameter<bool>("require_follower_state_before_tracking", !use_sim_time_)',
            source,
        )

    def test_arm_commander_uses_loop_count_not_loop_bool(self):
        source = (
            self.package_root("arm_trajectory_commander")
            / "src"
            / "arm_trajectory_commander_node.cpp"
        ).read_text(encoding="utf-8")
        header = (
            self.package_root("arm_trajectory_commander")
            / "include"
            / "arm_trajectory_commander"
            / "arm_trajectory.hpp"
        ).read_text(encoding="utf-8")

        self.assertIn('declare_parameter<int>("loop_count", 1)', source)
        self.assertIn("int loop_count{1};", header)
        self.assertNotIn('declare_parameter<bool>("loop"', source)
        self.assertNotIn("bool loop", header)

    def test_arm_commander_uses_segment_durations_not_waypoint_times(self):
        source = (
            self.package_root("arm_trajectory_commander")
            / "src"
            / "arm_trajectory_commander_node.cpp"
        ).read_text(encoding="utf-8")
        header = (
            self.package_root("arm_trajectory_commander")
            / "include"
            / "arm_trajectory_commander"
            / "arm_trajectory.hpp"
        ).read_text(encoding="utf-8")

        self.assertIn('"segment_durations_s"', source)
        self.assertIn("segment_durations_s", header)
        self.assertNotIn('"waypoint_times_s"', source)
        self.assertNotIn("waypoint_times_s", header)

    def test_arm_commander_declares_vehicle_position_metrics_inputs(self):
        source = (
            self.package_root("arm_trajectory_commander")
            / "src"
            / "arm_trajectory_commander_node.cpp"
        ).read_text(encoding="utf-8")
        package_xml = (self.package_root("arm_trajectory_commander") / "package.xml").read_text(
            encoding="utf-8")

        self.assertIn('"vehicle_position_source"', source)
        self.assertIn('"vehicle_pose_topic"', source)
        self.assertIn('"vehicle_odometry_topic"', source)
        self.assertIn("parseVehiclePositionSource", source)
        self.assertIn("<depend>geometry_msgs</depend>", package_xml)
        self.assertIn("<depend>nav_msgs</depend>", package_xml)

    def test_arm_commander_uses_sensor_qos_for_vehicle_position_metrics(self):
        source = (
            self.package_root("arm_trajectory_commander")
            / "src"
            / "arm_trajectory_commander_node.cpp"
        ).read_text(encoding="utf-8")

        self.assertIn("rclcpp::SensorDataQoS", source)
        self.assertRegex(
            source,
            re.compile(
                r"create_subscription<geometry_msgs::msg::PoseStamped>\(\s*"
                r"vehicle_pose_topic,\s*vehicle_sensor_qos,"
            ),
        )
        self.assertRegex(
            source,
            re.compile(
                r"create_subscription<nav_msgs::msg::Odometry>\(\s*"
                r"vehicle_odometry_topic,\s*vehicle_sensor_qos,"
            ),
        )

    def test_arm_commander_stops_after_finished_non_loop_trajectory(self):
        source = (
            self.package_root("arm_trajectory_commander")
            / "src"
            / "arm_trajectory_commander_node.cpp"
        ).read_text(encoding="utf-8")

        self.assertIn("trajectory_finished_", source)
        self.assertIn('publishSyncMode("stop")', source)
        self.assertIn("sample.finished", source)
        self.assertRegex(
            source,
            re.compile(
                r"if\s*\(\s*trajectory_finished_\s*\)\s*\{"
                r"\s*publishSyncMode\(\"stop\"\);"
                r".*?rclcpp::shutdown\(\);"
                r"\s*return;",
                re.DOTALL,
            ),
        )

    def test_arm_commander_reports_vehicle_hover_summary_before_shutdown(self):
        source = (
            self.package_root("arm_trajectory_commander")
            / "src"
            / "arm_trajectory_commander_node.cpp"
        ).read_text(encoding="utf-8")

        self.assertIn("Vehicle hover summary", source)
        self.assertIn("Vehicle hover summary unavailable", source)
        self.assertRegex(
            source,
            re.compile(
                r"if\s*\(\s*trajectory_finished_\s*\)\s*\{"
                r".*?reportVehiclePositionSummary\(\);"
                r".*?rclcpp::shutdown\(\);",
                re.DOTALL,
            ),
        )

    def test_sim_configs_allow_no_clock_fallback(self):
        content = (
            self.package_root("px4_velocity_commander")
            / "config"
            / "sim_px4_velocity_commander.yaml"
        ).read_text(encoding="utf-8")
        self.assertIn("use_sim_time: true", content)
        self.assertIn("sim_clock_topic: /acesim/clock", content)
        self.assertIn("allow_wall_time_without_clock: true", content)

        arm_content = (
            self.package_root("arm_trajectory_commander")
            / "config"
            / "sim_arm_trajectory_commander.yaml"
        ).read_text(encoding="utf-8")
        self.assertIn("use_sim_time: true", arm_content)
        self.assertIn("sim_clock_topic: /acesim/clock", arm_content)
        self.assertNotIn("allow_wall_time_without_clock:", arm_content)

    def test_px4_velocity_sim_config_uses_acesim_odometry_measurement(self):
        content = (
            self.package_root("px4_velocity_commander")
            / "config"
            / "sim_px4_velocity_commander.yaml"
        ).read_text(encoding="utf-8")

        self.assertIn("measurement_source: odometry_pose", content)
        self.assertIn("measured_odometry_topic: /acesim/vehicle/odometry", content)
        self.assertIn("sim_clock_topic: /acesim/clock", content)
        self.assertNotIn("mocap_pose_topic", content)

    def test_px4_velocity_real_config_keeps_pose_stamped_measurement(self):
        content = (
            self.package_root("px4_velocity_commander")
            / "config"
            / "real_px4_velocity_commander.yaml"
        ).read_text(encoding="utf-8")

        self.assertIn("measurement_source: pose_stamped", content)
        self.assertIn("mocap_pose_topic: xxy/pose", content)
        self.assertNotIn("measured_odometry_topic", content)

    def test_aerial_manipulator_sim_launch_passes_acesim_clock_and_odometry_args(self):
        content = (
            self.package_root("aerial_manipulator_commander")
            / "launch"
            / "sim_aerial_manipulator_commander.launch.py"
        ).read_text(encoding="utf-8")

        self.assertIn('"sim_clock_topic"', content)
        self.assertIn("default_value=\"/acesim/clock\"", content)
        self.assertNotIn('"measurement_source"', content)
        self.assertNotIn("default_value=\"odometry_pose\"", content)
        self.assertIn('"measured_odometry_topic"', content)
        self.assertIn("default_value=\"/acesim/vehicle/odometry\"", content)
        self.assertNotIn('"mocap_pose_topic"', content)
        self.assertNotIn("LaunchConfiguration(\"mocap_pose_topic\")", content)

    def test_am_position_sim_configs_use_acesim_clock(self):
        config_dir = REPO_ROOT / "px4_external_modes" / "am_position_mode" / "config"
        for config_name in ("sim_am_position_motor.yaml", "sim_am_position_ctbr.yaml"):
            with self.subTest(config=config_name):
                content = (config_dir / config_name).read_text(encoding="utf-8")
                self.assertIn("use_sim_time: true", content)
                self.assertIn("sim_clock_topic: /acesim/clock", content)

    def test_old_commander_package_names_are_removed_from_source_docs_and_tests(self):
        search_roots = [
            REPO_ROOT / "README.md",
            REPO_ROOT / "AGENT.md",
            REPO_ROOT / "tools",
            REPO_ROOT / "trajectory_generators",
        ]

        for old_name in OLD_PACKAGE_NAMES:
            with self.subTest(old_name=old_name):
                matches = []
                for root in search_roots:
                    paths = [root] if root.is_file() else root.rglob("*")
                    for path in paths:
                        if (
                            path.is_dir()
                            or path == pathlib.Path(__file__)
                            or "__pycache__" in path.parts
                            or path.suffix == ".pyc"
                        ):
                            continue
                        text = path.read_text(encoding="utf-8", errors="ignore")
                        if old_name in text:
                            matches.append(str(path.relative_to(REPO_ROOT)))
                self.assertEqual(matches, [], f"old package name remains in: {matches}")


if __name__ == "__main__":
    unittest.main()
