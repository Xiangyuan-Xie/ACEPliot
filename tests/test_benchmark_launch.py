import pathlib
import re
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
BENCHMARKS_ROOT = REPO_ROOT / "benchmarks"


BENCHMARK_PACKAGES = {
    "velocity_tracking_benchmark": {
        "launches": {
            "sim_velocity_tracking_benchmark.launch.py",
            "real_velocity_tracking_benchmark.launch.py",
        },
        "configs": {
            "sim_velocity_tracking_benchmark.yaml",
            "real_velocity_tracking_benchmark.yaml",
        },
    },
    "arm_motion_benchmark": {
        "launches": {
            "sim_arm_motion_benchmark.launch.py",
            "real_arm_motion_benchmark.launch.py",
        },
        "configs": {
            "sim_arm_motion_benchmark.yaml",
            "real_arm_motion_benchmark.yaml",
        },
    },
    "aerial_manipulation_benchmark": {
        "launches": {
            "sim_aerial_manipulation_benchmark.launch.py",
            "real_aerial_manipulation_benchmark.launch.py",
        },
        "configs": set(),
    },
    "figure8_tracking_benchmark": {
        "launches": {
            "sim_px4_figure8_tracking_benchmark.launch.py",
            "real_px4_figure8_tracking_benchmark.launch.py",
            "sim_ee_figure8_tracking_benchmark.launch.py",
            "real_ee_figure8_tracking_benchmark.launch.py",
        },
        "configs": set(),
    },
    "benchmark_reporting": {"launches": set(), "configs": set()},
}


class BenchmarkLaunchTests(unittest.TestCase):
    def package(self, name):
        return BENCHMARKS_ROOT / name

    def test_packages_use_ament_python_and_expected_public_files(self):
        for name, contract in BENCHMARK_PACKAGES.items():
            with self.subTest(package=name):
                package = self.package(name)
                self.assertTrue((package / "package.xml").is_file())
                self.assertTrue((package / "setup.py").is_file())
                package_xml = (package / "package.xml").read_text(encoding="utf-8")
                self.assertIn("<build_type>ament_python</build_type>", package_xml)
                launches = {
                    path.name for path in (package / "launch").glob("*.launch.py")
                }
                configs = {path.name for path in (package / "config").glob("*.yaml")}
                self.assertEqual(launches, contract["launches"])
                self.assertEqual(configs, contract["configs"])

    def test_velocity_topics_and_environment_sources_are_canonical(self):
        config_dir = self.package("velocity_tracking_benchmark") / "config"
        sim = (config_dir / "sim_velocity_tracking_benchmark.yaml").read_text(
            encoding="utf-8"
        )
        real = (config_dir / "real_velocity_tracking_benchmark.yaml").read_text(
            encoding="utf-8"
        )
        for topic in (
            "/velocity_tracking_benchmark/command_velocity",
            "/velocity_tracking_benchmark/measured_velocity",
            "/velocity_tracking_benchmark/velocity_error",
            "/velocity_tracking_benchmark/status",
            "/fmu/in/offboard_control_mode",
            "/fmu/in/trajectory_setpoint",
        ):
            self.assertIn(topic, sim)
            self.assertIn(topic, real)
        self.assertIn("use_sim_clock: true", sim)
        self.assertIn("sim_clock_topic: /acesim/clock", sim)
        self.assertIn("measured_odometry_topic: /acesim/vehicle/odometry", sim)
        self.assertNotIn("allow_wall_time_without_clock", sim)
        self.assertNotIn("mocap_pose_topic", sim)
        self.assertIn("use_sim_clock: false", real)
        self.assertIn("mocap_pose_topic: /xxy/pose", real)

    def test_arm_motion_keeps_motion_contract_but_reports_only_base(self):
        package = self.package("arm_motion_benchmark")
        for config_path in (package / "config").glob("*.yaml"):
            content = config_path.read_text(encoding="utf-8")
            self.assertIn("segment_durations_s: [2.0, 2.0, 2.0, 2.0]", content)
            self.assertIn("loop_count:", content)
            self.assertNotIn("waypoint_times_s:", content)
            self.assertIn("/ace_leader/arm/command", content)
            self.assertIn("/ace_follower/arm/sync_status", content)
            self.assertIn("/arm_motion_benchmark/status", content)
            self.assertRegex(
                content,
                re.compile(r"^\s*vehicle_position_source:\s*\w+\s*$", re.MULTILINE),
            )
        reporter = (
            self.package("benchmark_reporting")
            / "benchmark_reporting"
            / "reporter_node.py"
        ).read_text(encoding="utf-8")
        reporting_manifest = (
            self.package("benchmark_reporting") / "package.xml"
        ).read_text(encoding="utf-8")
        self.assertNotIn("JointState", reporter)
        self.assertNotIn("sensor_msgs", reporting_manifest)

    def test_hidden_arm_handshake_policy_is_not_yaml_surface(self):
        hidden = (
            "enable_sync_handshake",
            "auto_start_tracking",
            "require_follower_state_before_tracking",
            "follower_state_timeout_s",
            "sync_status_timeout_s",
            "ready_dwell_s",
            "allow_wall_time_without_clock",
        )
        for config_path in (
            self.package("arm_motion_benchmark") / "config"
        ).glob("*.yaml"):
            content = config_path.read_text(encoding="utf-8")
            for parameter in hidden:
                self.assertNotIn(f"{parameter}:", content)

    def test_aerial_launches_compose_workloads_and_one_reporter(self):
        launch_dir = self.package("aerial_manipulation_benchmark") / "launch"
        for path in launch_dir.glob("*.launch.py"):
            content = path.read_text(encoding="utf-8")
            self.assertIn('package="velocity_tracking_benchmark"', content)
            self.assertIn('package="arm_motion_benchmark"', content)
            self.assertIn('"report_type": "aerial"', content)
            self.assertIn("OnProcessExit", content)
            self.assertIn("Shutdown", content)

    def test_reporter_declares_expected_artifacts_and_invalid_status(self):
        reporting = self.package("benchmark_reporting") / "benchmark_reporting"
        reporter = (reporting / "reporter_node.py").read_text(encoding="utf-8")
        style = (reporting / "figure_style.py").read_text(encoding="utf-8")
        plots = (reporting / "plots.py").read_text(encoding="utf-8")
        self.assertIn('"summary.json"', reporter)
        self.assertIn('status = "invalid"', reporter)
        self.assertIn("write_csv", reporter)
        self.assertIn("figure.savefig", style)
        self.assertIn('matplotlib.use("Agg", force=True)', style)
        self.assertIn('"svg.fonttype": "none"', style)
        self.assertIn('("png", "svg", "pdf")', style)
        self.assertIn('projection="3d"', plots)
        for artifact in ("overview", "velocity_error", "orientation_error"):
            self.assertIn(f'"{artifact}"', reporter)

    def test_launches_store_reports_in_their_benchmark_package(self):
        for package_name, contract in BENCHMARK_PACKAGES.items():
            for launch_name in contract["launches"]:
                launch = (
                    self.package(package_name) / "launch" / launch_name
                ).read_text(encoding="utf-8")
                self.assertIn(
                    f'"benchmark_package": "{package_name}"',
                    launch,
                )

    def test_figure8_tracking_launches_own_only_acepliot_nodes(self):
        launch_dir = self.package("figure8_tracking_benchmark") / "launch"
        for path in launch_dir.glob("*.launch.py"):
            content = path.read_text(encoding="utf-8")
            self.assertIn('package="figure8_trajectory"', content)
            self.assertIn('package="benchmark_reporting"', content)
            self.assertNotIn("acesim_ros2", content)
            if "_ee_" in path.name:
                self.assertIn('package="am_ee_pose_commander"', content)
                self.assertIn("upper_model_path", content)
            else:
                self.assertNotIn('package="am_ee_pose_commander"', content)

    def test_figure8_sim_trajectory_and_reporter_share_acesim_clock(self):
        launch_dir = self.package("figure8_tracking_benchmark") / "launch"
        for launch_name in (
            "sim_px4_figure8_tracking_benchmark.launch.py",
            "sim_ee_figure8_tracking_benchmark.launch.py",
        ):
            content = (launch_dir / launch_name).read_text(encoding="utf-8")
            self.assertEqual(content.count('"use_sim_clock": True'), 2)
            self.assertEqual(
                content.count('"sim_clock_topic": "/acesim/clock"'), 2
            )

    def test_am_position_sim_config_still_uses_acesim_clock(self):
        config = (
            REPO_ROOT
            / "px4_external_modes"
            / "am_position_mode"
            / "config"
            / "sim_am_position.yaml"
        ).read_text(encoding="utf-8")
        self.assertIn("use_sim_time: true", config)
        self.assertIn("sim_clock_topic: /acesim/clock", config)


if __name__ == "__main__":
    unittest.main()
