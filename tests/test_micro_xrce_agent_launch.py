import ast
import pathlib
import unittest


REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]

LAUNCH_FILES = [
    REPO_ROOT / "px4_state_converter" / "launch" / "generic_odometry.launch.py",
    REPO_ROOT / "px4_state_converter" / "launch" / "odometry.launch.py",
    REPO_ROOT / "px4_state_converter" / "launch" / "gt_odometry.launch.py",
    REPO_ROOT
    / "px4_external_modes"
    / "am_position_mode"
    / "launch"
    / "real_am_position.launch.py",
]


class MicroXRCEAgentLaunchTests(unittest.TestCase):
    def test_micro_xrce_agent_is_opt_in_for_real_launch_files(self):
        for launch_file in LAUNCH_FILES:
            with self.subTest(launch_file=launch_file.relative_to(REPO_ROOT)):
                source = launch_file.read_text(encoding="utf-8")
                tree = ast.parse(source, filename=str(launch_file))

                self.assertIn("start_micro_xrce_agent", source)
                self.assertRegex(
                    source,
                    r"DeclareLaunchArgument\(\s*['\"]start_micro_xrce_agent['\"]",
                )
                self.assertRegex(
                    source,
                    r"default_value\s*=\s*['\"]false['\"]",
                )

                micro_agent_calls = [
                    node
                    for node in ast.walk(tree)
                    if isinstance(node, ast.Call)
                    and getattr(node.func, "id", "") == "ExecuteProcess"
                    and "MicroXRCEAgent" in ast.get_source_segment(source, node)
                ]
                self.assertEqual(
                    len(micro_agent_calls),
                    1,
                    f"{launch_file} should define exactly one MicroXRCEAgent process",
                )
                call_source = ast.get_source_segment(source, micro_agent_calls[0])
                self.assertIn("condition=IfCondition", call_source)
                self.assertIn("LaunchConfiguration('start_micro_xrce_agent')", call_source)


if __name__ == "__main__":
    unittest.main()
