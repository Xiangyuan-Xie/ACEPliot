from pathlib import Path
import tempfile
import unittest

import numpy as np

from benchmark_reporting.figure_style import pyplot
from benchmark_reporting.plots import (
    _prepare_trajectory,
    plot_aerial_overview_bundle,
    plot_base_overview_bundle,
    plot_pose_overview_bundle,
    plot_trajectory_bundle,
    plot_vector_error_bundle,
    plot_velocity_overview_bundle,
)
from benchmark_reporting.report import ReportWriter, _find_workspace_source_package


class ReportWriterTests(unittest.TestCase):
    def test_finds_package_through_workspace_source_symlink(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            repository = root / "checkout"
            package = repository / "benchmarks" / "example_benchmark"
            package.mkdir(parents=True)
            (package / "package.xml").write_text("<package/>", encoding="utf-8")
            source_root = root / "workspace" / "src"
            source_root.mkdir(parents=True)
            (source_root / "repository").symlink_to(
                repository, target_is_directory=True
            )

            self.assertEqual(
                _find_workspace_source_package("example_benchmark", source_root),
                package,
            )

    def test_default_output_directory_is_package_local(self):
        with tempfile.TemporaryDirectory() as directory:
            package = Path(directory) / "example_benchmark"
            package.mkdir()
            writer = ReportWriter(
                "example_benchmark",
                package_name="example_benchmark",
                package_directory=package,
            )
            self.assertEqual(writer.output_dir.parent, package / "logs")
            self.assertTrue(writer.output_dir.is_dir())

    def test_writes_json_csv_and_publication_figure_bundle(self):
        with tempfile.TemporaryDirectory() as directory:
            writer = ReportWriter("test", directory)
            json_path = writer.write_json("summary.json", {"status": "complete"})
            csv_path = writer.write_csv("samples.csv", [{"time_s": 0.0, "x": 1.0}])
            positions = np.array([[0.0, 0.0, 1.0], [1.0, 0.5, 1.0]])
            trajectory_paths = plot_trajectory_bundle(
                Path(directory), "trajectory", positions, origin=positions[0]
            )
            error_paths = plot_vector_error_bundle(
                Path(directory),
                "error",
                np.array([0.0, 1.0]),
                positions - positions[0],
                "Drift (m)",
            )
            figure_paths = trajectory_paths + error_paths
            self.assertEqual(
                {path.suffix for path in trajectory_paths}, {".png", ".svg", ".pdf"}
            )
            for path in (json_path, csv_path, *figure_paths):
                self.assertTrue(path.is_file())
                self.assertGreater(path.stat().st_size, 0)
            svg = (Path(directory) / "trajectory.svg").read_text(encoding="utf-8")
            self.assertIn("<text", svg)
            pixels = pyplot().imread(Path(directory) / "trajectory.png")
            self.assertGreater(float(np.std(pixels)), 0.0)

    def test_relative_trajectory_preserves_xyz_displacement(self):
        actual = np.array([[10.0, -2.0, 3.0], [11.0, 0.0, 3.5]])
        reference = np.array([[10.0, -2.0, 3.0], [10.5, -1.0, 3.25]])
        prepared_actual, prepared_reference, relative = _prepare_trajectory(
            actual, reference, origin=actual[0]
        )
        np.testing.assert_allclose(
            prepared_actual, [[0.0, 0.0, 0.0], [1.0, 2.0, 0.5]]
        )
        np.testing.assert_allclose(
            prepared_reference, [[0.0, 0.0, 0.0], [0.5, 1.0, 0.25]]
        )
        self.assertTrue(relative)

    def test_stationary_trajectory_and_overview_are_visible_artifacts(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            times = np.linspace(0.0, 1.0, 20)
            reference = np.column_stack(
                (
                    np.sin(2.0 * np.pi * times),
                    0.5 * np.sin(4.0 * np.pi * times),
                    np.ones_like(times),
                )
            )
            actual = reference + np.column_stack(
                (0.01 * times, -0.01 * times, 0.02 * times)
            )
            stationary = np.tile([0.0, 0.0, 1.0], (len(times), 1))
            trajectory_paths = plot_trajectory_bundle(
                output, "stationary", stationary, origin=stationary[0]
            )
            overview_paths = plot_pose_overview_bundle(
                output,
                "overview",
                times,
                reference,
                actual,
                times,
                actual - reference,
            )
            for path in trajectory_paths + overview_paths:
                self.assertTrue(path.is_file())
                self.assertGreater(path.stat().st_size, 0)

    def test_velocity_base_and_aerial_overview_bundles(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            times = np.linspace(0.0, 2.0, 21)
            reference_velocity = np.column_stack(
                (np.ones_like(times), np.zeros_like(times), np.zeros_like(times))
            )
            actual_velocity = reference_velocity + np.column_stack(
                (
                    0.02 * np.sin(times),
                    0.01 * np.cos(times),
                    0.005 * np.sin(2.0 * times),
                )
            )
            positions = np.column_stack(
                (
                    0.05 * np.sin(times),
                    0.03 * np.cos(times),
                    1.0 + 0.01 * np.sin(2.0 * times),
                )
            )
            bundles = (
                plot_velocity_overview_bundle(
                    output,
                    "velocity_overview",
                    times,
                    reference_velocity,
                    actual_velocity,
                    positions,
                ),
                plot_base_overview_bundle(
                    output, "base_overview", times, positions
                ),
                plot_aerial_overview_bundle(
                    output,
                    "aerial_overview",
                    times,
                    reference_velocity,
                    actual_velocity,
                    times,
                    positions,
                ),
            )
            for bundle in bundles:
                self.assertEqual(len(bundle), 3)
                for path in bundle:
                    self.assertTrue(path.is_file())
                    self.assertGreater(path.stat().st_size, 0)

    def test_rejects_non_finite_trajectory(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, "finite"):
                plot_trajectory_bundle(
                    Path(directory),
                    "trajectory",
                    [[0.0, 0.0, 0.0], [1.0, np.nan, 0.0]],
                )

    def test_rejects_relative_output_directory(self):
        with self.assertRaisesRegex(ValueError, "absolute path"):
            ReportWriter("test", "relative/path")


if __name__ == "__main__":
    unittest.main()
