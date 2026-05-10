import importlib.util
import pathlib
import sys
import unittest

import numpy as np


REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]
MODULE_PATH = REPO_ROOT / "tools" / "ulog" / "utils.py"


def load_module():
    spec = importlib.util.spec_from_file_location("ulog_utils", MODULE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load module from {MODULE_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class UlogUtilsTests(unittest.TestCase):
    def test_ned_to_enu_positions_swaps_xy_and_flips_z(self):
        utils = load_module()

        positions = utils.ned_to_enu_positions(
            np.array([1.0, 2.0]),
            np.array([3.0, 4.0]),
            np.array([-5.0, 6.0]),
        )

        np.testing.assert_allclose(positions, np.array([[3.0, 1.0, 5.0], [4.0, 2.0, -6.0]]))

    def test_quaternion_rotation_rotates_vectors(self):
        utils = load_module()

        quats_wxyz = np.array(
            [
                [1.0, 0.0, 0.0, 0.0],
                [np.sqrt(0.5), 0.0, 0.0, np.sqrt(0.5)],
            ],
            dtype=np.float64,
        )

        rotated = utils.rotate_vectors_by_quat_wxyz(quats_wxyz, np.array([1.0, 0.0, 0.0]))

        np.testing.assert_allclose(rotated[0], np.array([1.0, 0.0, 0.0]), atol=1e-12)
        np.testing.assert_allclose(rotated[1], np.array([0.0, 1.0, 0.0]), atol=1e-12)

    def test_finite_rows_handles_1d_and_2d_inputs(self):
        utils = load_module()

        mask = utils.finite_rows(
            np.array([0.0, 1.0, np.nan, 3.0]),
            np.array([[0.0, 1.0], [2.0, 3.0], [4.0, 5.0], [6.0, np.inf]]),
        )

        np.testing.assert_array_equal(mask, np.array([True, True, False, False]))

    def test_interpolate_columns_ignores_non_finite_source_values(self):
        utils = load_module()

        interpolated = utils.interpolate_columns(
            np.array([0.0, 1.0, 2.0]),
            np.array([[0.0, 0.0], [np.nan, 10.0], [2.0, 20.0]]),
            np.array([0.5, 1.5]),
        )

        np.testing.assert_allclose(interpolated, np.array([[0.5, 5.0], [1.5, 15.0]]))


if __name__ == "__main__":
    unittest.main()
