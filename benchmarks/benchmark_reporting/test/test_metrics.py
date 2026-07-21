import unittest

import numpy as np

from benchmark_reporting.metrics import (
    align_linear,
    base_drift_metrics,
    interpolate_quaternions_at,
    quaternion_angular_errors,
    tracking_metrics,
)


class MetricTests(unittest.TestCase):
    def test_linear_alignment_rejects_large_gaps_and_extrapolation(self):
        aligned = align_linear(
            [0.5, 1.5, 3.0],
            [[1.0], [2.0], [3.0]],
            [0.0, 1.0, 2.0],
            [[0.0], [2.0], [4.0]],
            1.1,
        )
        np.testing.assert_allclose(aligned.times_s, [0.5, 1.5])
        np.testing.assert_allclose(aligned.actual[:, 0], [1.0, 3.0])

    def test_vector_metrics(self):
        metrics = tracking_metrics(
            np.zeros((2, 3)), np.array([[1.0, 0.0, 0.0], [-1.0, 0.0, 0.0]]), ("x", "y", "z")
        )
        self.assertAlmostEqual(metrics["axes"]["x"]["rmse"], 1.0)
        self.assertAlmostEqual(metrics["horizontal"]["rmse"], 1.0)
        self.assertAlmostEqual(metrics["vector"]["rmse"], 1.0)

    def test_base_drift_uses_first_sample(self):
        metrics = base_drift_metrics(np.array([[2.0, 3.0, 4.0], [3.0, 3.0, 4.0]]))
        self.assertEqual(metrics["reference_position_enu_m"], [2.0, 3.0, 4.0])
        self.assertAlmostEqual(metrics["horizontal"]["max"], 1.0)

    def test_quaternion_error_is_sign_invariant(self):
        errors = quaternion_angular_errors(
            np.array([[1.0, 0.0, 0.0, 0.0]]),
            np.array([[-1.0, 0.0, 0.0, 0.0]]),
        )
        self.assertAlmostEqual(errors[0], 0.0)

    def test_quaternion_interpolation_handles_sign_flip(self):
        values, valid = interpolate_quaternions_at(
            [0.5],
            [0.0, 1.0],
            [[1.0, 0.0, 0.0, 0.0], [-1.0, 0.0, 0.0, 0.0]],
            1.1,
        )
        self.assertTrue(valid[0])
        np.testing.assert_allclose(values[0], [1.0, 0.0, 0.0, 0.0])


if __name__ == "__main__":
    unittest.main()
