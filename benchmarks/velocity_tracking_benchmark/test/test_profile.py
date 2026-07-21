import unittest

from velocity_tracking_benchmark.profile import PoseVelocityEstimator, VelocityProfile


class VelocityProfileTests(unittest.TestCase):
    def make_profile(self, loop=False):
        return VelocityProfile(
            [1.0, 1.0],
            [1.0, 0.0],
            [0.0, 1.0],
            [0.0, 0.0],
            [0.0, 0.0],
            loop,
            1.0,
            1.0,
        )

    def test_finite_profile_finishes_at_zero(self):
        command = self.make_profile().sample(2.0)
        self.assertTrue(command.finished)
        self.assertEqual(command.linear_enu_m_s, (0.0, 0.0, 0.0))

    def test_loop_wraps(self):
        command = self.make_profile(loop=True).sample(2.5)
        self.assertFalse(command.finished)
        self.assertEqual(command.linear_enu_m_s, (1.0, 0.0, 0.0))

    def test_velocity_limit_is_checked(self):
        with self.assertRaisesRegex(ValueError, "max_linear_speed"):
            VelocityProfile([1.0], [2.0], [0.0], [0.0], [0.0], False, 1.0, 1.0)

    def test_estimator_resets_after_large_dt(self):
        estimator = PoseVelocityEstimator(0.01, 0.5, 1.0)
        self.assertIsNone(estimator.update(0.0, (0.0, 0.0, 0.0)))
        self.assertEqual(estimator.update(0.1, (0.1, 0.0, 0.0)), (1.0, 0.0, 0.0))
        self.assertIsNone(estimator.update(1.0, (1.0, 0.0, 0.0)))


if __name__ == "__main__":
    unittest.main()
