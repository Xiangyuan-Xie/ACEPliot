import math
import unittest

from figure8_trajectory.profile import Figure8Parameters, Figure8Profile


class Figure8ProfileTests(unittest.TestCase):
    def test_geometry_uses_physical_peak_amplitudes(self):
        profile = Figure8Profile(
            Figure8Parameters(
                period_s=4.0,
                amplitude_x_m=2.0,
                amplitude_y_m=1.0,
                max_linear_speed_m_s=10.0,
                transition_time_s=0.0,
            )
        )
        quarter = profile.sample(1.0)
        self.assertAlmostEqual(quarter.position_xy_m[0], 2.0)
        self.assertAlmostEqual(quarter.position_xy_m[1], 0.0, places=12)

    def test_speed_limit_is_validated(self):
        with self.assertRaisesRegex(ValueError, "theoretical maximum speed"):
            Figure8Profile(
                Figure8Parameters(
                    period_s=1.0,
                    amplitude_x_m=1.0,
                    amplitude_y_m=1.0,
                    max_linear_speed_m_s=1.0,
                )
            )

    def test_finite_loop_returns_origin_and_zero_velocity(self):
        profile = Figure8Profile(
            Figure8Parameters(
                period_s=2.0,
                amplitude_x_m=0.5,
                amplitude_y_m=0.25,
                max_linear_speed_m_s=4.0,
                transition_time_s=0.0,
                loops_to_run=1,
            )
        )
        sample = profile.sample(2.0)
        self.assertTrue(sample.finished)
        self.assertEqual(sample.position_xy_m, (0.0, 0.0))
        self.assertEqual(sample.velocity_xy_m_s, (0.0, 0.0))

    def test_transition_ramps_phase_speed_without_amplitude_scaling(self):
        profile = Figure8Profile(
            Figure8Parameters(
                period_s=10.0,
                amplitude_x_m=1.0,
                amplitude_y_m=0.5,
                max_linear_speed_m_s=1.0,
                transition_time_s=2.0,
            )
        )
        self.assertEqual(profile.sample(0.0).velocity_xy_m_s, (0.0, 0.0))
        self.assertTrue(math.isfinite(profile.sample(1.0).phase_rad))


if __name__ == "__main__":
    unittest.main()
