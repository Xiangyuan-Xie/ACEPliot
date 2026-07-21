import unittest

from arm_motion_benchmark.trajectory import (
    ArmSyncHandshake,
    ArmTrajectoryPlayback,
    ArmTrajectoryProfile,
)


def make_profile(loop_count=1):
    return ArmTrajectoryProfile(
        ["j1", "j2"],
        [1.0, 2.0, 2.0],
        [0.0, 0.0, 1.0, 0.0, 1.0, 1.0],
        1.0,
        loop_count,
        True,
        "gripper",
        [0.0, 0.5, 0.0],
        1.0,
    )


class ArmTrajectoryTests(unittest.TestCase):
    def test_first_duration_controls_follower_transition(self):
        playback = ArmTrajectoryPlayback(make_profile())
        playback.start(10.0, [-1.0, 0.0], 0.0)
        sample = playback.sample(10.5)
        self.assertEqual(sample.positions, (-0.5, 0.0))

    def test_finite_loop_finishes_after_transition_and_waypoints(self):
        playback = ArmTrajectoryPlayback(make_profile(loop_count=1))
        playback.start(0.0)
        self.assertFalse(playback.sample(4.9).finished)
        self.assertTrue(playback.sample(5.0).finished)

    def test_zero_loop_count_is_infinite(self):
        profile = make_profile(loop_count=0)
        self.assertFalse(profile.sample(100.0).finished)

    def test_relaxed_sim_handshake_does_not_require_arm_state(self):
        handshake = ArmSyncHandshake(require_follower_state=False)
        handshake.notify_status("ready", 1.0)
        self.assertEqual(handshake.update(1.0)[0], "ready")
        mode, allowed, started = handshake.update(1.21)
        self.assertEqual(mode, "tracking")
        self.assertTrue(allowed)
        self.assertTrue(started)

    def test_strict_handshake_requires_recent_arm_state(self):
        handshake = ArmSyncHandshake(require_follower_state=True)
        handshake.notify_status("ready", 1.0)
        self.assertEqual(handshake.update(1.0)[0], "sync_request")


if __name__ == "__main__":
    unittest.main()
