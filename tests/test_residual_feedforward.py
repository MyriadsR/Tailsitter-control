"""Tests for consistent residual attitude and angular-rate feedforward."""

import unittest

import numpy as np

from config import TailsitterConfig
from environment.closed_loop import TailsitterClosedLoop
from learning.residual_policy import trajectory_mode_name
from trajectory.factory import make_trajectory


class ResidualFeedforwardConsistencyTest(unittest.TestCase):
    def _simulator(self, consistent):
        return TailsitterClosedLoop(
            TailsitterConfig(),
            make_trajectory("circle-coordinated", laps=1),
            residual_feedforward_consistency=consistent,
            residual_filter_cutoff_hz=10.0,
        )

    def test_zero_residual_preserves_legacy_closed_loop(self):
        legacy = self._simulator(False)
        consistent = self._simulator(True)
        for _ in range(50):
            legacy_sample = legacy.step()
            consistent_sample = consistent.step()
            np.testing.assert_allclose(consistent.state, legacy.state, atol=1e-12)
            np.testing.assert_allclose(
                consistent_sample.angular_velocity_ref,
                legacy_sample.angular_velocity_ref,
                atol=1e-12,
            )

    def test_residual_updates_angular_rate_feedforward(self):
        legacy = self._simulator(False)
        consistent = self._simulator(True)
        residual = np.array([0.3, -0.2, 0.1])
        legacy_sample = legacy.step(residual)
        consistent_sample = consistent.step(residual)

        self.assertGreater(
            np.linalg.norm(
                consistent_sample.angular_velocity_ref
                - legacy_sample.angular_velocity_ref
            ),
            1e-6,
        )
        self.assertTrue(
            np.all(np.isfinite(consistent_sample.residual_jerk_inertial))
        )
        self.assertLessEqual(
            np.linalg.norm(
                consistent_sample.applied_residual_acceleration_inertial
            ),
            np.linalg.norm(residual) + 1e-12,
        )

    def test_known_trajectory_mode_names(self):
        self.assertEqual(
            trajectory_mode_name(make_trajectory("transition-to-hover")),
            "transition-to-hover",
        )
        self.assertEqual(
            trajectory_mode_name(make_trajectory("differential-turn")),
            "differential-turn",
        )


if __name__ == "__main__":
    unittest.main()
