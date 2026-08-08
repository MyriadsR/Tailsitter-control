"""Regression tests for the selected balanced relative RMS reward."""

import unittest
from types import SimpleNamespace

import numpy as np

from learning.residual_env import PreviewResidualEnv


class BalancedRelativeRmsRewardTest(unittest.TestCase):
    def test_zero_action_matches_baseline_and_action_is_bounded(self):
        env = PreviewResidualEnv()
        try:
            env.reset(seed=123)
            _, reward, _, _, info = env.step(np.zeros(3, dtype=np.float32))
            self.assertAlmostEqual(reward, 0.0, places=12)
            self.assertAlmostEqual(info["reward_tracking"], 0.0, places=12)

            _, _, _, _, info = env.step(np.ones(3, dtype=np.float32))
            self.assertLessEqual(
                float(np.max(np.abs(info["residual_acceleration_body"]))),
                0.5,
            )
        finally:
            env.close()

    def test_balanced_reward_penalizes_regression_and_is_bounded(self):
        baseline = SimpleNamespace(
            position_ref=np.array([1.0, 0.0, 0.0]),
            position=np.zeros(3),
            velocity_ref=np.zeros(3),
            velocity=np.zeros(3),
        )
        controlled = SimpleNamespace(
            position_ref=np.array([1.0, 0.0, 0.0]),
            position=np.array([0.5, 0.0, 0.0]),
            velocity_ref=np.zeros(3),
            velocity=np.zeros(3),
        )
        improvement = PreviewResidualEnv._balanced_rms_improvement(
            baseline, controlled, downside_multiplier=2.0
        )
        regression = PreviewResidualEnv._balanced_rms_improvement(
            controlled, baseline, downside_multiplier=2.0
        )
        self.assertGreater(improvement, 0.0)
        self.assertAlmostEqual(regression, -2.0 * improvement)
        self.assertLessEqual(abs(improvement), 1.02)
        self.assertLessEqual(abs(regression), 2.04)

    def test_balanced_sampling_equalizes_expected_transition_counts(self):
        env = PreviewResidualEnv()
        expected_transition_shares = np.asarray(
            [
                probability * env.NOMINAL_DURATION_SECONDS[name]
                for name, probability in zip(
                    env.trajectory_names, env.trajectory_probabilities
                )
            ]
        )
        self.assertIn("knife-edge-transition", env.trajectory_names)
        self.assertAlmostEqual(float(np.sum(env.trajectory_probabilities)), 1.0)
        self.assertLess(
            float(np.max(expected_transition_shares)
                  / np.min(expected_transition_shares)),
            1.01,
        )

    def test_mode_scale_zero_disables_environment_residual(self):
        env = PreviewResidualEnv(
            trajectory_names=("transition-to-hover",),
            trajectory_sampling="uniform",
            random_initial_error=False,
            residual_mode_scales={"transition-to-hover": 0.0},
        )
        try:
            env.reset(seed=9)
            _, _, _, _, info = env.step(np.ones(3, dtype=np.float32))
            np.testing.assert_array_equal(
                info["residual_acceleration_body"], np.zeros(3)
            )
        finally:
            env.close()


if __name__ == "__main__":
    unittest.main()
