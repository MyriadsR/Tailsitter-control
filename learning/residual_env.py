"""Gymnasium environment for the selected acceleration-residual policy."""

import numpy as np
from gymnasium import Env, spaces

from config import TailsitterConfig
from environment.closed_loop import TailsitterClosedLoop
from learning.preview_features import (
    DEFAULT_PREVIEW_POINTS,
    DEFAULT_PREVIEW_SECONDS,
    PREVIEW_CHANNELS,
    STATE_DIM,
    build_preview_observation,
)
from trajectory.test import (
    CircularTrajectory,
    CircularTransitionTrajectory,
    DifferentialThrustTurnTrajectory,
    KnifeEdgeTransitionTrajectory,
    LemniscateTrajectory,
)


class PreviewResidualEnv(Env):
    """Train at 50 Hz while retaining the original 500 Hz inner loop."""

    metadata = {"render_modes": []}
    DEFAULT_TRAJECTORIES = (
        "lemniscate",
        "circle-coordinated",
        "knife-edge-transition",
        "transition-to-forward",
        "transition-to-hover",
        "differential-turn",
    )
    NOMINAL_DURATION_SECONDS = {
        "lemniscate": 6.75,
        "circle-coordinated": 3.52,
        "knife-edge-transition": 6.65,
        "transition-to-forward": 3.8,
        "transition-to-hover": 3.8,
        "differential-turn": 3.3,
    }

    def __init__(
        self,
        residual_limit=0.5,
        policy_rate_hz=50.0,
        preview_points=DEFAULT_PREVIEW_POINTS,
        preview_seconds=DEFAULT_PREVIEW_SECONDS,
        trajectory_names=None,
        trajectory_sampling="balanced",
        random_initial_error=True,
        downside_multiplier=2.0,
        residual_feedforward_consistency=True,
        residual_filter_cutoff_hz=10.0,
        residual_mode_scales=None,
    ):
        super().__init__()
        self.residual_limit = float(residual_limit)
        self.policy_rate_hz = float(policy_rate_hz)
        self.preview_points = int(preview_points)
        self.preview_seconds = float(preview_seconds)
        self.trajectory_sampling = str(trajectory_sampling)
        self.downside_multiplier = float(downside_multiplier)
        self.residual_feedforward_consistency = bool(
            residual_feedforward_consistency
        )
        self.residual_filter_cutoff_hz = float(residual_filter_cutoff_hz)
        self.residual_mode_scales = dict(residual_mode_scales or {})
        if self.residual_limit <= 0.0:
            raise ValueError("residual_limit 必须大于 0")
        if self.policy_rate_hz <= 0.0:
            raise ValueError("policy_rate_hz 必须大于 0")
        if self.preview_points < 5:
            raise ValueError("preview_points 必须至少为 5")
        if self.preview_seconds <= 0.0:
            raise ValueError("preview_seconds 必须大于 0")
        if self.trajectory_sampling not in {"uniform", "balanced"}:
            raise ValueError("trajectory_sampling 必须是 uniform 或 balanced")
        if self.downside_multiplier < 1.0:
            raise ValueError("downside_multiplier 必须至少为 1")
        if self.residual_filter_cutoff_hz <= 0.0:
            raise ValueError("residual_filter_cutoff_hz 必须大于 0")
        if any(
            not 0.0 <= float(scale) <= 1.0
            for scale in self.residual_mode_scales.values()
        ):
            raise ValueError("飞行模式残差缩放必须位于 [0, 1]")
        self.trajectory_names = tuple(
            trajectory_names or self.DEFAULT_TRAJECTORIES
        )
        self.trajectory_probabilities = self._trajectory_probabilities()
        self.trajectory_probability_by_name = dict(
            zip(self.trajectory_names, self.trajectory_probabilities)
        )
        self.random_initial_error = bool(random_initial_error)
        self.action_space = spaces.Box(-1.0, 1.0, shape=(3,), dtype=np.float32)
        self.observation_space = spaces.Dict(
            {
                "state": spaces.Box(
                    -10.0, 10.0, shape=(STATE_DIM,), dtype=np.float32
                ),
                "preview": spaces.Box(
                    -10.0,
                    10.0,
                    shape=(PREVIEW_CHANNELS, self.preview_points),
                    dtype=np.float32,
                ),
            }
        )
        self.simulator = None
        self.baseline_simulator = None
        self.previous_action = np.zeros(3)

    def _trajectory_probabilities(self):
        if self.trajectory_sampling == "uniform":
            return np.full(
                len(self.trajectory_names), 1.0 / len(self.trajectory_names)
            )
        try:
            durations = np.asarray(
                [
                    self.NOMINAL_DURATION_SECONDS[name]
                    for name in self.trajectory_names
                ],
                dtype=float,
            )
        except KeyError as error:
            raise ValueError(
                f"balanced 采样缺少轨迹 {error.args[0]} 的时长估计"
            ) from error
        inverse_durations = 1.0 / durations
        return inverse_durations / np.sum(inverse_durations)

    def _make_trajectory(self):
        name = self.np_random.choice(
            self.trajectory_names, p=self.trajectory_probabilities
        )
        if name == "lemniscate":
            return name, LemniscateTrajectory(
                speed=self.np_random.uniform(4.5, 7.0),
                lap_time=self.np_random.uniform(5.5, 8.0),
                laps=1,
            )
        if name == "circle-coordinated":
            return name, CircularTrajectory(
                mode="coordinated",
                speed=self.np_random.uniform(4.5, 8.0),
                laps=1,
            )
        if name == "knife-edge-transition":
            return name, KnifeEdgeTransitionTrajectory(
                speed=self.np_random.uniform(4.5, 6.5),
                lap_time=self.np_random.uniform(5.8, 7.5),
                laps=1,
            )
        if name in {"transition-to-forward", "transition-to-hover"}:
            return name, CircularTransitionTrajectory(
                transition=(
                    "to_forward"
                    if name == "transition-to-forward"
                    else "to_hover"
                ),
                tangential_acceleration=self.np_random.uniform(1.8, 2.8),
            )
        if name == "differential-turn":
            return name, DifferentialThrustTurnTrajectory(
                speed=self.np_random.uniform(4.5, 7.0)
            )
        raise ValueError(f"未知训练轨迹: {name}")

    def _observation(self):
        return build_preview_observation(
            self.simulator, self.preview_points, self.preview_seconds
        )

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        config = TailsitterConfig()
        self.trajectory_name, trajectory = self._make_trajectory()
        simulator_kwargs = {
            "residual_feedforward_consistency": (
                self.residual_feedforward_consistency
            ),
            "residual_filter_cutoff_hz": self.residual_filter_cutoff_hz,
        }
        self.simulator = TailsitterClosedLoop(
            config, trajectory, **simulator_kwargs
        )
        self.baseline_simulator = TailsitterClosedLoop(
            config, trajectory, **simulator_kwargs
        )
        if self.random_initial_error:
            position_offset = self.np_random.uniform(-0.15, 0.15, 3)
            velocity_offset = self.np_random.uniform(-0.1, 0.1, 3)
            for simulator in (self.simulator, self.baseline_simulator):
                simulator.state[0:3] += position_offset
                simulator.state[3:6] += velocity_offset
        self.control_decimation = max(
            1, int(round(config.control_rate_hz / self.policy_rate_hz))
        )
        self.previous_action = np.zeros(3)
        self.episode_inner_steps = 0
        self.episode_policy_position_squared_error = 0.0
        self.episode_baseline_position_squared_error = 0.0
        return self._observation(), {"trajectory": self.trajectory_name}

    def step(self, action):
        normalized_action = np.clip(np.asarray(action, dtype=float), -1.0, 1.0)
        mode_scale = float(
            self.residual_mode_scales.get(self.trajectory_name, 1.0)
        )
        residual = mode_scale * self.residual_limit * normalized_action
        relative_reward_sum = 0.0
        sample = None
        failed = False
        inner_steps = 0
        for _ in range(self.control_decimation):
            if self.simulator.done:
                break
            sample = self.simulator.step(residual)
            baseline_sample = self.baseline_simulator.step()
            inner_steps += 1
            controlled_position_error = sample.position_ref - sample.position
            baseline_position_error = (
                baseline_sample.position_ref - baseline_sample.position
            )
            self.episode_policy_position_squared_error += float(
                np.dot(controlled_position_error, controlled_position_error)
            )
            self.episode_baseline_position_squared_error += float(
                np.dot(baseline_position_error, baseline_position_error)
            )
            self.episode_inner_steps += 1
            relative_reward_sum += self._balanced_rms_improvement(
                baseline_sample, sample, self.downside_multiplier
            )
            position_error = np.linalg.norm(controlled_position_error)
            failed = (
                not np.all(np.isfinite(self.simulator.state))
                or position_error > 15.0
            )
            if failed:
                break

        action_delta = normalized_action - self.previous_action
        self.previous_action = normalized_action
        action_cost = 0.002 * np.dot(normalized_action, normalized_action)
        rate_cost = 0.001 * np.dot(action_delta, action_delta)
        tracking_reward = relative_reward_sum / max(inner_steps, 1)
        reward = tracking_reward - action_cost - rate_cost
        if failed:
            reward -= 100.0
        terminated = bool(failed)
        truncated = bool(self.simulator.done and not failed)
        info = {
            "trajectory": self.trajectory_name,
            "trajectory_sampling_probability": float(
                self.trajectory_probability_by_name[self.trajectory_name]
            ),
            "residual_acceleration_body": residual.astype(np.float32),
            "reward_tracking": float(tracking_reward),
            "reward_action_penalty": float(action_cost),
            "reward_rate_penalty": float(rate_cost),
        }
        if sample is not None:
            info["position_error"] = float(
                np.linalg.norm(sample.position_ref - sample.position)
            )
        if terminated or truncated:
            sample_count = max(self.episode_inner_steps, 1)
            policy_rms = np.sqrt(
                self.episode_policy_position_squared_error / sample_count
            )
            baseline_rms = np.sqrt(
                self.episode_baseline_position_squared_error / sample_count
            )
            info.update(
                {
                    "episode_policy_rms": float(policy_rms),
                    "episode_baseline_rms": float(baseline_rms),
                    "episode_rms_improvement_percent": float(
                        100.0
                        * (baseline_rms - policy_rms)
                        / max(baseline_rms, 1e-12)
                    ),
                }
            )
        observation = self._observation() if not self.simulator.done else {
            key: np.zeros(space.shape, dtype=np.float32)
            for key, space in self.observation_space.spaces.items()
        }
        return observation, float(reward), terminated, truncated, info

    @staticmethod
    def _balanced_rms_improvement(
        baseline_sample,
        controlled_sample,
        downside_multiplier=2.0,
    ):
        baseline_position_error = (
            baseline_sample.position_ref - baseline_sample.position
        )
        controlled_position_error = (
            controlled_sample.position_ref - controlled_sample.position
        )
        baseline_velocity_error = (
            baseline_sample.velocity_ref - baseline_sample.velocity
        )
        controlled_velocity_error = (
            controlled_sample.velocity_ref - controlled_sample.velocity
        )

        def normalized_improvement(baseline_error, controlled_error, floor):
            baseline_squared = float(np.dot(baseline_error, baseline_error))
            controlled_squared = float(
                np.dot(controlled_error, controlled_error)
            )
            value = (baseline_squared - controlled_squared) / (
                baseline_squared + controlled_squared + floor
            )
            value = float(np.clip(value, -1.0, 1.0))
            return value if value >= 0.0 else downside_multiplier * value

        position_improvement = normalized_improvement(
            baseline_position_error, controlled_position_error, floor=0.01
        )
        velocity_improvement = normalized_improvement(
            baseline_velocity_error, controlled_velocity_error, floor=0.25
        )
        return float(position_improvement + 0.02 * velocity_improvement)

    def render(self):
        return None

    def close(self):
        self.simulator = None
        self.baseline_simulator = None
