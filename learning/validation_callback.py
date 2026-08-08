"""Fixed-suite validation for the future-trajectory residual policy."""

import csv
import json
from pathlib import Path

import numpy as np
from stable_baselines3.common.callbacks import BaseCallback

from config import TailsitterConfig
from environment.closed_loop import TailsitterClosedLoop
from learning.preview_features import build_preview_observation
from learning.residual_policy import trajectory_mode_name
from trajectory.factory import make_trajectory


VALIDATION_CASES = (
    ("lemniscate", {"laps": 1}),
    ("circle-coordinated", {"laps": 1}),
    ("knife-edge-transition", {"laps": 1}),
    ("transition-to-forward", {}),
    ("transition-to-hover", {}),
    ("differential-turn", {}),
)


class _LiveModelProvider:
    """Run the model owned by the callback without saving and reloading it."""

    def __init__(
        self,
        model,
        residual_limit,
        policy_rate_hz,
        preview_points,
        preview_seconds,
        residual_mode_scales=None,
    ):
        self.model = model
        self.residual_limit = float(residual_limit)
        self.policy_rate_hz = float(policy_rate_hz)
        self.preview_points = int(preview_points)
        self.preview_seconds = float(preview_seconds)
        self.residual_mode_scales = dict(residual_mode_scales or {})
        self.reset()

    def reset(self):
        self._last_update_step = None
        self._residual = np.zeros(3)

    def __call__(self, simulator):
        update_interval = max(
            1,
            int(round(simulator.cfg.control_rate_hz / self.policy_rate_hz)),
        )
        if (
            self._last_update_step is None
            or simulator.step_index - self._last_update_step >= update_interval
        ):
            observation = build_preview_observation(
                simulator,
                self.preview_points,
                self.preview_seconds,
            )
            action, _ = self.model.predict(observation, deterministic=True)
            action = np.clip(np.asarray(action, dtype=float), -1.0, 1.0)
            mode_scale = float(
                self.residual_mode_scales.get(
                    trajectory_mode_name(simulator.trajectory), 1.0
                )
            )
            self._residual = mode_scale * self.residual_limit * action
            self._last_update_step = simulator.step_index
        return self._residual.copy()


def _position_rms(name, trajectory_kwargs, provider=None):
    simulator = TailsitterClosedLoop(
        TailsitterConfig(),
        make_trajectory(name, **trajectory_kwargs),
        residual_feedforward_consistency=getattr(
            provider, "residual_feedforward_consistency", False
        ),
        residual_filter_cutoff_hz=getattr(
            provider, "residual_filter_cutoff_hz", 10.0
        ),
    )
    squared_errors = []
    if provider is not None:
        provider.reset()
    while not simulator.done:
        residual = None if provider is None else provider(simulator)
        sample = simulator.step(residual)
        squared_errors.append(
            float(np.dot(sample.position_ref - sample.position,
                         sample.position_ref - sample.position))
        )
    return float(np.sqrt(np.mean(squared_errors)))


class FixedSuiteValidationCallback(BaseCallback):
    """Evaluate deterministic tracking on fixed trajectories during PPO training."""

    def __init__(
        self,
        log_dir,
        eval_freq=25_000,
        residual_limit=0.5,
        policy_rate_hz=50.0,
        preview_points=12,
        preview_seconds=0.6,
        worst_case_weight=0.5,
        residual_feedforward_consistency=True,
        residual_filter_cutoff_hz=10.0,
        residual_mode_scales=None,
        verbose=1,
    ):
        super().__init__(verbose=verbose)
        self.log_dir = Path(log_dir)
        self.eval_freq = int(eval_freq)
        self.residual_limit = float(residual_limit)
        self.policy_rate_hz = float(policy_rate_hz)
        self.preview_points = int(preview_points)
        self.preview_seconds = float(preview_seconds)
        self.worst_case_weight = float(worst_case_weight)
        self.residual_feedforward_consistency = bool(
            residual_feedforward_consistency
        )
        self.residual_filter_cutoff_hz = float(residual_filter_cutoff_hz)
        self.residual_mode_scales = dict(residual_mode_scales or {})
        self.validation_path = self.log_dir / "validation.csv"
        self.best_model_path = self.log_dir / "best_model"
        self.best_selection_score = -np.inf
        self.baseline_rms = {}
        self.next_eval_step = None
        case_columns = [name.replace("-", "_") for name, _ in VALIDATION_CASES]
        self.fieldnames = [
            "total_timesteps",
            "mean_improvement_percent",
            "worst_improvement_percent",
            "selection_score",
        ]
        self.fieldnames += [f"{name}_baseline_rms" for name in case_columns]
        self.fieldnames += [f"{name}_policy_rms" for name in case_columns]
        self.fieldnames += [f"{name}_improvement_percent" for name in case_columns]

    def _on_training_start(self):
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.baseline_rms = {
            name: _position_rms(name, kwargs)
            for name, kwargs in VALIDATION_CASES
        }
        if not self.validation_path.exists():
            with self.validation_path.open("w", newline="", encoding="utf-8") as stream:
                csv.DictWriter(stream, fieldnames=self.fieldnames).writeheader()
        self._evaluate_and_record()
        self.next_eval_step = (
            (self.num_timesteps // self.eval_freq) + 1
        ) * self.eval_freq

    def _on_step(self):
        if self.num_timesteps >= self.next_eval_step:
            self._evaluate_and_record()
            while self.next_eval_step <= self.num_timesteps:
                self.next_eval_step += self.eval_freq
        return True

    def _evaluate_and_record(self):
        provider = _LiveModelProvider(
            self.model,
            self.residual_limit,
            self.policy_rate_hz,
            self.preview_points,
            self.preview_seconds,
            self.residual_mode_scales,
        )
        provider.residual_feedforward_consistency = (
            self.residual_feedforward_consistency
        )
        provider.residual_filter_cutoff_hz = self.residual_filter_cutoff_hz
        improvements = []
        row = {"total_timesteps": int(self.num_timesteps)}
        for name, kwargs in VALIDATION_CASES:
            column = name.replace("-", "_")
            baseline = self.baseline_rms[name]
            policy = _position_rms(name, kwargs, provider)
            improvement = 100.0 * (baseline - policy) / baseline
            improvements.append(improvement)
            row[f"{column}_baseline_rms"] = baseline
            row[f"{column}_policy_rms"] = policy
            row[f"{column}_improvement_percent"] = improvement
        mean_improvement = float(np.mean(improvements))
        worst_improvement = float(np.min(improvements))
        selection_score = mean_improvement + self.worst_case_weight * min(
            worst_improvement, 0.0
        )
        row["mean_improvement_percent"] = mean_improvement
        row["worst_improvement_percent"] = worst_improvement
        row["selection_score"] = selection_score
        with self.validation_path.open("a", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=self.fieldnames)
            writer.writerow(row)

        if selection_score > self.best_selection_score:
            self.best_selection_score = selection_score
            self.model.save(self.best_model_path)
            metadata = {
                "residual_limit": self.residual_limit,
                "policy_rate_hz": self.policy_rate_hz,
                "preview_points": self.preview_points,
                "preview_seconds": self.preview_seconds,
                "validation_step": int(self.num_timesteps),
                "mean_improvement_percent": mean_improvement,
                "worst_improvement_percent": worst_improvement,
                "selection_score": selection_score,
                "worst_case_weight": self.worst_case_weight,
                "residual_feedforward_consistency": (
                    self.residual_feedforward_consistency
                ),
                "residual_filter_cutoff_hz": self.residual_filter_cutoff_hz,
                "residual_mode_scales": self.residual_mode_scales,
            }
            self.best_model_path.with_suffix(".json").write_text(
                json.dumps(metadata, indent=2, ensure_ascii=False) + "\n",
                encoding="utf-8",
            )
        if self.verbose:
            print(
                f"固定验证集 @ {self.num_timesteps}: "
                f"平均 RMS 改善 {mean_improvement:+.2f}%, "
                f"最差 {worst_improvement:+.2f}%, "
                f"选择分数 {selection_score:+.2f} "
                f"(最佳 {self.best_selection_score:+.2f})"
            )
