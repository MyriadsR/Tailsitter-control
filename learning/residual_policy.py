"""Inference adapter for a trained Stable-Baselines3 preview policy."""

import json
from pathlib import Path

import numpy as np

from learning.preview_features import build_preview_observation


def trajectory_mode_name(trajectory):
    """Return the deployment gate key for a known reference trajectory."""
    transition = getattr(trajectory, "transition", None)
    if transition in {"to_forward", "to_hover"}:
        return f"transition-{transition.replace('_', '-')}"
    mode = getattr(trajectory, "mode", None)
    if mode == "knife_edge":
        return "circular-knife-edge"
    if mode == "coordinated":
        return "circle-coordinated"
    class_name = type(trajectory).__name__
    return {
        "DifferentialThrustTurnTrajectory": "differential-turn",
        "KnifeEdgeTransitionTrajectory": "knife-edge-transition",
        "LemniscateTrajectory": "lemniscate",
    }.get(class_name, class_name)


class PreviewResidualProvider:
    def __init__(
        self,
        model_path,
        policy_rate_hz=None,
        residual_limit=None,
        device="cpu",
        allow_circular_knife_edge=False,
        residual_feedforward_consistency=None,
        residual_filter_cutoff_hz=None,
        residual_mode_scales=None,
    ):
        # NumPy is intentionally imported before SB3/PyTorch. This also avoids an
        # old system libstdc++ taking precedence in some Conda installations.
        from stable_baselines3 import PPO

        path = Path(model_path)
        self.model = PPO.load(path, device=device)
        metadata_path = path.with_suffix(".json")
        metadata = {}
        if metadata_path.exists():
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        self.policy_rate_hz = float(
            policy_rate_hz
            if policy_rate_hz is not None
            else metadata.get("policy_rate_hz", 50.0)
        )
        self.residual_limit = float(
            residual_limit
            if residual_limit is not None
            else metadata.get("residual_limit", 2.0)
        )
        self.preview_points = int(metadata.get("preview_points", 12))
        self.preview_seconds = float(metadata.get("preview_seconds", 0.6))
        self.residual_feedforward_consistency = bool(
            residual_feedforward_consistency
            if residual_feedforward_consistency is not None
            else metadata.get("residual_feedforward_consistency", False)
        )
        self.residual_filter_cutoff_hz = float(
            residual_filter_cutoff_hz
            if residual_filter_cutoff_hz is not None
            else metadata.get("residual_filter_cutoff_hz", 10.0)
        )
        self.residual_mode_scales = dict(
            residual_mode_scales
            if residual_mode_scales is not None
            else metadata.get("residual_mode_scales", {})
        )
        self.allow_circular_knife_edge = bool(allow_circular_knife_edge)
        if (
            self.policy_rate_hz <= 0.0
            or self.residual_limit <= 0.0
            or self.residual_filter_cutoff_hz <= 0.0
        ):
            raise ValueError("策略频率、残差限幅和滤波截止频率必须大于 0")
        if any(
            not 0.0 <= float(scale) <= 1.0
            for scale in self.residual_mode_scales.values()
        ):
            raise ValueError("飞行模式残差缩放必须位于 [0, 1]")
        self.reset()

    def reset(self):
        self._last_update_step = None
        self._residual = np.zeros(3)

    def __call__(self, simulator):
        if (
            getattr(simulator.trajectory, "mode", None) == "knife_edge"
            and not self.allow_circular_knife_edge
        ):
            self._residual.fill(0.0)
            return self._residual.copy()
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
