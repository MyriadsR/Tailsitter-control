"""Train the DATT-style future-trajectory acceleration-residual policy."""

import argparse
import json
from pathlib import Path

import numpy as np

from learning.preview_features import (
    DEFAULT_PREVIEW_POINTS,
    DEFAULT_PREVIEW_SECONDS,
)


def parse_mode_scale(value):
    try:
        name, scale_text = value.rsplit("=", 1)
        scale = float(scale_text)
    except ValueError as error:
        raise argparse.ArgumentTypeError("格式必须为 TRAJECTORY=SCALE") from error
    if not name or not 0.0 <= scale <= 1.0:
        raise argparse.ArgumentTypeError("SCALE 必须位于 [0, 1]")
    return name, scale


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timesteps", type=int, default=100_000)
    parser.add_argument("--n-envs", type=int, default=4)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--learning-rate", type=float, default=3e-4)
    parser.add_argument(
        "--output", default="saved_models/preview_residual_consistent_retrain"
    )
    parser.add_argument(
        "--resume",
        help="从已有 PPO 模型续训（路径可带或不带 .zip）",
    )
    parser.add_argument(
        "--log-dir",
        help="训练 CSV、Monitor 和 checkpoint 目录；默认按模型名创建",
    )
    parser.add_argument(
        "--checkpoint-freq",
        type=int,
        default=25_000,
        help="每多少个总 PPO transition 保存 checkpoint；设为 0 禁用",
    )
    parser.add_argument("--residual-limit", type=float, default=0.5)
    parser.add_argument(
        "--deployment-residual-limit",
        type=float,
        default=0.5,
        help="固定验证和最终部署使用的残差限幅 (m/s^2)",
    )
    parser.add_argument(
        "--eval-freq",
        type=int,
        default=25_000,
        help="每多少个总 PPO transition 运行固定验证集；设为 0 禁用",
    )
    parser.add_argument(
        "--validation-worst-weight",
        type=float,
        default=0.5,
        help="固定验证选择分数中最差负改善的惩罚权重",
    )
    parser.add_argument("--policy-rate", type=float, default=50.0)
    parser.add_argument(
        "--residual-feedforward-consistency",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="使残差同步修正姿态和角速度前馈",
    )
    parser.add_argument(
        "--residual-filter-cutoff",
        type=float,
        default=10.0,
        help="一致性残差的一阶低通截止频率 (Hz)",
    )
    parser.add_argument(
        "--mode-scale",
        action="append",
        type=parse_mode_scale,
        default=[],
        metavar="TRAJECTORY=SCALE",
        help="按轨迹/飞行模式缩放残差，可重复指定",
    )
    parser.add_argument("--preview-points", type=int, default=DEFAULT_PREVIEW_POINTS)
    parser.add_argument("--preview-seconds", type=float, default=DEFAULT_PREVIEW_SECONDS)
    parser.add_argument(
        "--trajectories",
        nargs="+",
        help="覆盖训练轨迹集合",
    )
    parser.add_argument(
        "--trajectory-sampling",
        choices=("uniform", "balanced"),
        default="balanced",
        help="balanced 按轨迹时长倒数采样，使 transition 贡献近似等权",
    )
    parser.add_argument(
        "--downside-multiplier",
        type=float,
        default=2.0,
        help="balanced-relative-rms 对性能退化项的额外惩罚倍数",
    )
    args = parser.parse_args()
    residual_mode_scales = dict(args.mode_scale)

    import torch
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import CallbackList, CheckpointCallback
    from stable_baselines3.common.env_util import make_vec_env
    from stable_baselines3.common.logger import configure
    from stable_baselines3.common.utils import FloatSchedule

    from learning.preview_extractor import PreviewFeaturesExtractor
    from learning.residual_env import PreviewResidualEnv
    from learning.validation_callback import FixedSuiteValidationCallback

    torch.set_num_threads(1)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    log_dir = (
        Path(args.log_dir)
        if args.log_dir
        else Path("training_logs") / output.name
    )
    monitor_dir = log_dir / "monitor"
    checkpoint_dir = log_dir / "checkpoints"
    monitor_dir.mkdir(parents=True, exist_ok=True)
    env_kwargs = {
        "residual_limit": args.residual_limit,
        "policy_rate_hz": args.policy_rate,
        "preview_points": args.preview_points,
        "preview_seconds": args.preview_seconds,
        "trajectory_names": args.trajectories,
        "trajectory_sampling": args.trajectory_sampling,
        "downside_multiplier": args.downside_multiplier,
        "residual_feedforward_consistency": (
            args.residual_feedforward_consistency
        ),
        "residual_filter_cutoff_hz": args.residual_filter_cutoff,
        "residual_mode_scales": residual_mode_scales,
    }
    env = make_vec_env(
        PreviewResidualEnv,
        n_envs=args.n_envs,
        seed=args.seed,
        env_kwargs=env_kwargs,
        monitor_dir=str(monitor_dir),
        monitor_kwargs={
            "info_keywords": (
                "trajectory",
                "episode_policy_rms",
                "episode_baseline_rms",
                "episode_rms_improvement_percent",
            )
        },
    )
    policy_kwargs = {
        "features_extractor_class": PreviewFeaturesExtractor,
        "features_extractor_kwargs": {"features_dim": 64},
        "activation_fn": torch.nn.ReLU,
        "net_arch": {"pi": [64, 64], "vf": [64, 64]},
        "log_std_init": -1.5,
    }
    if args.resume:
        model = PPO.load(args.resume, env=env, device="cpu")
        model.learning_rate = args.learning_rate
        model.lr_schedule = FloatSchedule(args.learning_rate)
        print(
            f"从 {args.resume} 续训，已有 {model.num_timesteps} transitions，"
            f"学习率设为 {args.learning_rate:g}"
        )
    else:
        model = PPO(
            "MultiInputPolicy",
            env,
            policy_kwargs=policy_kwargs,
            learning_rate=args.learning_rate,
            n_steps=512,
            batch_size=256,
            gamma=0.99,
            gae_lambda=0.95,
            ent_coef=0.0,
            seed=args.seed,
            device="cpu",
            verbose=1,
        )
    model.set_logger(configure(str(log_dir), ["stdout", "csv"]))
    callbacks = []
    if args.checkpoint_freq > 0:
        checkpoint_dir.mkdir(parents=True, exist_ok=True)
        callbacks.append(CheckpointCallback(
            save_freq=max(args.checkpoint_freq // args.n_envs, 1),
            save_path=str(checkpoint_dir),
            name_prefix=output.name,
        ))
    if args.eval_freq > 0:
        callbacks.append(FixedSuiteValidationCallback(
            log_dir=log_dir,
            eval_freq=args.eval_freq,
            residual_limit=args.deployment_residual_limit,
            policy_rate_hz=args.policy_rate,
            preview_points=args.preview_points,
            preview_seconds=args.preview_seconds,
            worst_case_weight=args.validation_worst_weight,
            residual_feedforward_consistency=(
                args.residual_feedforward_consistency
            ),
            residual_filter_cutoff_hz=args.residual_filter_cutoff,
            residual_mode_scales=residual_mode_scales,
        ))
    callback = CallbackList(callbacks) if callbacks else None
    model.learn(
        total_timesteps=args.timesteps,
        callback=callback,
        reset_num_timesteps=not bool(args.resume),
    )
    model.save(output)
    metadata = {
        "algorithm": "PPO",
        "timesteps": int(model.num_timesteps),
        "additional_timesteps_requested": args.timesteps,
        "resume_from": args.resume,
        "seed": args.seed,
        "learning_rate": args.learning_rate,
        "residual_limit": args.deployment_residual_limit,
        "training_residual_limit": args.residual_limit,
        "policy_rate_hz": args.policy_rate,
        "preview_points": args.preview_points,
        "preview_seconds": args.preview_seconds,
        "preview_channels": 11,
        "state_dim": 16,
        "training_trajectories": args.trajectories or "default",
        "trajectory_sampling": args.trajectory_sampling,
        "downside_multiplier": args.downside_multiplier,
        "action": "body-frame acceleration residual [m/s^2]",
        "reward_mode": "balanced-relative-rms",
        "log_dir": str(log_dir),
        "eval_freq": args.eval_freq,
        "validation_worst_weight": args.validation_worst_weight,
        "residual_feedforward_consistency": (
            args.residual_feedforward_consistency
        ),
        "residual_filter_cutoff_hz": args.residual_filter_cutoff,
        "residual_mode_scales": residual_mode_scales,
    }
    output.with_suffix(".json").write_text(
        json.dumps(metadata, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    env.close()
    print(f"模型已保存到 {output.with_suffix('.zip')}")
    print(f"训练指标已保存到 {log_dir / 'progress.csv'}")


if __name__ == "__main__":
    main()
