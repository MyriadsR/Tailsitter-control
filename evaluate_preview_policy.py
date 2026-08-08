"""A/B evaluation of the baseline and learned preview residual controller."""

import argparse
import json
from pathlib import Path

import numpy as np

from config import TailsitterConfig
from learning.residual_policy import PreviewResidualProvider
from main_sim import compute_metrics, run_simulation, save_log
from trajectory.factory import make_trajectory


def parse_mode_scale(value):
    try:
        name, scale_text = value.rsplit("=", 1)
        scale = float(scale_text)
    except ValueError as error:
        raise argparse.ArgumentTypeError("格式必须为 TRAJECTORY=SCALE") from error
    if not name or not 0.0 <= scale <= 1.0:
        raise argparse.ArgumentTypeError("SCALE 必须位于 [0, 1]")
    return name, scale


DEFAULT_CASES = (
    ("lemniscate", {"laps": 1}),
    ("circle-coordinated", {"laps": 1}),
    ("knife-edge-transition", {"laps": 1}),
    ("circular-knife-edge", {"laps": 4}),
    ("transition-to-forward", {}),
    ("transition-to-hover", {}),
    ("differential-turn", {}),
)


def evaluate_case(name, kwargs, provider, output_dir):
    results = {}
    for controller_name, residual_provider in (
        ("baseline", None),
        ("preview", provider),
    ):
        if residual_provider is not None:
            residual_provider.reset()
        config = TailsitterConfig()
        trajectory = make_trajectory(name, **kwargs)
        simulator, logs = run_simulation(config, trajectory, residual_provider)
        metrics = compute_metrics(config, simulator, logs)
        results[controller_name] = metrics
        save_log(output_dir / f"{name}_{controller_name}.npz", logs)
    baseline = results["baseline"]["rms_position_error"]
    preview = results["preview"]["rms_position_error"]
    results["rms_improvement_percent"] = 100.0 * (baseline - preview) / baseline
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("model", help="PPO 模型路径（带或不带 .zip）")
    parser.add_argument("--output-dir", default="preview_results")
    parser.add_argument(
        "--residual-limit",
        type=float,
        help="覆盖模型元数据中的部署残差限幅 (m/s^2)",
    )
    parser.add_argument(
        "--allow-circular-knife-edge",
        action="store_true",
        help="仅离线诊断：绕过已知退化的全程刀锋圆周安全门",
    )
    parser.add_argument(
        "--residual-feedforward-consistency",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="覆盖模型元数据中的姿态/角速度前馈一致性设置",
    )
    parser.add_argument(
        "--residual-filter-cutoff",
        type=float,
        help="覆盖模型元数据中的一致性残差低通截止频率 (Hz)",
    )
    parser.add_argument(
        "--mode-scale",
        action="append",
        type=parse_mode_scale,
        default=[],
        metavar="TRAJECTORY=SCALE",
        help="覆盖指定轨迹/飞行模式的残差缩放，可重复指定",
    )
    parser.add_argument(
        "--trajectories",
        nargs="+",
        help="只评估指定轨迹；省略时使用七个代表性工况",
    )
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    provider = PreviewResidualProvider(
        args.model,
        residual_limit=args.residual_limit,
        allow_circular_knife_edge=args.allow_circular_knife_edge,
        residual_feedforward_consistency=(
            args.residual_feedforward_consistency
        ),
        residual_filter_cutoff_hz=args.residual_filter_cutoff,
        residual_mode_scales=(dict(args.mode_scale) if args.mode_scale else None),
    )
    if args.trajectories:
        cases = tuple((name, {}) for name in args.trajectories)
    else:
        cases = DEFAULT_CASES
    summary = {}
    for name, kwargs in cases:
        summary[name] = evaluate_case(name, kwargs, provider, output_dir)
        case = summary[name]
        print(
            f"{name}: baseline={case['baseline']['rms_position_error']:.3f} m, "
            f"preview={case['preview']['rms_position_error']:.3f} m, "
            f"变化={case['rms_improvement_percent']:+.1f}%"
        )
    summary_path = output_dir / "summary.json"
    summary_path.write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(f"汇总已保存到 {summary_path}")


if __name__ == "__main__":
    main()
