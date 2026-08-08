"""Plot Stable-Baselines3 PPO training metrics from progress.csv."""

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def read_progress(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def series(rows, key, x_key="time/total_timesteps"):
    x_values = []
    y_values = []
    for row in rows:
        try:
            x_value = float(row.get(x_key, ""))
            y_value = float(row.get(key, ""))
        except (TypeError, ValueError):
            continue
        if np.isfinite(x_value) and np.isfinite(y_value):
            x_values.append(x_value)
            y_values.append(y_value)
    return np.asarray(x_values), np.asarray(y_values)


def plot_line(axis, rows, key, label, color, *, zero_line=False):
    x, y = series(rows, key)
    if y.size:
        axis.plot(x, y, color=color, linewidth=1.8, marker="o", markersize=3, label=label)
    if zero_line:
        axis.axhline(0.0, color="#888888", linewidth=0.8, linestyle="--")
    axis.set_xlabel("PPO transitions")
    axis.grid(True, alpha=0.25)
    return x, y


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "log_dir",
        help="包含 progress.csv 的训练日志目录",
    )
    parser.add_argument(
        "--output",
        default="results/PreviewResidual/training_metrics.png",
    )
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args()

    log_dir = Path(args.log_dir)
    progress_path = log_dir / "progress.csv"
    if not progress_path.exists():
        raise FileNotFoundError(f"未找到训练日志: {progress_path}")
    rows = read_progress(progress_path)
    if not rows:
        raise ValueError(f"训练日志为空: {progress_path}")

    figure, axes = plt.subplots(3, 2, figsize=(13, 11), constrained_layout=True)
    figure.suptitle("Future-trajectory CNN residual PPO training", fontsize=15)

    plot_line(axes[0, 0], rows, "rollout/ep_rew_mean", "Mean episode reward", "#176B87", zero_line=True)
    axes[0, 0].set_title("Episode reward")
    axes[0, 0].set_ylabel("Reward")

    plot_line(axes[0, 1], rows, "rollout/ep_len_mean", "Mean episode length", "#B65A1B")
    axes[0, 1].set_title("Episode length")
    axes[0, 1].set_ylabel("Policy steps")

    policy_axis = axes[1, 0]
    value_axis = policy_axis.twinx()
    plot_line(policy_axis, rows, "train/policy_gradient_loss", "Policy loss", "#247BA0", zero_line=True)
    plot_line(value_axis, rows, "train/value_loss", "Value loss", "#D1495B")
    policy_axis.set_title("Optimization losses")
    policy_axis.set_ylabel("Policy loss", color="#247BA0")
    value_axis.set_ylabel("Value loss", color="#D1495B")
    lines = policy_axis.lines[:1] + value_axis.lines[:1]
    if lines:
        policy_axis.legend(lines, [line.get_label() for line in lines], loc="best")

    plot_line(axes[1, 1], rows, "train/approx_kl", "Approx. KL", "#6A4C93")
    plot_line(axes[1, 1], rows, "train/clip_fraction", "Clip fraction", "#2A9D8F")
    axes[1, 1].set_title("PPO update size")
    axes[1, 1].set_ylabel("Fraction / divergence")
    axes[1, 1].legend(loc="best")

    plot_line(axes[2, 0], rows, "train/explained_variance", "Explained variance", "#3A7D44", zero_line=True)
    axes[2, 0].axhline(1.0, color="#444444", linewidth=0.8, linestyle=":")
    axes[2, 0].set_title("Critic explained variance")
    axes[2, 0].set_ylabel("Explained variance")

    std_axis = axes[2, 1]
    entropy_axis = std_axis.twinx()
    plot_line(std_axis, rows, "train/std", "Action std", "#E09F3E")
    plot_line(entropy_axis, rows, "train/entropy_loss", "Entropy loss", "#7A5195")
    std_axis.set_title("Policy exploration")
    std_axis.set_ylabel("Action standard deviation", color="#E09F3E")
    entropy_axis.set_ylabel("Entropy loss", color="#7A5195")
    lines = std_axis.lines[:1] + entropy_axis.lines[:1]
    if lines:
        std_axis.legend(lines, [line.get_label() for line in lines], loc="best")

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=180)
    print(f"训练指标图已保存到 {output}")
    if args.show:
        plt.show()
    else:
        plt.close(figure)


if __name__ == "__main__":
    main()
