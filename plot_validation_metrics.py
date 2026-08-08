"""Plot fixed-suite tracking validation recorded during PPO training."""

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


CASE_LABELS = {
    "lemniscate": "Lemniscate",
    "circle_coordinated": "Coordinated circle",
    "knife_edge_transition": "Knife-edge transition",
    "transition_to_forward": "Transition to forward",
    "transition_to_hover": "Transition to hover",
    "differential_turn": "Differential turn",
}


def _read_rows(path):
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log_dir", help="包含 validation.csv 的训练日志目录")
    parser.add_argument(
        "--output",
        default="results/PreviewResidual/validation_metrics.png",
    )
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args()

    validation_path = Path(args.log_dir) / "validation.csv"
    if not validation_path.exists():
        raise FileNotFoundError(f"未找到固定验证日志: {validation_path}")
    rows = _read_rows(validation_path)
    if not rows:
        raise ValueError(f"固定验证日志为空: {validation_path}")

    steps = np.asarray([float(row["total_timesteps"]) for row in rows])
    mean_improvement = np.asarray(
        [float(row["mean_improvement_percent"]) for row in rows]
    )
    has_selection_score = all(row.get("selection_score") for row in rows)
    selection_score = (
        np.asarray([float(row["selection_score"]) for row in rows])
        if has_selection_score
        else mean_improvement
    )
    figure, axes = plt.subplots(2, 1, figsize=(12, 8), constrained_layout=True)
    figure.suptitle("Fixed-suite validation during PPO training", fontsize=15)

    axes[0].plot(
        steps,
        mean_improvement,
        color="#176B87",
        marker="o",
        linewidth=2.2,
        label="Six-case mean",
    )
    if has_selection_score:
        axes[0].plot(
            steps,
            selection_score,
            color="#6A4C93",
            marker="s",
            linewidth=1.6,
            label="Worst-case selection score",
        )
    best_index = int(np.argmax(selection_score))
    axes[0].scatter(
        steps[best_index],
        mean_improvement[best_index],
        color="#C23B22",
        marker="*",
        s=140,
        zorder=3,
        label="Best checkpoint",
    )
    axes[0].axhline(0.0, color="#666666", linewidth=0.9, linestyle="--")
    axes[0].set_ylabel("RMS improvement vs baseline (%)")
    axes[0].set_title("Mean tracking improvement")
    axes[0].legend(loc="best")
    axes[0].grid(True, alpha=0.25)

    colors = ("#247BA0", "#D1495B", "#3A7D44", "#E09F3E", "#6A4C93", "#2A9D8F")
    for (case, label), color in zip(CASE_LABELS.items(), colors):
        values = np.asarray(
            [float(row[f"{case}_improvement_percent"]) for row in rows]
        )
        axes[1].plot(
            steps,
            values,
            marker="o",
            markersize=3,
            linewidth=1.5,
            color=color,
            label=label,
        )
    axes[1].axhline(0.0, color="#666666", linewidth=0.9, linestyle="--")
    axes[1].set_xlabel("PPO transitions")
    axes[1].set_ylabel("RMS improvement vs baseline (%)")
    axes[1].set_title("Per-trajectory tracking improvement")
    axes[1].legend(ncol=2, fontsize=9, loc="best")
    axes[1].grid(True, alpha=0.25)

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=180)
    print(f"固定验证指标图已保存到 {output}")
    if args.show:
        plt.show()
    else:
        plt.close(figure)


if __name__ == "__main__":
    main()
