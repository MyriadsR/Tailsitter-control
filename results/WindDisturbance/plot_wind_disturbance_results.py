#!/usr/bin/env python3
"""绘制风速、风致外力/力矩和位置跟踪误差诊断图。"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parents[1]
DEFAULT_LOG = PROJECT_ROOT / "validation_logs" / "lemniscate-windowed-wind.npz"
DEFAULT_OUTPUT = SCRIPT_DIR / "wind_disturbance_results.png"

VECTOR_FIELDS = (
    "position",
    "position_ref",
    "wind_velocity",
    "wind_external_force",
    "wind_external_moment",
)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="绘制风扰输入、动力学增量与位置跟踪误差"
    )
    parser.add_argument(
        "log",
        nargs="?",
        type=Path,
        default=DEFAULT_LOG,
        help=f"包含风扰字段的 NPZ 日志（默认：{DEFAULT_LOG}）",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help=f"输出图片（默认：{DEFAULT_OUTPUT}）",
    )
    parser.add_argument("--start", type=float, help="绘图起始时刻，单位 s")
    parser.add_argument("--end", type=float, help="绘图结束时刻，单位 s")
    parser.add_argument("--dpi", type=int, default=180, help="输出图片分辨率")
    parser.add_argument("--show", action="store_true", help="保存后显示绘图窗口")
    return parser.parse_args()


def _load_log(path: Path) -> dict[str, np.ndarray]:
    if not path.is_file():
        raise FileNotFoundError(
            f"找不到风扰日志：{path}\n"
            "请先使用 main_sim.py 的 --wind-mode 和 --log 参数生成日志。"
        )

    required = ("time",) + VECTOR_FIELDS
    with np.load(path, allow_pickle=False) as archive:
        missing = [name for name in required if name not in archive]
        if missing:
            raise KeyError(
                f"{path.name} 缺少风扰诊断字段：{', '.join(missing)}。"
                "请使用当前 main_sim.py 重新生成有风日志。"
            )
        data = {
            name: np.asarray(archive[name], dtype=float) for name in required
        }

    time = data["time"]
    if time.ndim != 1 or time.size < 2 or not np.all(np.diff(time) > 0.0):
        raise ValueError("time 必须是一维、严格递增且至少包含两个采样点")
    if not np.all(np.isfinite(time)):
        raise ValueError("time 中包含 NaN 或无穷值")

    for name in VECTOR_FIELDS:
        values = data[name]
        if values.shape != (time.size, 3):
            raise ValueError(
                f"{name} 的形状应为 ({time.size}, 3)，实际为 {values.shape}"
            )
        if not np.all(np.isfinite(values)):
            raise ValueError(f"{name} 中包含 NaN 或无穷值")
    return data


def _select_window(
    time: np.ndarray, start: float | None, end: float | None
) -> np.ndarray:
    start_time = float(time[0]) if start is None else float(start)
    end_time = float(time[-1]) if end is None else float(end)
    if not np.isfinite(start_time) or not np.isfinite(end_time):
        raise ValueError("--start 和 --end 必须是有限数值")
    if end_time <= start_time:
        raise ValueError("--end 必须大于 --start")

    mask = (time >= start_time) & (time <= end_time)
    if np.count_nonzero(mask) < 2:
        raise ValueError("指定时间窗口内至少需要两个日志采样点")
    return mask


def _active_intervals(time: np.ndarray, active: np.ndarray) -> list[tuple[float, float]]:
    """把风速非零采样转换为用于背景标记的连续时间区间。"""
    if not np.any(active):
        return []

    edges = np.diff(active.astype(np.int8), prepend=0, append=0)
    starts = np.flatnonzero(edges == 1)
    stops = np.flatnonzero(edges == -1)
    median_dt = float(np.median(np.diff(time)))
    return [
        (
            float(time[start]),
            float(time[stop]) if stop < time.size else float(time[-1] + median_dt),
        )
        for start, stop in zip(starts, stops)
    ]


def _plot_vector(
    axis,
    time: np.ndarray,
    vector: np.ndarray,
    component_labels: tuple[str, str, str],
    ylabel: str,
) -> None:
    colors = ("#0072B2", "#D55E00", "#009E73")
    for index, (label, color) in enumerate(zip(component_labels, colors)):
        axis.plot(time, vector[:, index], color=color, linewidth=1.1, label=label)
    axis.plot(
        time,
        np.linalg.norm(vector, axis=1),
        color="#202020",
        linewidth=1.5,
        label="Magnitude",
    )
    axis.set_ylabel(ylabel)
    axis.grid(True, color="0.88", linewidth=0.6)
    axis.legend(loc="upper right", ncol=4, fontsize=8)


def create_figure(
    data: dict[str, np.ndarray], mask: np.ndarray
) -> tuple[plt.Figure, dict[str, float | list[tuple[float, float]]]]:
    time = data["time"][mask]
    wind = data["wind_velocity"][mask]
    force = data["wind_external_force"][mask]
    moment = data["wind_external_moment"][mask]
    position_error = (data["position_ref"] - data["position"])[mask]

    wind_norm = np.linalg.norm(wind, axis=1)
    active_threshold = max(1e-9, 1e-6 * float(np.max(wind_norm)))
    active = wind_norm > active_threshold
    intervals = _active_intervals(time, active)

    figure, axes = plt.subplots(4, 1, figsize=(12.0, 11.0), sharex=True)
    _plot_vector(axes[0], time, wind, ("North", "East", "Down"), "Wind [m/s]")
    axes[0].set_title("a) Wind velocity in world NED frame", loc="left")
    _plot_vector(axes[1], time, force, ("North", "East", "Down"), "Force [N]")
    axes[1].set_title("b) Wind-induced external force in world frame", loc="left")
    _plot_vector(axes[2], time, moment, (r"$b_x$", r"$b_y$", r"$b_z$"), "Moment [N m]")
    axes[2].set_title("c) Wind-induced external moment in body frame", loc="left")
    _plot_vector(axes[3], time, position_error, ("North", "East", "Down"), "Error [m]")
    axes[3].set_title("d) Position tracking error", loc="left")
    axes[3].set_xlabel("Time [s]")

    for axis in axes:
        for start, stop in intervals:
            axis.axvspan(start, stop, color="#E69F00", alpha=0.10, linewidth=0.0)
        axis.set_xlim(time[0], time[-1])
        axis.tick_params(direction="in", top=True, right=True)

    figure.suptitle("Wind Disturbance Response", fontsize=15)
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.98))

    error_norm = np.linalg.norm(position_error, axis=1)
    stats: dict[str, float | list[tuple[float, float]]] = {
        "intervals": intervals,
        "max_wind_speed": float(np.max(wind_norm)),
        "max_external_force": float(np.max(np.linalg.norm(force, axis=1))),
        "max_external_moment": float(np.max(np.linalg.norm(moment, axis=1))),
        "rms_position_error": float(np.sqrt(np.mean(error_norm**2))),
        "active_rms_position_error": (
            float(np.sqrt(np.mean(error_norm[active] ** 2)))
            if np.any(active)
            else float("nan")
        ),
    }
    return figure, stats


def main() -> None:
    args = _parse_args()
    log_path = args.log.resolve()
    data = _load_log(log_path)
    mask = _select_window(data["time"], args.start, args.end)
    figure, stats = create_figure(data, mask)

    output_path = args.output.resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=args.dpi, bbox_inches="tight")
    print(f"输入风扰日志：{log_path}")
    print(f"风扰诊断图已保存到：{output_path}")
    if stats["intervals"]:
        intervals_text = ", ".join(
            f"[{start:.3f}, {stop:.3f})" for start, stop in stats["intervals"]
        )
        print(f"检测到的风扰区间：{intervals_text} s")
    else:
        print("所选窗口内未检测到非零风速。")
    print(
        f"最大风速 = {stats['max_wind_speed']:.3f} m/s，"
        f"最大风致外力 = {stats['max_external_force']:.3f} N，"
        f"最大风致外力矩 = {stats['max_external_moment']:.3f} N m"
    )
    print(f"所选窗口位置误差 RMS = {stats['rms_position_error']:.3f} m")
    if np.isfinite(stats["active_rms_position_error"]):
        print(
            "有风采样位置误差 RMS = "
            f"{stats['active_rms_position_error']:.3f} m"
        )

    if args.show:
        plt.show()
    else:
        plt.close(figure)


if __name__ == "__main__":
    main()
