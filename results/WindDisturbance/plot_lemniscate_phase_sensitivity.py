#!/usr/bin/env python3
"""扫描并绘制 Lemniscate 在 5–6 m/s 常值风下的开始相位敏感性。"""

from __future__ import annotations

import argparse
import contextlib
import csv
import io
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

DEFAULT_LOG_DIR = PROJECT_ROOT / "validation_logs" / "wind_phase_sweep"
DEFAULT_SUMMARY_OUTPUT = SCRIPT_DIR / "lemniscate_phase_sensitivity_summary.png"
DEFAULT_HISTORY_OUTPUT = SCRIPT_DIR / "lemniscate_phase_sensitivity_histories.png"
DEFAULT_CSV_OUTPUT = SCRIPT_DIR / "lemniscate_phase_sensitivity_metrics.csv"

WIND_SPEEDS = (5.0, 6.0)
START_TIMES = (0.5, 1.0, 2.0, 3.0, 4.0)
WIND_DURATION = 3.0
DIRECTIONS = {
    "North": np.array([1.0, 0.0, 0.0]),
    "East": np.array([0.0, 1.0, 0.0]),
    "Up": np.array([0.0, 0.0, -1.0]),
}
COLORS = {"North": "#0072B2", "East": "#D55E00", "Up": "#009E73"}


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="绘制 Lemniscate 在 5–6 m/s 风扰下的开始相位敏感性"
    )
    parser.add_argument(
        "--run-simulations",
        action="store_true",
        help="先运行全部 30 组仿真；否则读取已有扫描日志",
    )
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    parser.add_argument("--summary-output", type=Path, default=DEFAULT_SUMMARY_OUTPUT)
    parser.add_argument("--history-output", type=Path, default=DEFAULT_HISTORY_OUTPUT)
    parser.add_argument("--csv-output", type=Path, default=DEFAULT_CSV_OUTPUT)
    parser.add_argument("--dpi", type=int, default=180)
    parser.add_argument("--show", action="store_true")
    return parser.parse_args()


def _number_token(value: float) -> str:
    return f"{value:g}".replace(".", "p")


def _log_path(log_dir: Path, direction: str, speed: float, start: float) -> Path:
    return log_dir / (
        f"lemniscate_{direction.lower()}_{_number_token(speed)}ms_"
        f"start_{_number_token(start)}s.npz"
    )


def _run_simulations(log_dir: Path) -> None:
    from environment.wind import ConstantWind, TimeWindowWind
    from main_sim import main as run_simulation

    log_dir.mkdir(parents=True, exist_ok=True)
    case_count = len(DIRECTIONS) * len(WIND_SPEEDS) * len(START_TIMES)
    completed = 0
    for direction, unit_vector in DIRECTIONS.items():
        for speed in WIND_SPEEDS:
            for start in START_TIMES:
                wind = TimeWindowWind(
                    ConstantWind(speed * unit_vector),
                    start_time=start,
                    end_time=start + WIND_DURATION,
                )
                path = _log_path(log_dir, direction, speed, start)
                # main_sim 会输出每次仿真的常规摘要；扫描时只保留下面的进度信息。
                with contextlib.redirect_stdout(io.StringIO()):
                    result = run_simulation(
                        trajectory_name="lemniscate",
                        laps=1,
                        log_path=str(path),
                        show_plot=False,
                        wind_profile=wind,
                    )
                completed += 1
                print(
                    f"[{completed:02d}/{case_count}] {direction:5s} "
                    f"{speed:g} m/s, start={start:g} s: "
                    f"RMS={result['rms_position_error']:.3f} m, "
                    f"max={result['max_position_error']:.3f} m"
                )


def _load_case(path: Path) -> dict[str, np.ndarray | float]:
    if not path.is_file():
        raise FileNotFoundError(
            f"缺少扫描日志：{path}\n"
            "请先添加 --run-simulations 运行 30 组相位扫描。"
        )
    required = ("time", "position", "position_ref", "rotor_speed")
    with np.load(path, allow_pickle=False) as archive:
        missing = [name for name in required if name not in archive]
        if missing:
            raise KeyError(f"{path.name} 缺少字段：{', '.join(missing)}")
        data = {name: np.asarray(archive[name], dtype=float) for name in required}

    time = data["time"]
    position = data["position"]
    position_ref = data["position_ref"]
    rotor_speed = data["rotor_speed"]
    if time.ndim != 1 or time.size < 2 or not np.all(np.diff(time) > 0.0):
        raise ValueError(f"{path.name} 的 time 必须一维且严格递增")
    for name, values in (("position", position), ("position_ref", position_ref)):
        if values.shape != (time.size, 3) or not np.all(np.isfinite(values)):
            raise ValueError(f"{path.name} 的 {name} 必须是有限的 N x 3 数组")
    if rotor_speed.shape != (time.size, 2) or not np.all(np.isfinite(rotor_speed)):
        raise ValueError(f"{path.name} 的 rotor_speed 必须是有限的 N x 2 数组")

    error_norm = np.linalg.norm(position_ref - position, axis=1)
    return {
        **data,
        "error_norm": error_norm,
        "rms_error": float(np.sqrt(np.mean(error_norm**2))),
        "max_error": float(np.max(error_norm)),
        "rotor_saturation_fraction": float(
            np.mean(np.any(rotor_speed >= 2499.0, axis=1))
        ),
    }


def _classification(rms_error: float, max_error: float, saturation: float) -> str:
    if max_error > 5.0 or rms_error > 3.0 or saturation > 0.05:
        return "tracking collapse"
    if rms_error > 1.0 or max_error > 2.0 or saturation >= 0.01:
        return "degraded"
    return "effective"


def _load_sweep(log_dir: Path) -> dict[tuple[str, float, float], dict]:
    cases = {}
    for direction in DIRECTIONS:
        for speed in WIND_SPEEDS:
            for start in START_TIMES:
                cases[(direction, speed, start)] = _load_case(
                    _log_path(log_dir, direction, speed, start)
                )
    return cases


def _write_metrics_csv(path: Path, cases: dict[tuple[str, float, float], dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            (
                "direction",
                "wind_speed_mps",
                "wind_start_s",
                "wind_end_s",
                "rms_position_error_m",
                "max_position_error_m",
                "rotor_saturation_fraction",
                "classification",
            )
        )
        for (direction, speed, start), case in cases.items():
            rms_error = float(case["rms_error"])
            max_error = float(case["max_error"])
            saturation = float(case["rotor_saturation_fraction"])
            writer.writerow(
                (
                    direction,
                    speed,
                    start,
                    start + WIND_DURATION,
                    rms_error,
                    max_error,
                    saturation,
                    _classification(rms_error, max_error, saturation),
                )
            )


def _plot_summary(cases: dict[tuple[str, float, float], dict]) -> plt.Figure:
    figure, axes = plt.subplots(2, 2, figsize=(12.0, 8.2), sharex=True)
    for column, speed in enumerate(WIND_SPEEDS):
        rms_axis = axes[0, column]
        max_axis = axes[1, column]
        for direction in DIRECTIONS:
            rms_values = [
                cases[(direction, speed, start)]["rms_error"]
                for start in START_TIMES
            ]
            max_values = [
                cases[(direction, speed, start)]["max_error"]
                for start in START_TIMES
            ]
            rms_axis.plot(
                START_TIMES,
                rms_values,
                marker="o",
                linewidth=1.8,
                color=COLORS[direction],
                label=direction,
            )
            max_axis.plot(
                START_TIMES,
                max_values,
                marker="o",
                linewidth=1.8,
                color=COLORS[direction],
                label=direction,
            )

        rms_axis.axhspan(0.0, 1.0, color="#009E73", alpha=0.08)
        rms_axis.axhline(1.0, color="#CC3311", linestyle="--", linewidth=1.2)
        max_axis.axhspan(0.0, 2.0, color="#009E73", alpha=0.08)
        max_axis.axhline(2.0, color="#CC3311", linestyle="--", linewidth=1.2)
        rms_axis.set_title(f"{speed:g} m/s constant wind")
        rms_axis.set_ylabel("RMS position error [m]")
        max_axis.set_ylabel("Maximum position error [m]")
        max_axis.set_xlabel("Wind start time [s]")
        rms_axis.legend(loc="upper right")
        for axis in (rms_axis, max_axis):
            axis.grid(True, color="0.87", linewidth=0.6)
            axis.set_xticks(START_TIMES)
            axis.set_ylim(bottom=0.0)
            axis.tick_params(direction="in", top=True, right=True)

    figure.suptitle(
        "Lemniscate Wind-Onset Phase Sensitivity (3 s duration)", fontsize=15
    )
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.97))
    return figure


def _plot_histories(cases: dict[tuple[str, float, float], dict]) -> plt.Figure:
    figure, axes = plt.subplots(
        len(WIND_SPEEDS),
        len(DIRECTIONS),
        figsize=(15.0, 7.8),
        sharex=True,
        sharey=True,
    )
    phase_colors = plt.colormaps["viridis"](np.linspace(0.08, 0.92, len(START_TIMES)))
    for row, speed in enumerate(WIND_SPEEDS):
        for column, direction in enumerate(DIRECTIONS):
            axis = axes[row, column]
            for start, color in zip(START_TIMES, phase_colors):
                case = cases[(direction, speed, start)]
                local_time = np.asarray(case["time"]) - start
                visible = (local_time >= -0.5) & (local_time <= 3.5)
                axis.plot(
                    local_time[visible],
                    np.asarray(case["error_norm"])[visible],
                    color=color,
                    linewidth=1.35,
                    label=f"start={start:g} s",
                )
            axis.axvspan(0.0, WIND_DURATION, color="#E69F00", alpha=0.10)
            axis.axvline(0.0, color="#A35C00", linestyle="--", linewidth=0.9)
            axis.axvline(
                WIND_DURATION, color="#A35C00", linestyle="--", linewidth=0.9
            )
            axis.axhline(2.0, color="#CC3311", linestyle=":", linewidth=1.1)
            axis.set_title(f"{direction}, {speed:g} m/s")
            axis.grid(True, color="0.88", linewidth=0.6)
            axis.tick_params(direction="in", top=True, right=True)
            if column == 0:
                axis.set_ylabel("Position error norm [m]")
            if row == len(WIND_SPEEDS) - 1:
                axis.set_xlabel(r"Time from wind onset $\tau$ [s]")

    axes[0, -1].legend(loc="upper left", fontsize=8)
    figure.suptitle(
        "Phase-Aligned Tracking Error Histories (orange: wind active)", fontsize=15
    )
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.96))
    return figure


def main() -> None:
    args = _parse_args()
    log_dir = args.log_dir.resolve()
    if args.run_simulations:
        _run_simulations(log_dir)

    cases = _load_sweep(log_dir)
    _write_metrics_csv(args.csv_output.resolve(), cases)
    summary_figure = _plot_summary(cases)
    history_figure = _plot_histories(cases)

    for figure, path in (
        (summary_figure, args.summary_output.resolve()),
        (history_figure, args.history_output.resolve()),
    ):
        path.parent.mkdir(parents=True, exist_ok=True)
        figure.savefig(path, dpi=args.dpi, bbox_inches="tight")
        print(f"结果图已保存到：{path}")
    print(f"指标表已保存到：{args.csv_output.resolve()}")

    for speed in WIND_SPEEDS:
        print(f"\n{speed:g} m/s 最差相位：")
        for direction in DIRECTIONS:
            worst_start = max(
                START_TIMES,
                key=lambda start: cases[(direction, speed, start)]["max_error"],
            )
            case = cases[(direction, speed, worst_start)]
            print(
                f"  {direction:5s}: start={worst_start:g} s, "
                f"RMS={case['rms_error']:.3f} m, "
                f"max={case['max_error']:.3f} m"
            )

    if args.show:
        plt.show()
    else:
        plt.close(summary_figure)
        plt.close(history_figure)


if __name__ == "__main__":
    main()
