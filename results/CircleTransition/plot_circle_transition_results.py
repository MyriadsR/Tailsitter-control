#!/usr/bin/env python3
"""绘制论文 Fig. 10 风格的圆轨迹双向过渡结果。"""

from __future__ import annotations

import argparse
import warnings
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parents[1]
DEFAULT_TO_FORWARD_LOG = (
    PROJECT_ROOT / "validation_logs" / "transition-to-forward.npz"
)
DEFAULT_TO_HOVER_LOG = (
    PROJECT_ROOT / "validation_logs" / "transition-to-hover.npz"
)
DEFAULT_OUTPUT = SCRIPT_DIR / "circle_transition_results.png"
GRAVITY_NED = np.array([0.0, 0.0, 9.81])


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="生成论文 Fig. 10 风格的圆轨迹双向过渡十联图"
    )
    parser.add_argument(
        "--to-forward-log", type=Path, default=DEFAULT_TO_FORWARD_LOG
    )
    parser.add_argument("--to-hover-log", type=Path, default=DEFAULT_TO_HOVER_LOG)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--dpi", type=int, default=180)
    parser.add_argument("--show", action="store_true")
    return parser.parse_args()


def _load_log(path: Path) -> dict[str, np.ndarray | None]:
    if not path.is_file():
        raise FileNotFoundError(f"找不到仿真日志：{path}")
    required = (
        "time",
        "position",
        "velocity",
        "position_ref",
        "velocity_ref",
        "acceleration_ref",
        "euler_deg",
    )
    with np.load(path, allow_pickle=False) as archive:
        missing = [name for name in required if name not in archive]
        if missing:
            raise KeyError(f"{path.name} 缺少字段：{', '.join(missing)}")
        data: dict[str, np.ndarray | None] = {
            name: np.asarray(archive[name], dtype=float) for name in required
        }
        data["acceleration"] = (
            np.asarray(archive["acceleration"], dtype=float)
            if "acceleration" in archive
            else None
        )
        data["euler_command_deg"] = (
            np.asarray(archive["euler_command_deg"], dtype=float)
            if "euler_command_deg" in archive
            else None
        )

    time = np.asarray(data["time"])
    if time.ndim != 1 or time.size < 3 or not np.all(np.diff(time) > 0.0):
        raise ValueError(f"{path.name} 的 time 必须一维且严格递增")
    for name in required[1:]:
        values = np.asarray(data[name])
        if values.shape != (time.size, 3):
            raise ValueError(
                f"{path.name}: {name} 应为 ({time.size}, 3)，"
                f"实际为 {values.shape}"
            )
    for name in ("acceleration", "euler_command_deg"):
        values = data[name]
        if values is not None and np.asarray(values).shape != (time.size, 3):
            raise ValueError(
                f"{path.name}: {name} 应为 ({time.size}, 3)，"
                f"实际为 {np.asarray(values).shape}"
            )

    if data["acceleration"] is None:
        warnings.warn(
            f"{path.name} 缺少 acceleration，将由速度数值微分估算。",
            stacklevel=2,
        )
        data["acceleration"] = np.gradient(
            np.asarray(data["velocity"]), time, axis=0, edge_order=2
        )
    if data["euler_command_deg"] is None:
        warnings.warn(
            f"{path.name} 缺少 euler_command_deg，姿态图只显示实际值。",
            stacklevel=2,
        )
    return data


def _continuous_euler_radians(euler_deg: np.ndarray) -> np.ndarray:
    return np.unwrap(np.deg2rad(np.asarray(euler_deg, dtype=float)), axis=0)


def _style_time_axis(axis: plt.Axes, duration: float) -> None:
    axis.set_xlim(0.0, duration)
    axis.set_xlabel("Time [s]")
    axis.grid(True, color="0.84", linewidth=0.7)
    axis.tick_params(direction="in", top=True, right=True)


def _shared_position_limits(
    datasets: tuple[dict[str, np.ndarray | None], ...]
) -> tuple[tuple[float, float], tuple[float, float], tuple[float, float]]:
    points = np.vstack(
        [
            np.asarray(data[name])
            for data in datasets
            for name in ("position_ref", "position")
        ]
    )
    displayed = points.copy()
    displayed[:, 2] *= -1.0
    lower = np.min(displayed, axis=0)
    upper = np.max(displayed, axis=0)
    margins = np.array([0.4, 0.4, 0.3])
    return tuple(
        (float(lower[index] - margins[index]), float(upper[index] + margins[index]))
        for index in (1, 0, 2)
    )


def _plot_position(
    axis: plt.Axes,
    data: dict[str, np.ndarray | None],
    title: str,
    limits: tuple[tuple[float, float], tuple[float, float], tuple[float, float]],
    reference_color: str,
    actual_color: str,
) -> None:
    position = np.asarray(data["position"])
    position_ref = np.asarray(data["position_ref"])
    axis.plot(
        position_ref[:, 1], position_ref[:, 0], -position_ref[:, 2],
        color=reference_color, linewidth=2.0, label="Reference"
    )
    axis.plot(
        position[:, 1], position[:, 0], -position[:, 2],
        color=actual_color, linewidth=1.25, label="Simulation"
    )
    axis.set_xlim(*limits[0])
    axis.set_ylim(*limits[1])
    axis.set_zlim(*limits[2])
    axis.set_box_aspect((1.35, 1.05, 0.58))
    axis.view_init(elev=23.0, azim=-58.0)
    axis.set_title(title, loc="left")
    axis.set_xlabel(r"$y_i$ [m]", labelpad=5)
    axis.set_ylabel(r"$x_i$ [m]", labelpad=5)
    axis.set_zlabel(r"Altitude $-z_i$ [m]", labelpad=5)
    axis.grid(True, color="0.88", linewidth=0.6)
    axis.legend(loc="upper right")


def _plot_error(
    axis: plt.Axes,
    data: dict[str, np.ndarray | None],
    title: str,
    colors: tuple[str, str, str],
) -> None:
    time = np.asarray(data["time"])
    error = np.asarray(data["position_ref"]) - np.asarray(data["position"])
    labels = (
        r"$x_{\mathrm{ref}}-x$",
        r"$y_{\mathrm{ref}}-y$",
        r"$z_{\mathrm{ref}}-z$",
    )
    for index, (color, label) in enumerate(zip(colors, labels)):
        axis.plot(time, error[:, index], color=color, label=label)
    axis.set_title(title, loc="left")
    axis.set_ylabel("Error [m]")
    axis.legend(loc="best", ncol=3)
    _style_time_axis(axis, float(time[-1]))


def _plot_speed(
    axis: plt.Axes,
    data: dict[str, np.ndarray | None],
    title: str,
    reference_color: str,
    actual_color: str,
) -> None:
    time = np.asarray(data["time"])
    speed = np.linalg.norm(np.asarray(data["velocity"]), axis=1)
    speed_ref = np.linalg.norm(np.asarray(data["velocity_ref"]), axis=1)
    axis.plot(time, speed_ref, color=reference_color, linewidth=2.0, label="Reference")
    axis.plot(time, speed, color=actual_color, linewidth=1.35, label="Simulation")
    axis.set_title(title, loc="left")
    axis.set_ylabel(r"$\|v\|$ [m/s]")
    axis.legend(loc="best")
    _style_time_axis(axis, float(time[-1]))


def _plot_acceleration(
    axis: plt.Axes,
    data: dict[str, np.ndarray | None],
    title: str,
    reference_color: str,
    actual_color: str,
) -> None:
    time = np.asarray(data["time"])
    specific_force = np.linalg.norm(
        np.asarray(data["acceleration"]) - GRAVITY_NED, axis=1
    )
    specific_force_ref = np.linalg.norm(
        np.asarray(data["acceleration_ref"]) - GRAVITY_NED, axis=1
    )
    axis.plot(
        time, specific_force_ref, color=reference_color,
        linewidth=2.0, label="Reference"
    )
    axis.plot(
        time, specific_force, color=actual_color,
        linewidth=1.25, label="Simulation"
    )
    axis.set_title(title, loc="left")
    axis.set_ylabel(r"$\|a-g i_z\|$ [m/s$^2$]")
    axis.legend(loc="best")
    _style_time_axis(axis, float(time[-1]))


def _plot_attitude(
    axis: plt.Axes,
    data: dict[str, np.ndarray | None],
    title: str,
    colors: tuple[str, str, str],
) -> None:
    time = np.asarray(data["time"])
    actual = _continuous_euler_radians(np.asarray(data["euler_deg"]))
    command_deg = data["euler_command_deg"]
    symbols = (r"$\psi$", r"$\phi$", r"$\theta$")
    if command_deg is not None:
        command = _continuous_euler_radians(np.asarray(command_deg))
        offsets = 2.0 * np.pi * np.round(
            (command[0] - actual[0]) / (2.0 * np.pi)
        )
        actual += offsets
        for index, (color, symbol) in enumerate(zip(colors, symbols)):
            axis.plot(
                time, command[:, index], color=color, linewidth=1.9,
                label=rf"{symbol[:-1]}_c$"
            )
            axis.plot(
                time, actual[:, index], color=color, linewidth=1.15,
                linestyle="--", label=symbol
            )
    else:
        for index, (color, symbol) in enumerate(zip(colors, symbols)):
            axis.plot(time, actual[:, index], color=color, label=symbol)
    axis.set_title(title, loc="left")
    axis.set_ylabel("ZXY Euler angle [rad]")
    axis.legend(loc="best", ncol=3)
    _style_time_axis(axis, float(time[-1]))


def _case_stats(data: dict[str, np.ndarray | None]) -> dict[str, float]:
    error_norm = np.linalg.norm(
        np.asarray(data["position_ref"]) - np.asarray(data["position"]), axis=1
    )
    speed = np.linalg.norm(np.asarray(data["velocity"]), axis=1)
    speed_ref = np.linalg.norm(np.asarray(data["velocity_ref"]), axis=1)
    specific_force = np.linalg.norm(
        np.asarray(data["acceleration"]) - GRAVITY_NED, axis=1
    )
    return {
        "rms_error": float(np.sqrt(np.mean(error_norm**2))),
        "max_error": float(np.max(error_norm)),
        "initial_speed": float(speed[0]),
        "final_speed": float(speed[-1]),
        "reference_initial_speed": float(speed_ref[0]),
        "reference_final_speed": float(speed_ref[-1]),
        "max_specific_force": float(np.max(specific_force)),
    }


def create_figure(
    to_forward: dict[str, np.ndarray | None],
    to_hover: dict[str, np.ndarray | None],
) -> tuple[plt.Figure, dict[str, dict[str, float]]]:
    plt.rcParams.update(
        {
            "font.family": "serif",
            "font.size": 9.5,
            "axes.titlesize": 11,
            "axes.labelsize": 9.5,
            "legend.fontsize": 8,
            "lines.linewidth": 1.5,
        }
    )
    figure = plt.figure(figsize=(14.0, 17.0), constrained_layout=True)
    figure.get_layout_engine().set(hspace=0.07, wspace=0.05)
    grid = figure.add_gridspec(
        5, 2, height_ratios=(1.35, 1.0, 1.0, 1.0, 1.0)
    )
    reference_color = "#0072B2"
    actual_color = "#D55E00"
    component_colors = ("#0072B2", "#D55E00", "#009E73")
    position_limits = _shared_position_limits((to_forward, to_hover))

    forward_position_axis = figure.add_subplot(grid[0, 0], projection="3d")
    hover_position_axis = figure.add_subplot(grid[0, 1], projection="3d")
    _plot_position(
        forward_position_axis, to_forward,
        "a) Position — Static hover to coordinated flight",
        position_limits, reference_color, actual_color
    )
    _plot_position(
        hover_position_axis, to_hover,
        "b) Position — Coordinated flight to static hover",
        position_limits, reference_color, actual_color
    )

    _plot_error(
        figure.add_subplot(grid[1, 0]), to_forward,
        "c) Position tracking error", component_colors
    )
    _plot_error(
        figure.add_subplot(grid[1, 1]), to_hover,
        "d) Position tracking error", component_colors
    )
    _plot_speed(
        figure.add_subplot(grid[2, 0]), to_forward, "e) Speed",
        reference_color, actual_color
    )
    _plot_speed(
        figure.add_subplot(grid[2, 1]), to_hover, "f) Speed",
        reference_color, actual_color
    )
    _plot_acceleration(
        figure.add_subplot(grid[3, 0]), to_forward, "g) Acceleration",
        reference_color, actual_color
    )
    _plot_acceleration(
        figure.add_subplot(grid[3, 1]), to_hover, "h) Acceleration",
        reference_color, actual_color
    )
    _plot_attitude(
        figure.add_subplot(grid[4, 0]), to_forward, "i) Attitude",
        component_colors
    )
    _plot_attitude(
        figure.add_subplot(grid[4, 1]), to_hover, "j) Attitude",
        component_colors
    )

    figure.suptitle(
        r"Transition on a 3.5 m Circle — Static Hover $\leftrightarrow$ "
        "Coordinated Flight at 8 m/s",
        fontsize=15,
    )
    return figure, {
        "to_forward": _case_stats(to_forward),
        "to_hover": _case_stats(to_hover),
    }


def main() -> None:
    args = _parse_args()
    to_forward = _load_log(args.to_forward_log.resolve())
    to_hover = _load_log(args.to_hover_log.resolve())
    figure, stats = create_figure(to_forward, to_hover)

    output_path = args.output.resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=args.dpi, bbox_inches="tight")
    print(f"结果图已保存到：{output_path}")
    for key, label in (
        ("to_forward", "Hover -> coordinated"),
        ("to_hover", "Coordinated -> hover"),
    ):
        case = stats[key]
        print(
            f"{label}: 位置误差 RMS = {case['rms_error']:.3f} m，"
            f"最大 = {case['max_error']:.3f} m；实际速度 "
            f"{case['initial_speed']:.3f} -> {case['final_speed']:.3f} m/s，"
            f"最大比力 = {case['max_specific_force']:.3f} m/s^2"
        )

    if args.show:
        plt.show()
    else:
        plt.close(figure)


if __name__ == "__main__":
    main()
