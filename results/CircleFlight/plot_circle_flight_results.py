#!/usr/bin/env python3
"""绘制论文 Fig. 8 风格的 coordinated/knife-edge 圆轨迹结果。"""

from __future__ import annotations

import argparse
import warnings
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parents[1]
DEFAULT_COORDINATED_LOG = (
    PROJECT_ROOT / "validation_logs" / "circle-coordinated.npz"
)
DEFAULT_KNIFE_EDGE_LOG = (
    PROJECT_ROOT / "validation_logs" / "circular-knife-edge.npz"
)
DEFAULT_OUTPUT = SCRIPT_DIR / "circle_flight_results.png"
GRAVITY_NED = np.array([0.0, 0.0, 9.81])


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="生成论文 Fig. 8 风格的圆轨迹十联结果图"
    )
    parser.add_argument(
        "--coordinated-log",
        type=Path,
        default=DEFAULT_COORDINATED_LOG,
        help=f"coordinated flight NPZ（默认：{DEFAULT_COORDINATED_LOG}）",
    )
    parser.add_argument(
        "--knife-edge-log",
        type=Path,
        default=DEFAULT_KNIFE_EDGE_LOG,
        help=f"knife-edge flight NPZ（默认：{DEFAULT_KNIFE_EDGE_LOG}）",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help=f"输出图片（默认：{DEFAULT_OUTPUT}）",
    )
    parser.add_argument(
        "--lap",
        type=int,
        help=(
            "时域图使用的基础圈数，从 1 开始；默认选择最后一个同时允许"
            "两种构型完成论文相位对齐的圈"
        ),
    )
    parser.add_argument(
        "--radius", type=float, default=3.5, help="论文圆轨迹半径，默认 3.5 m"
    )
    parser.add_argument(
        "--reference-speed",
        type=float,
        default=8.1,
        help="论文参考速度，默认 8.1 m/s（实验实际速度约 7.8 m/s）",
    )
    parser.add_argument(
        "--lap-duration",
        type=float,
        help="单圈时间；默认由 2*pi*radius/reference-speed 计算",
    )
    parser.add_argument("--dpi", type=int, default=180, help="输出图片分辨率")
    parser.add_argument("--show", action="store_true", help="保存后显示绘图窗口")
    return parser.parse_args()


def _load_log(log_path: Path) -> dict[str, np.ndarray | None]:
    if not log_path.is_file():
        raise FileNotFoundError(f"找不到仿真日志：{log_path}")

    required = (
        "time",
        "position",
        "velocity",
        "position_ref",
        "velocity_ref",
        "acceleration_ref",
        "euler_deg",
    )
    with np.load(log_path, allow_pickle=False) as archive:
        missing = [name for name in required if name not in archive]
        if missing:
            raise KeyError(f"{log_path.name} 缺少字段：{', '.join(missing)}")
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
        raise ValueError(f"{log_path.name} 的 time 必须一维且严格递增")
    for name in required[1:]:
        values = np.asarray(data[name])
        if values.shape != (time.size, 3):
            raise ValueError(
                f"{log_path.name}: {name} 应为 ({time.size}, 3)，"
                f"实际为 {values.shape}"
            )
    for name in ("acceleration", "euler_command_deg"):
        values = data[name]
        if values is not None and np.asarray(values).shape != (time.size, 3):
            raise ValueError(
                f"{log_path.name}: {name} 应为 ({time.size}, 3)，"
                f"实际为 {np.asarray(values).shape}"
            )

    if data["acceleration"] is None:
        warnings.warn(
            f"{log_path.name} 缺少 acceleration，将由 velocity 数值微分估算。",
            stacklevel=2,
        )
        data["acceleration"] = np.gradient(
            np.asarray(data["velocity"]), time, axis=0, edge_order=2
        )
    if data["euler_command_deg"] is None:
        warnings.warn(
            f"{log_path.name} 缺少 euler_command_deg，姿态图只显示实际值。",
            stacklevel=2,
        )
    return data


def _maximum_window_lap(
    time: np.ndarray, lap_duration: float, phase_fraction: float
) -> int:
    sample_period = float(np.median(np.diff(time)))
    recorded_duration = float(time[-1] - time[0] + sample_period)
    return int(
        np.floor(recorded_duration / lap_duration - phase_fraction + 1e-8)
    )


def _select_lap_window(
    time: np.ndarray,
    lap_duration: float,
    lap: int,
    phase_fraction: float,
) -> tuple[np.ndarray, np.ndarray, float]:
    start = time[0] + (lap - 1 + phase_fraction) * lap_duration
    stop = start + lap_duration
    sample_period = float(np.median(np.diff(time)))
    if stop > time[-1] + sample_period:
        raise ValueError(
            f"第 {lap} 圈加 {phase_fraction:.2f} 圈相位后超出日志范围"
        )
    mask = (time >= start - 0.25 * sample_period) & (
        time < stop - 0.25 * sample_period
    )
    if np.count_nonzero(mask) < 3:
        raise ValueError("选定单圈窗口内没有足够的采样点")
    return mask, time[mask] - start, start


def _continuous_euler_radians(euler_deg: np.ndarray) -> np.ndarray:
    return np.unwrap(np.deg2rad(np.asarray(euler_deg, dtype=float)), axis=0)


def _style_time_axis(axis: plt.Axes, lap_duration: float) -> None:
    axis.set_xlim(0.0, lap_duration)
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
    # 三维位置图使用 North-East-Up 显示方式：NED 的 z 向下，画图时取负
    # 得到直观的正高度；日志和其他误差/动力学计算仍保留原始 NED 坐标。
    display_points = points.copy()
    display_points[:, 2] *= -1.0
    lower = np.min(display_points, axis=0)
    upper = np.max(display_points, axis=0)
    margins = np.array([0.45, 0.45, 0.35])
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
        position_ref[:, 1],
        position_ref[:, 0],
        -position_ref[:, 2],
        color=reference_color,
        linewidth=2.0,
        label="Reference",
    )
    axis.plot(
        position[:, 1],
        position[:, 0],
        -position[:, 2],
        color=actual_color,
        linewidth=1.15,
        label="Simulation",
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
    mask: np.ndarray,
    local_time: np.ndarray,
    title: str,
    component_colors: tuple[str, str, str],
    lap_duration: float,
) -> None:
    error = np.asarray(data["position_ref"]) - np.asarray(data["position"])
    labels = (
        r"$x_{\mathrm{ref}}-x$",
        r"$y_{\mathrm{ref}}-y$",
        r"$z_{\mathrm{ref}}-z$",
    )
    for index, (color, label) in enumerate(zip(component_colors, labels)):
        axis.plot(local_time, error[mask, index], color=color, label=label)
    axis.set_title(title, loc="left")
    axis.set_ylabel("Error [m]")
    axis.legend(loc="best", ncol=3)
    _style_time_axis(axis, lap_duration)


def _plot_speed(
    axis: plt.Axes,
    data: dict[str, np.ndarray | None],
    mask: np.ndarray,
    local_time: np.ndarray,
    title: str,
    reference_color: str,
    actual_color: str,
    lap_duration: float,
) -> None:
    speed = np.linalg.norm(np.asarray(data["velocity"]), axis=1)
    speed_ref = np.linalg.norm(np.asarray(data["velocity_ref"]), axis=1)
    axis.plot(
        local_time,
        speed_ref[mask],
        color=reference_color,
        linewidth=2.0,
        label="Reference",
    )
    axis.plot(
        local_time,
        speed[mask],
        color=actual_color,
        linewidth=1.35,
        label="Simulation",
    )
    axis.set_title(title, loc="left")
    axis.set_ylabel(r"$\|v\|$ [m/s]")
    axis.legend(loc="best")
    _style_time_axis(axis, lap_duration)


def _plot_acceleration(
    axis: plt.Axes,
    data: dict[str, np.ndarray | None],
    mask: np.ndarray,
    local_time: np.ndarray,
    title: str,
    reference_color: str,
    actual_color: str,
    lap_duration: float,
) -> None:
    specific_force = np.linalg.norm(
        np.asarray(data["acceleration"]) - GRAVITY_NED, axis=1
    )
    specific_force_ref = np.linalg.norm(
        np.asarray(data["acceleration_ref"]) - GRAVITY_NED, axis=1
    )
    axis.plot(
        local_time,
        specific_force_ref[mask],
        color=reference_color,
        linewidth=2.0,
        label="Reference",
    )
    axis.plot(
        local_time,
        specific_force[mask],
        color=actual_color,
        linewidth=1.25,
        label="Simulation",
    )
    axis.set_title(title, loc="left")
    axis.set_ylabel(r"$\|a-g i_z\|$ [m/s$^2$]")
    axis.legend(loc="best")
    _style_time_axis(axis, lap_duration)


def _plot_attitude(
    axis: plt.Axes,
    data: dict[str, np.ndarray | None],
    mask: np.ndarray,
    local_time: np.ndarray,
    title: str,
    component_colors: tuple[str, str, str],
    lap_duration: float,
) -> None:
    actual = _continuous_euler_radians(np.asarray(data["euler_deg"])[mask])
    command_deg = data["euler_command_deg"]
    symbols = (r"$\psi$", r"$\phi$", r"$\theta$")
    if command_deg is not None:
        command = _continuous_euler_radians(np.asarray(command_deg)[mask])
        offsets = 2.0 * np.pi * np.round(
            (command[0] - actual[0]) / (2.0 * np.pi)
        )
        actual += offsets
        for index, (color, symbol) in enumerate(zip(component_colors, symbols)):
            axis.plot(
                local_time,
                command[:, index],
                color=color,
                linewidth=1.9,
                label=rf"{symbol[:-1]}_c$",
            )
            axis.plot(
                local_time,
                actual[:, index],
                color=color,
                linewidth=1.15,
                linestyle="--",
                label=symbol,
            )
    else:
        for index, (color, symbol) in enumerate(zip(component_colors, symbols)):
            axis.plot(local_time, actual[:, index], color=color, label=symbol)
    axis.set_title(title, loc="left")
    axis.set_ylabel("ZXY Euler angle [rad]")
    axis.legend(loc="best", ncol=3)
    _style_time_axis(axis, lap_duration)


def _case_stats(
    data: dict[str, np.ndarray | None], mask: np.ndarray
) -> dict[str, float]:
    error_norm = np.linalg.norm(
        np.asarray(data["position_ref"]) - np.asarray(data["position"]), axis=1
    )
    speed = np.linalg.norm(np.asarray(data["velocity"])[mask], axis=1)
    specific_force = np.linalg.norm(
        np.asarray(data["acceleration"])[mask] - GRAVITY_NED, axis=1
    )
    return {
        "rms_error": float(np.sqrt(np.mean(error_norm**2))),
        "max_error": float(np.max(error_norm)),
        "lap_rms_error": float(np.sqrt(np.mean(error_norm[mask] ** 2))),
        "lap_mean_speed": float(np.mean(speed)),
        "lap_mean_specific_force": float(np.mean(specific_force)),
    }


def create_figure(
    coordinated: dict[str, np.ndarray | None],
    knife_edge: dict[str, np.ndarray | None],
    lap_duration: float,
    requested_lap: int | None,
) -> tuple[plt.Figure, dict[str, dict[str, float] | float]]:
    if lap_duration <= 0.0:
        raise ValueError("单圈时间必须大于 0")

    # Fig. 8i/j 的 yaw 都从 +pi 连续下降到 -pi。由于两种构型的 yaw
    # 定义相差 pi/2，coordinated 数据窗口相对基础圈偏移 1/4 圈。
    coordinated_phase = 0.25
    knife_edge_phase = 0.0
    max_lap = min(
        _maximum_window_lap(
            np.asarray(coordinated["time"]), lap_duration, coordinated_phase
        ),
        _maximum_window_lap(
            np.asarray(knife_edge["time"]), lap_duration, knife_edge_phase
        ),
    )
    lap = max_lap if requested_lap is None else requested_lap
    if not 1 <= lap <= max_lap:
        raise ValueError(f"--lap 必须位于 1～{max_lap}，当前为 {lap}")

    coordinated_mask, coordinated_time, coordinated_start = _select_lap_window(
        np.asarray(coordinated["time"]), lap_duration, lap, coordinated_phase
    )
    knife_edge_mask, knife_edge_time, knife_edge_start = _select_lap_window(
        np.asarray(knife_edge["time"]), lap_duration, lap, knife_edge_phase
    )

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
    position_limits = _shared_position_limits((coordinated, knife_edge))

    coordinated_position_axis = figure.add_subplot(grid[0, 0], projection="3d")
    knife_edge_position_axis = figure.add_subplot(grid[0, 1], projection="3d")
    _plot_position(
        coordinated_position_axis,
        coordinated,
        "a) Position — Coordinated flight",
        position_limits,
        reference_color,
        actual_color,
    )
    _plot_position(
        knife_edge_position_axis,
        knife_edge,
        "b) Position — Knife-edge flight",
        position_limits,
        reference_color,
        actual_color,
    )

    coordinated_error_axis = figure.add_subplot(grid[1, 0])
    knife_edge_error_axis = figure.add_subplot(grid[1, 1])
    _plot_error(
        coordinated_error_axis,
        coordinated,
        coordinated_mask,
        coordinated_time,
        "c) Position tracking error",
        component_colors,
        lap_duration,
    )
    _plot_error(
        knife_edge_error_axis,
        knife_edge,
        knife_edge_mask,
        knife_edge_time,
        "d) Position tracking error",
        component_colors,
        lap_duration,
    )

    coordinated_speed_axis = figure.add_subplot(grid[2, 0])
    knife_edge_speed_axis = figure.add_subplot(grid[2, 1])
    _plot_speed(
        coordinated_speed_axis,
        coordinated,
        coordinated_mask,
        coordinated_time,
        "e) Speed",
        reference_color,
        actual_color,
        lap_duration,
    )
    _plot_speed(
        knife_edge_speed_axis,
        knife_edge,
        knife_edge_mask,
        knife_edge_time,
        "f) Speed",
        reference_color,
        actual_color,
        lap_duration,
    )

    coordinated_acceleration_axis = figure.add_subplot(grid[3, 0])
    knife_edge_acceleration_axis = figure.add_subplot(grid[3, 1])
    _plot_acceleration(
        coordinated_acceleration_axis,
        coordinated,
        coordinated_mask,
        coordinated_time,
        "g) Acceleration",
        reference_color,
        actual_color,
        lap_duration,
    )
    _plot_acceleration(
        knife_edge_acceleration_axis,
        knife_edge,
        knife_edge_mask,
        knife_edge_time,
        "h) Acceleration",
        reference_color,
        actual_color,
        lap_duration,
    )

    coordinated_attitude_axis = figure.add_subplot(grid[4, 0])
    knife_edge_attitude_axis = figure.add_subplot(grid[4, 1])
    _plot_attitude(
        coordinated_attitude_axis,
        coordinated,
        coordinated_mask,
        coordinated_time,
        "i) Attitude",
        component_colors,
        lap_duration,
    )
    _plot_attitude(
        knife_edge_attitude_axis,
        knife_edge,
        knife_edge_mask,
        knife_edge_time,
        "j) Attitude",
        component_colors,
        lap_duration,
    )

    figure.suptitle(
        "Circle Trajectory at 7.8 m/s — Coordinated vs Knife-Edge Flight",
        fontsize=15,
    )
    return figure, {
        "lap": float(lap),
        "coordinated_start": float(coordinated_start),
        "knife_edge_start": float(knife_edge_start),
        "coordinated": _case_stats(coordinated, coordinated_mask),
        "knife_edge": _case_stats(knife_edge, knife_edge_mask),
    }


def main() -> None:
    args = _parse_args()
    if args.radius <= 0.0 or args.reference_speed <= 0.0:
        raise ValueError("--radius 和 --reference-speed 必须大于 0")
    lap_duration = (
        2.0 * np.pi * args.radius / args.reference_speed
        if args.lap_duration is None
        else args.lap_duration
    )

    coordinated = _load_log(args.coordinated_log.resolve())
    knife_edge = _load_log(args.knife_edge_log.resolve())
    figure, stats = create_figure(
        coordinated, knife_edge, lap_duration, args.lap
    )

    output_path = args.output.resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=args.dpi, bbox_inches="tight")
    print(f"结果图已保存到：{output_path}")
    print(
        f"时域图使用基础第 {int(stats['lap'])} 圈；"
        f"coordinated 起点 {stats['coordinated_start']:.3f} s，"
        f"knife-edge 起点 {stats['knife_edge_start']:.3f} s"
    )
    for name, label in (
        ("coordinated", "Coordinated"),
        ("knife_edge", "Knife-edge"),
    ):
        case = stats[name]
        print(
            f"{label}: 全程位置误差 RMS = {case['rms_error']:.3f} m，"
            f"最大 = {case['max_error']:.3f} m；单圈平均速度 = "
            f"{case['lap_mean_speed']:.3f} m/s，平均比力 = "
            f"{case['lap_mean_specific_force']:.3f} m/s^2"
        )

    if args.show:
        plt.show()
    else:
        plt.close(figure)


if __name__ == "__main__":
    main()
