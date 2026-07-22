#!/usr/bin/env python3
"""绘制论文 Fig. 6 风格的 Lemniscate 轨迹跟踪结果。

默认读取 ``validation_logs/lemniscate.npz``，并在当前目录生成包含
Position、Position tracking error、Speed、Acceleration 和 Attitude
五个子图的结果图。
"""

from __future__ import annotations

import argparse
import warnings
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parents[1]
DEFAULT_LOG = PROJECT_ROOT / "validation_logs" / "lemniscate.npz"
DEFAULT_OUTPUT = SCRIPT_DIR / "lemniscate_trajectory_results.png"
GRAVITY_NED = np.array([0.0, 0.0, 9.81])


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="生成论文 Fig. 6 风格的 Lemniscate 五联跟踪结果图"
    )
    parser.add_argument(
        "log",
        nargs="?",
        type=Path,
        default=DEFAULT_LOG,
        help=f"输入 NPZ 日志（默认：{DEFAULT_LOG}）",
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
        help="时域图使用的圈数，从 1 开始；默认使用最后一个完整单圈",
    )
    parser.add_argument(
        "--lap-duration",
        type=float,
        default=7.0,
        help="单圈持续时间，论文 Lemniscate 默认为 7 s",
    )
    parser.add_argument("--dpi", type=int, default=180, help="输出图片分辨率")
    parser.add_argument("--show", action="store_true", help="保存后显示绘图窗口")
    return parser.parse_args()


def _load_log(log_path: Path) -> dict[str, np.ndarray | None]:
    if not log_path.is_file():
        raise FileNotFoundError(
            f"找不到仿真日志：{log_path}\n"
            "请先运行：python main_sim.py --trajectory lemniscate "
            "--log validation_logs/lemniscate.npz --no-show"
        )

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
            raise KeyError(f"日志缺少字段：{', '.join(missing)}")

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
        raise ValueError("time 必须是一维、严格递增且至少包含三个采样点")

    for name in required[1:]:
        values = np.asarray(data[name])
        if values.shape != (time.size, 3):
            raise ValueError(
                f"{name} 的形状应为 ({time.size}, 3)，实际为 {values.shape}"
            )

    for name in ("acceleration", "euler_command_deg"):
        values = data[name]
        if values is not None and np.asarray(values).shape != (time.size, 3):
            raise ValueError(
                f"{name} 的形状应为 ({time.size}, 3)，"
                f"实际为 {np.asarray(values).shape}"
            )

    if data["acceleration"] is None:
        warnings.warn(
            "旧日志中没有 acceleration，将由 velocity 数值微分估算。"
            "为得到更可靠的曲线，建议重新运行 main_sim.py。",
            stacklevel=2,
        )
        data["acceleration"] = np.gradient(
            np.asarray(data["velocity"]), time, axis=0, edge_order=2
        )

    if data["euler_command_deg"] is None:
        warnings.warn(
            "旧日志中没有 euler_command_deg，姿态子图只能绘制实际姿态。"
            "重新运行 main_sim.py 后可同时显示指令姿态。",
            stacklevel=2,
        )

    return data


def _select_lap(
    time: np.ndarray, lap_duration: float, requested_lap: int | None
) -> tuple[np.ndarray, np.ndarray, int, int]:
    if lap_duration <= 0.0:
        raise ValueError("--lap-duration 必须大于 0")

    sample_period = float(np.median(np.diff(time)))
    recorded_duration = float(time[-1] - time[0] + sample_period)
    complete_laps = int(np.floor(recorded_duration / lap_duration + 1e-8))
    if complete_laps < 1:
        raise ValueError(
            f"日志时长 {recorded_duration:.3f} s 不足一个 {lap_duration:.3f} s 单圈"
        )

    lap = complete_laps if requested_lap is None else requested_lap
    if not 1 <= lap <= complete_laps:
        raise ValueError(f"--lap 必须位于 1～{complete_laps}，当前为 {lap}")

    start = time[0] + (lap - 1) * lap_duration
    stop = start + lap_duration
    mask = (time >= start - 0.25 * sample_period) & (
        time < stop - 0.25 * sample_period
    )
    if np.count_nonzero(mask) < 3:
        raise ValueError(f"第 {lap} 圈没有足够的采样点")

    local_time = time[mask] - start
    return mask, local_time, lap, complete_laps


def _continuous_euler_radians(euler_deg: np.ndarray) -> np.ndarray:
    """将日志中的 [psi, phi, theta] 转成连续弧度曲线。"""
    euler_rad = np.deg2rad(np.asarray(euler_deg, dtype=float))
    return np.unwrap(euler_rad, axis=0)


def _ned_to_display(vectors: np.ndarray) -> np.ndarray:
    """与 animate_flight.py 一致：将 NED 的向下 Z 改成显示用的高度。"""
    displayed = np.array(vectors, dtype=float, copy=True)
    displayed[..., 2] *= -1.0
    return displayed


def _set_equal_3d_limits(
    axis: plt.Axes, points: np.ndarray, margin: float = 1.8
) -> None:
    """采用 animate_flight.py 的等比例三维范围，避免轨迹形状失真。"""
    minimum = np.min(points, axis=0)
    maximum = np.max(points, axis=0)
    center = 0.5 * (minimum + maximum)
    half_range = max(0.5 * np.max(maximum - minimum) + margin, margin, 1.0)

    axis.set_xlim(center[0] - half_range, center[0] + half_range)
    axis.set_ylim(center[1] - half_range, center[1] + half_range)
    axis.set_zlim(center[2] - half_range, center[2] + half_range)
    axis.set_box_aspect((1.0, 1.0, 1.0))


def _style_time_axis(axis: plt.Axes, duration: float) -> None:
    axis.set_xlim(0.0, duration)
    axis.grid(True, color="0.84", linewidth=0.7)
    axis.tick_params(direction="in", top=True, right=True)


def create_figure(
    data: dict[str, np.ndarray | None],
    lap_duration: float,
    requested_lap: int | None,
) -> tuple[plt.Figure, dict[str, float | int]]:
    time = np.asarray(data["time"])
    position = np.asarray(data["position"])
    velocity = np.asarray(data["velocity"])
    acceleration = np.asarray(data["acceleration"])
    position_ref = np.asarray(data["position_ref"])
    velocity_ref = np.asarray(data["velocity_ref"])
    acceleration_ref = np.asarray(data["acceleration_ref"])
    euler_actual_deg = np.asarray(data["euler_deg"])
    euler_command_deg = data["euler_command_deg"]

    mask, lap_time, lap, complete_laps = _select_lap(
        time, lap_duration, requested_lap
    )

    position_error = position_ref - position
    speed = np.linalg.norm(velocity, axis=1)
    speed_ref = np.linalg.norm(velocity_ref, axis=1)
    specific_force = np.linalg.norm(acceleration - GRAVITY_NED, axis=1)
    specific_force_ref = np.linalg.norm(acceleration_ref - GRAVITY_NED, axis=1)

    plt.rcParams.update(
        {
            "font.family": "serif",
            "font.size": 10.5,
            "axes.titlesize": 12,
            "axes.labelsize": 10.5,
            "legend.fontsize": 9,
            "lines.linewidth": 1.6,
        }
    )
    figure = plt.figure(figsize=(14.0, 12.2), constrained_layout=True)
    figure.get_layout_engine().set(hspace=0.08)
    grid = figure.add_gridspec(3, 2, height_ratios=(1.5, 1.0, 1.0))

    reference_color = "#0072B2"
    actual_color = "#D55E00"
    component_colors = ("#0072B2", "#D55E00", "#009E73")

    # a) 沿用 animate_flight.py 的 North-East-Up 显示坐标、等比例范围和视角。
    # 不交换 X/Y，也不单独压缩 Z，否则透视投影会掩盖双纽线的真实形状。
    position_display = _ned_to_display(position)
    position_ref_display = _ned_to_display(position_ref)
    axis_position = figure.add_subplot(grid[0, :], projection="3d")
    axis_position.plot(
        position_ref_display[:, 0],
        position_ref_display[:, 1],
        position_ref_display[:, 2],
        color=reference_color,
        linestyle="--",
        linewidth=1.8,
        label="Reference",
    )
    axis_position.plot(
        position_display[:, 0],
        position_display[:, 1],
        position_display[:, 2],
        color=actual_color,
        linewidth=1.4,
        label="Simulation",
    )
    axis_position.set_title("a) Position", loc="left", pad=8)
    axis_position.set_xlabel("North X [m]", labelpad=5)
    axis_position.set_ylabel("East Y [m]", labelpad=5)
    axis_position.set_zlabel(r"Altitude $-Z_{NED}$ [m]", labelpad=6)
    axis_position.view_init(elev=25.0, azim=-55.0)
    _set_equal_3d_limits(
        axis_position,
        np.vstack((position_ref_display, position_display)),
    )
    axis_position.grid(True, color="0.88", linewidth=0.6)
    axis_position.legend(loc="upper right")

    axis_error = figure.add_subplot(grid[1, 0])
    error_labels = (
        r"$x_{\mathrm{ref}}-x$",
        r"$y_{\mathrm{ref}}-y$",
        r"$z_{\mathrm{ref}}-z$",
    )
    for index, (color, label) in enumerate(zip(component_colors, error_labels)):
        axis_error.plot(lap_time, position_error[mask, index], color=color, label=label)
    axis_error.set_title("b) Position tracking error", loc="left")
    axis_error.set_ylabel("Error [m]")
    axis_error.legend(loc="best", ncol=3)
    _style_time_axis(axis_error, lap_duration)

    axis_speed = figure.add_subplot(grid[1, 1])
    axis_speed.plot(
        lap_time,
        speed_ref[mask],
        color=reference_color,
        linewidth=2.2,
        label="Reference",
    )
    axis_speed.plot(
        lap_time,
        speed[mask],
        color=actual_color,
        linewidth=1.4,
        label="Simulation",
    )
    axis_speed.set_title("c) Speed", loc="left")
    axis_speed.set_ylabel(r"$\|v\|$ [m/s]")
    axis_speed.legend(loc="best")
    _style_time_axis(axis_speed, lap_duration)

    axis_acceleration = figure.add_subplot(grid[2, 0])
    axis_acceleration.plot(
        lap_time,
        specific_force_ref[mask],
        color=reference_color,
        linewidth=2.2,
        label="Reference",
    )
    axis_acceleration.plot(
        lap_time,
        specific_force[mask],
        color=actual_color,
        linewidth=1.4,
        label="Simulation",
    )
    axis_acceleration.set_title("d) Acceleration", loc="left")
    axis_acceleration.set_xlabel("Time [s]")
    axis_acceleration.set_ylabel(r"$\|a-g i_z\|$ [m/s$^2$]")
    axis_acceleration.legend(loc="best")
    _style_time_axis(axis_acceleration, lap_duration)

    axis_attitude = figure.add_subplot(grid[2, 1])
    euler_actual = _continuous_euler_radians(euler_actual_deg[mask])
    angle_symbols = (r"$\psi$", r"$\phi$", r"$\theta$")
    if euler_command_deg is not None:
        euler_command = _continuous_euler_radians(
            np.asarray(euler_command_deg)[mask]
        )
        # 四元数转欧拉角后可能相差 2*pi；只移动整周，不改变实际姿态。
        offsets = 2.0 * np.pi * np.round(
            (euler_command[0] - euler_actual[0]) / (2.0 * np.pi)
        )
        euler_actual = euler_actual + offsets
        for index, (color, symbol) in enumerate(
            zip(component_colors, angle_symbols)
        ):
            axis_attitude.plot(
                lap_time,
                euler_command[:, index],
                color=color,
                linewidth=2.0,
                label=rf"{symbol[:-1]}_c$",
            )
            axis_attitude.plot(
                lap_time,
                euler_actual[:, index],
                color=color,
                linewidth=1.25,
                linestyle="--",
                label=symbol,
            )
    else:
        for index, (color, symbol) in enumerate(
            zip(component_colors, angle_symbols)
        ):
            axis_attitude.plot(
                lap_time, euler_actual[:, index], color=color, label=symbol
            )
        axis_attitude.text(
            0.02,
            0.04,
            "Command attitude was not recorded",
            transform=axis_attitude.transAxes,
            color="0.35",
        )
    axis_attitude.set_title("e) Attitude", loc="left")
    axis_attitude.set_xlabel("Time [s]")
    axis_attitude.set_ylabel("ZXY Euler angle [rad]")
    axis_attitude.legend(loc="best", ncol=3)
    _style_time_axis(axis_attitude, lap_duration)

    figure.suptitle(
        f"Lemniscate Trajectory Tracking — lap {lap}/{complete_laps}",
        fontsize=15,
    )

    error_norm = np.linalg.norm(position_error, axis=1)
    stats: dict[str, float | int] = {
        "lap": lap,
        "complete_laps": complete_laps,
        "rms_position_error": float(np.sqrt(np.mean(error_norm**2))),
        "max_position_error": float(np.max(error_norm)),
    }
    return figure, stats


def main() -> None:
    args = _parse_args()
    data = _load_log(args.log.resolve())
    figure, stats = create_figure(data, args.lap_duration, args.lap)

    output_path = args.output.resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=args.dpi, bbox_inches="tight")
    print(f"结果图已保存到：{output_path}")
    print(
        "全程位置误差："
        f"RMS = {stats['rms_position_error']:.3f} m，"
        f"最大 = {stats['max_position_error']:.3f} m；"
        f"时域图使用第 {stats['lap']}/{stats['complete_laps']} 圈"
    )

    if args.show:
        plt.show()
    else:
        plt.close(figure)


if __name__ == "__main__":
    main()
