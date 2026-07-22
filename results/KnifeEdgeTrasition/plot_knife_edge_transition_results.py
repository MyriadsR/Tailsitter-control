#!/usr/bin/env python3
"""绘制论文 Fig. 7 风格的 knife-edge transitioning 轨迹结果。

默认读取 ``validation_logs/knife-edge-transition.npz``，并绘制：

* a) 八圈俯视位置轨迹；
* b) 从 knife-edge 转入倒飞 coordinated flight 开始的两圈位置误差；
* c) 同一两圈内的 ZXY 指令姿态与实际姿态。
"""

from __future__ import annotations

import argparse
import warnings
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parents[1]
DEFAULT_LOG = PROJECT_ROOT / "validation_logs" / "knife-edge-transition.npz"
DEFAULT_OUTPUT = SCRIPT_DIR / "knife_edge_transition_results.png"


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="生成论文 Fig. 7 风格的 knife-edge transitioning 结果图"
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
        "--window-start",
        type=float,
        help=(
            "两圈时域图在原日志中的起始时刻；默认自动对齐第二圈中从 "
            "knife-edge 转入倒飞 coordinated flight 的转弯起点"
        ),
    )
    parser.add_argument(
        "--window-laps",
        type=int,
        default=2,
        help="时域图包含的圈数（论文为 2）",
    )
    parser.add_argument(
        "--lap-duration",
        type=float,
        default=6.25,
        help="单圈时间，论文轨迹为 6.25 s",
    )
    parser.add_argument(
        "--speed", type=float, default=6.0, help="参考速度，论文轨迹为 6 m/s"
    )
    parser.add_argument(
        "--turn-radius",
        type=float,
        default=3.0,
        help="本项目椭圆跑道的平滑转弯半径，用于自动计算窗口起点",
    )
    parser.add_argument("--dpi", type=int, default=180, help="输出图片分辨率")
    parser.add_argument("--show", action="store_true", help="保存后显示绘图窗口")
    return parser.parse_args()


def _load_log(log_path: Path) -> dict[str, np.ndarray | None]:
    if not log_path.is_file():
        raise FileNotFoundError(
            f"找不到仿真日志：{log_path}\n"
            "请先运行：python main_sim.py --trajectory knife-edge-transition "
            "--log validation_logs/knife-edge-transition.npz --no-show"
        )

    required = ("time", "position", "position_ref", "euler_deg")
    with np.load(log_path, allow_pickle=False) as archive:
        missing = [name for name in required if name not in archive]
        if missing:
            raise KeyError(f"日志缺少字段：{', '.join(missing)}")
        data: dict[str, np.ndarray | None] = {
            name: np.asarray(archive[name], dtype=float) for name in required
        }
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
    command = data["euler_command_deg"]
    if command is not None and np.asarray(command).shape != (time.size, 3):
        raise ValueError(
            "euler_command_deg 的形状应为 "
            f"({time.size}, 3)，实际为 {np.asarray(command).shape}"
        )
    if command is None:
        warnings.warn(
            "日志没有 euler_command_deg，姿态子图只能绘制实际姿态；"
            "请使用当前 main_sim.py 重新生成日志以获得完整 Fig. 7。",
            stacklevel=2,
        )
    return data


def _automatic_window_start(
    lap_duration: float, speed: float, turn_radius: float
) -> float:
    """返回第二圈 knife-edge -> inverted coordinated 转弯的起点。"""
    if lap_duration <= 0.0 or speed <= 0.0 or turn_radius <= 0.0:
        raise ValueError("单圈时间、速度和转弯半径必须大于 0")
    path_length = speed * lap_duration
    straight_length = 0.5 * (path_length - 2.0 * np.pi * turn_radius)
    if straight_length <= 0.0:
        raise ValueError("轨迹周长不足以容纳两个指定半径的半圆")
    return lap_duration + straight_length / speed


def _select_window(
    time: np.ndarray,
    start: float,
    duration: float,
) -> tuple[np.ndarray, np.ndarray, float, float]:
    if duration <= 0.0:
        raise ValueError("时域窗口长度必须大于 0")
    sample_period = float(np.median(np.diff(time)))
    stop = start + duration
    if start < time[0] - sample_period or stop > time[-1] + sample_period:
        raise ValueError(
            f"请求窗口 [{start:.3f}, {stop:.3f}] s 超出日志范围 "
            f"[{time[0]:.3f}, {time[-1]:.3f}] s"
        )

    mask = (time >= start - 0.25 * sample_period) & (
        time < stop - 0.25 * sample_period
    )
    if np.count_nonzero(mask) < 3:
        raise ValueError("选定时域窗口内没有足够的采样点")
    local_time = time[mask] - start
    return mask, local_time, start, stop


def _continuous_euler_radians(euler_deg: np.ndarray) -> np.ndarray:
    """将 [psi, phi, theta] 转成连续弧度曲线。"""
    return np.unwrap(np.deg2rad(np.asarray(euler_deg, dtype=float)), axis=0)


def _style_time_axis(axis: plt.Axes, duration: float) -> None:
    axis.set_xlim(0.0, duration)
    axis.set_xlabel("Time [s]")
    axis.grid(True, color="0.84", linewidth=0.7)
    axis.tick_params(direction="in", top=True, right=True)


def create_figure(
    data: dict[str, np.ndarray | None],
    window_start: float,
    window_duration: float,
) -> tuple[plt.Figure, dict[str, float]]:
    time = np.asarray(data["time"])
    position = np.asarray(data["position"])
    position_ref = np.asarray(data["position_ref"])
    euler_actual_deg = np.asarray(data["euler_deg"])
    euler_command_deg = data["euler_command_deg"]

    mask, local_time, window_start, window_stop = _select_window(
        time, window_start, window_duration
    )
    position_error = position_ref - position

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
    figure = plt.figure(figsize=(14.0, 8.4), constrained_layout=True)
    figure.get_layout_engine().set(hspace=0.08)
    grid = figure.add_gridspec(2, 2, height_ratios=(1.12, 1.0))

    reference_color = "#0072B2"
    actual_color = "#D55E00"
    component_colors = ("#0072B2", "#D55E00", "#E1A100")

    # a) 论文 Fig. 7a 是世界水平面的俯视图：横轴 y、纵轴 x，显示八圈。
    axis_position = figure.add_subplot(grid[0, :])
    axis_position.plot(
        position_ref[:, 1],
        position_ref[:, 0],
        color=reference_color,
        linewidth=2.1,
        label="Reference",
    )
    axis_position.plot(
        position[:, 1],
        position[:, 0],
        color=actual_color,
        linewidth=1.05,
        label="Simulation",
    )
    all_y = np.concatenate((position_ref[:, 1], position[:, 1]))
    all_x = np.concatenate((position_ref[:, 0], position[:, 0]))
    y_center = 0.5 * (np.min(all_y) + np.max(all_y))
    x_center = 0.5 * (np.min(all_x) + np.max(all_x))
    y_half = max(0.5 * np.ptp(all_y) + 0.5, 1.0)
    x_half = max(0.5 * np.ptp(all_x) + 0.5, 1.0)
    axis_position.set_xlim(y_center - y_half, y_center + y_half)
    axis_position.set_ylim(x_center - x_half, x_center + x_half)
    axis_position.set_aspect("equal", adjustable="box")
    axis_position.set_title("a) Position", loc="left")
    axis_position.set_xlabel(r"$y_i$ [m]")
    axis_position.set_ylabel(r"$x_i$ [m]")
    axis_position.grid(True, color="0.88", linewidth=0.6)
    axis_position.tick_params(direction="in", top=True, right=True)
    axis_position.legend(loc="lower center", ncol=2)

    axis_error = figure.add_subplot(grid[1, 0])
    error_labels = (
        r"$x_{\mathrm{ref}}-x$",
        r"$y_{\mathrm{ref}}-y$",
        r"$z_{\mathrm{ref}}-z$",
    )
    for index, (color, label) in enumerate(zip(component_colors, error_labels)):
        axis_error.plot(
            local_time,
            position_error[mask, index],
            color=color,
            label=label,
        )
    axis_error.set_title("b) Position tracking error", loc="left")
    axis_error.set_ylabel("Error [m]")
    axis_error.legend(loc="best", ncol=3)
    _style_time_axis(axis_error, window_duration)

    axis_attitude = figure.add_subplot(grid[1, 1])
    euler_actual = _continuous_euler_radians(euler_actual_deg[mask])
    angle_symbols = (r"$\psi$", r"$\phi$", r"$\theta$")
    if euler_command_deg is not None:
        euler_command = _continuous_euler_radians(
            np.asarray(euler_command_deg)[mask]
        )
        offsets = 2.0 * np.pi * np.round(
            (euler_command[0] - euler_actual[0]) / (2.0 * np.pi)
        )
        euler_actual = euler_actual + offsets

        # 欧拉 yaw 可整体移动 2*pi 而不改变姿态；选取非负起点以得到与论文
        # Fig. 7c 相似的连续上升曲线，同时保留世界系中的绝对方向关系。
        yaw_shift = 2.0 * np.pi * np.ceil(
            -euler_command[0, 0] / (2.0 * np.pi)
        )
        euler_command[:, 0] += yaw_shift
        euler_actual[:, 0] += yaw_shift

        for index, (color, symbol) in enumerate(
            zip(component_colors, angle_symbols)
        ):
            axis_attitude.plot(
                local_time,
                euler_command[:, index],
                color=color,
                linewidth=2.0,
                label=rf"{symbol[:-1]}_c$",
            )
            axis_attitude.plot(
                local_time,
                euler_actual[:, index],
                color=color,
                linewidth=1.2,
                linestyle="--",
                label=symbol,
            )
    else:
        for index, (color, symbol) in enumerate(
            zip(component_colors, angle_symbols)
        ):
            axis_attitude.plot(
                local_time,
                euler_actual[:, index],
                color=color,
                label=symbol,
            )
        axis_attitude.text(
            0.02,
            0.04,
            "Command attitude was not recorded",
            transform=axis_attitude.transAxes,
            color="0.35",
        )
    axis_attitude.set_title("c) Attitude", loc="left")
    axis_attitude.set_ylabel("ZXY Euler angle [rad]")
    axis_attitude.legend(loc="best", ncol=3)
    _style_time_axis(axis_attitude, window_duration)

    figure.suptitle(
        "Knife-Edge Transitioning Trajectory Tracking at 6 m/s",
        fontsize=15,
    )

    error_norm = np.linalg.norm(position_error, axis=1)
    window_error_norm = error_norm[mask]
    stats = {
        "window_start": float(window_start),
        "window_stop": float(window_stop),
        "rms_position_error": float(np.sqrt(np.mean(error_norm**2))),
        "max_position_error": float(np.max(error_norm)),
        "window_rms_position_error": float(
            np.sqrt(np.mean(window_error_norm**2))
        ),
        "window_max_position_error": float(np.max(window_error_norm)),
    }
    return figure, stats


def main() -> None:
    args = _parse_args()
    if args.window_laps <= 0:
        raise ValueError("--window-laps 必须是正整数")
    window_start = (
        _automatic_window_start(args.lap_duration, args.speed, args.turn_radius)
        if args.window_start is None
        else args.window_start
    )
    window_duration = args.window_laps * args.lap_duration

    data = _load_log(args.log.resolve())
    figure, stats = create_figure(data, window_start, window_duration)

    output_path = args.output.resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=args.dpi, bbox_inches="tight")
    print(f"结果图已保存到：{output_path}")
    print(
        "全程位置误差："
        f"RMS = {stats['rms_position_error']:.3f} m，"
        f"最大 = {stats['max_position_error']:.3f} m"
    )
    print(
        f"时域窗口 [{stats['window_start']:.3f}, "
        f"{stats['window_stop']:.3f}] s："
        f"RMS = {stats['window_rms_position_error']:.3f} m，"
        f"最大 = {stats['window_max_position_error']:.3f} m"
    )

    if args.show:
        plt.show()
    else:
        plt.close(figure)


if __name__ == "__main__":
    main()
