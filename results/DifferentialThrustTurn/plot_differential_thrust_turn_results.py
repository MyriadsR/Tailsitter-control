#!/usr/bin/env python3
"""绘制论文 Fig. 11 风格的 differential-thrust turn 结果。"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parents[1]
DEFAULT_LOG = PROJECT_ROOT / "validation_logs" / "differential-turn.npz"
DEFAULT_OUTPUT = SCRIPT_DIR / "differential_thrust_turn_results.png"


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="生成论文 Fig. 11 风格的差分推力转弯五联结果图"
    )
    parser.add_argument("log", nargs="?", type=Path, default=DEFAULT_LOG)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--plot-duration",
        type=float,
        help="只绘制日志起始后的指定秒数；默认绘制完整 3.3 s 仿真",
    )
    parser.add_argument("--dpi", type=int, default=180)
    parser.add_argument("--show", action="store_true")
    return parser.parse_args()


def _load_log(path: Path) -> dict[str, np.ndarray]:
    if not path.is_file():
        raise FileNotFoundError(
            f"找不到仿真日志：{path}\n"
            "请先运行：python main_sim.py --trajectory differential-turn "
            "--log validation_logs/differential-turn.npz --no-show"
        )
    required = (
        "time",
        "position",
        "position_ref",
        "euler_deg",
        "angular_velocity",
        "rotor_speed",
        "flap_deg",
    )
    with np.load(path, allow_pickle=False) as archive:
        missing = [name for name in required if name not in archive]
        if missing:
            raise KeyError(
                f"日志缺少字段：{', '.join(missing)}。请使用当前 "
                "main_sim.py 重新生成日志。"
            )
        data = {
            name: np.asarray(archive[name], dtype=float) for name in required
        }

    time = data["time"]
    if time.ndim != 1 or time.size < 3 or not np.all(np.diff(time) > 0.0):
        raise ValueError("time 必须一维、严格递增且至少包含三个采样点")
    expected_shapes = {
        "position": (time.size, 3),
        "position_ref": (time.size, 3),
        "euler_deg": (time.size, 3),
        "angular_velocity": (time.size, 3),
        "rotor_speed": (time.size, 2),
        "flap_deg": (time.size, 2),
    }
    for name, shape in expected_shapes.items():
        if data[name].shape != shape:
            raise ValueError(f"{name} 应为 {shape}，实际为 {data[name].shape}")
        if not np.all(np.isfinite(data[name])):
            raise ValueError(f"{name} 包含非有限值")
    return data


def _select_duration(
    time: np.ndarray, requested_duration: float | None
) -> tuple[np.ndarray, np.ndarray, float]:
    duration = float(time[-1]) if requested_duration is None else requested_duration
    if duration <= 0.0:
        raise ValueError("--plot-duration 必须大于 0")
    sample_period = float(np.median(np.diff(time)))
    if duration > time[-1] + sample_period:
        raise ValueError(
            f"--plot-duration={duration:.3f} s 超出日志时长 {time[-1]:.3f} s"
        )
    mask = time <= duration + 0.25 * sample_period
    if np.count_nonzero(mask) < 3:
        raise ValueError("选定时间段内没有足够的采样点")
    return mask, time[mask], duration


def _continuous_euler_radians(euler_deg: np.ndarray) -> np.ndarray:
    return np.unwrap(np.deg2rad(np.asarray(euler_deg, dtype=float)), axis=0)


def _style_time_axis(axis: plt.Axes, duration: float) -> None:
    axis.set_xlim(0.0, duration)
    axis.set_xlabel("Time [s]")
    axis.grid(True, color="0.84", linewidth=0.7)
    axis.tick_params(direction="in", top=True, right=True)


def _plot_position(
    axis: plt.Axes,
    data: dict[str, np.ndarray],
    mask: np.ndarray,
    reference_color: str,
    actual_color: str,
) -> None:
    position = data["position"][mask]
    position_ref = data["position_ref"][mask]
    axis.plot(
        position_ref[:, 1], position_ref[:, 0], -position_ref[:, 2],
        color=reference_color, linewidth=2.0, label="Reference"
    )
    axis.plot(
        position[:, 1], position[:, 0], -position[:, 2],
        color=actual_color, linewidth=1.3, label="Simulation"
    )

    displayed = np.vstack((position_ref, position)).copy()
    displayed[:, 2] *= -1.0
    lower = np.min(displayed, axis=0)
    upper = np.max(displayed, axis=0)
    margins = np.array([0.45, 0.45, 0.35])
    axis.set_xlim(lower[1] - margins[1], upper[1] + margins[1])
    axis.set_ylim(lower[0] - margins[0], upper[0] + margins[0])
    axis.set_zlim(lower[2] - margins[2], upper[2] + margins[2])
    axis.set_box_aspect((1.65, 1.0, 0.58))
    axis.view_init(elev=22.0, azim=-60.0)
    axis.set_title("a) Position", loc="left")
    axis.set_xlabel(r"$y_i$ [m]", labelpad=5)
    axis.set_ylabel(r"$x_i$ [m]", labelpad=5)
    axis.set_zlabel(r"Altitude $-z_i$ [m]", labelpad=5)
    axis.grid(True, color="0.88", linewidth=0.6)
    axis.legend(loc="upper left")


def _plot_attitude(
    axis: plt.Axes,
    time: np.ndarray,
    euler_deg: np.ndarray,
    colors: tuple[str, str, str],
    duration: float,
) -> None:
    euler = _continuous_euler_radians(euler_deg)
    labels = (r"$\psi$", r"$\phi$", r"$\theta$")
    for index, (color, label) in enumerate(zip(colors, labels)):
        axis.plot(time, euler[:, index], color=color, label=label)
    axis.set_title("b) Attitude", loc="left")
    axis.set_ylabel("ZXY Euler angle [rad]")
    axis.legend(loc="best", ncol=3)
    _style_time_axis(axis, duration)


def _plot_angular_velocity(
    axis: plt.Axes,
    time: np.ndarray,
    angular_velocity: np.ndarray,
    colors: tuple[str, str, str],
    duration: float,
) -> None:
    labels = (r"$\Omega_x$", r"$\Omega_y$", r"$\Omega_z$")
    for index, (color, label) in enumerate(zip(colors, labels)):
        axis.plot(time, angular_velocity[:, index], color=color, label=label)
    axis.set_title("c) Angular velocity", loc="left")
    axis.set_ylabel(r"$\Omega$ [rad/s]")
    axis.legend(loc="best", ncol=3)
    _style_time_axis(axis, duration)


def _plot_rotors(
    axis: plt.Axes,
    time: np.ndarray,
    rotor_speed: np.ndarray,
    colors: tuple[str, str],
    duration: float,
) -> None:
    axis.plot(time, rotor_speed[:, 0], color=colors[0], label=r"$\omega_1$")
    axis.plot(time, rotor_speed[:, 1], color=colors[1], label=r"$\omega_2$")
    axis.set_title("d) Rotor speed", loc="left")
    axis.set_ylabel(r"$\omega$ [rad/s]")
    axis.legend(loc="best")
    _style_time_axis(axis, duration)


def _plot_flaps(
    axis: plt.Axes,
    time: np.ndarray,
    flap_deg: np.ndarray,
    colors: tuple[str, str],
    duration: float,
) -> None:
    flap_rad = np.deg2rad(flap_deg)
    axis.plot(time, flap_rad[:, 0], color=colors[0], label=r"$\delta_1$")
    axis.plot(time, flap_rad[:, 1], color=colors[1], label=r"$\delta_2$")
    axis.set_title("e) Flap deflection", loc="left")
    axis.set_ylabel(r"$\delta$ [rad]")
    axis.legend(loc="best")
    _style_time_axis(axis, duration)


def create_figure(
    data: dict[str, np.ndarray], requested_duration: float | None
) -> tuple[plt.Figure, dict[str, float]]:
    mask, time, duration = _select_duration(data["time"], requested_duration)

    plt.rcParams.update(
        {
            "font.family": "serif",
            "font.size": 10,
            "axes.titlesize": 11.5,
            "axes.labelsize": 10,
            "legend.fontsize": 8.5,
            "lines.linewidth": 1.5,
        }
    )
    figure = plt.figure(figsize=(14.0, 10.5), constrained_layout=True)
    figure.get_layout_engine().set(hspace=0.08, wspace=0.06)
    grid = figure.add_gridspec(3, 2, height_ratios=(1.35, 1.0, 1.0))
    component_colors = ("#0072B2", "#D55E00", "#009E73")
    pair_colors = ("#0072B2", "#D55E00")

    position_axis = figure.add_subplot(grid[0, :], projection="3d")
    _plot_position(position_axis, data, mask, pair_colors[0], pair_colors[1])
    _plot_attitude(
        figure.add_subplot(grid[1, 0]), time, data["euler_deg"][mask],
        component_colors, duration
    )
    _plot_angular_velocity(
        figure.add_subplot(grid[1, 1]), time, data["angular_velocity"][mask],
        component_colors, duration
    )
    _plot_rotors(
        figure.add_subplot(grid[2, 0]), time, data["rotor_speed"][mask],
        pair_colors, duration
    )
    _plot_flaps(
        figure.add_subplot(grid[2, 1]), time, data["flap_deg"][mask],
        pair_colors, duration
    )
    figure.suptitle("Differential Thrust Turn", fontsize=15)

    position_error = np.linalg.norm(
        data["position_ref"][mask] - data["position"][mask], axis=1
    )
    angular_velocity = data["angular_velocity"][mask]
    rotor_speed = data["rotor_speed"][mask]
    flap_rad = np.deg2rad(data["flap_deg"][mask])
    return figure, {
        "duration": duration,
        "rms_position_error": float(
            np.sqrt(np.mean(position_error**2))
        ),
        "max_position_error": float(np.max(position_error)),
        "max_angular_rate": float(
            np.max(np.linalg.norm(angular_velocity, axis=1))
        ),
        "max_rotor_speed": float(np.max(rotor_speed)),
        "max_rotor_speed_difference": float(
            np.max(np.abs(rotor_speed[:, 0] - rotor_speed[:, 1]))
        ),
        "max_abs_flap": float(np.max(np.abs(flap_rad))),
    }


def main() -> None:
    args = _parse_args()
    data = _load_log(args.log.resolve())
    figure, stats = create_figure(data, args.plot_duration)

    output_path = args.output.resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=args.dpi, bbox_inches="tight")
    print(f"结果图已保存到：{output_path}")
    print(
        f"绘图时长 {stats['duration']:.3f} s；位置误差 RMS = "
        f"{stats['rms_position_error']:.3f} m，最大 = "
        f"{stats['max_position_error']:.3f} m"
    )
    print(
        f"最大角速度 = {stats['max_angular_rate']:.3f} rad/s "
        f"({np.rad2deg(stats['max_angular_rate']):.1f} deg/s)，"
        f"最大电机转速 = {stats['max_rotor_speed']:.1f} rad/s，"
        f"最大电机转速差 = {stats['max_rotor_speed_difference']:.1f} rad/s，"
        f"最大舵偏 = {stats['max_abs_flap']:.3f} rad"
    )

    if args.show:
        plt.show()
    else:
        plt.close(figure)


if __name__ == "__main__":
    main()
