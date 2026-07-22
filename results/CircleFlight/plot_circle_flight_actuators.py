#!/usr/bin/env python3
"""绘制论文 Fig. 9 风格的圆轨迹执行器与角加速度结果。"""

from __future__ import annotations

import argparse
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
DEFAULT_OUTPUT = SCRIPT_DIR / "circle_flight_actuators_and_angular_acceleration.png"


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="生成论文 Fig. 9 风格的圆轨迹执行器和角加速度十联图"
    )
    parser.add_argument(
        "--coordinated-log", type=Path, default=DEFAULT_COORDINATED_LOG
    )
    parser.add_argument(
        "--knife-edge-log", type=Path, default=DEFAULT_KNIFE_EDGE_LOG
    )
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--lap",
        type=int,
        help="基础圈数，从 1 开始；默认选择最后一个可完成相位对齐的圈",
    )
    parser.add_argument("--radius", type=float, default=3.5)
    parser.add_argument("--reference-speed", type=float, default=8.1)
    parser.add_argument(
        "--lap-duration",
        type=float,
        help="单圈时间；默认由 2*pi*radius/reference-speed 计算",
    )
    parser.add_argument(
        "--max-command-shift-ms",
        type=float,
        default=70.0,
        help=(
            "每个角加速度指令允许向后移动的最大响应时间，论文为最多 "
            "70 ms；设为 0 可关闭自动时移"
        ),
    )
    parser.add_argument("--dpi", type=int, default=180)
    parser.add_argument("--show", action="store_true")
    return parser.parse_args()


def _load_log(path: Path) -> dict[str, np.ndarray]:
    if not path.is_file():
        raise FileNotFoundError(f"找不到仿真日志：{path}")
    required = (
        "time",
        "rotor_speed",
        "flap_deg",
        "angular_acceleration_command",
    )
    with np.load(path, allow_pickle=False) as archive:
        missing = [name for name in required if name not in archive]
        if missing:
            raise KeyError(
                f"{path.name} 缺少字段：{', '.join(missing)}。请用当前 "
                "main_sim.py 重新生成日志。"
            )
        actual_key = (
            "angular_acceleration_lpf"
            if "angular_acceleration_lpf" in archive
            else "angular_acceleration"
        )
        if actual_key not in archive:
            raise KeyError(f"{path.name} 缺少实际角加速度字段")
        data = {
            name: np.asarray(archive[name], dtype=float) for name in required
        }
        data["angular_acceleration_actual"] = np.asarray(
            archive[actual_key], dtype=float
        )

    time = data["time"]
    if time.ndim != 1 or time.size < 3 or not np.all(np.diff(time) > 0.0):
        raise ValueError(f"{path.name} 的 time 必须一维且严格递增")
    expected_shapes = {
        "rotor_speed": (time.size, 2),
        "flap_deg": (time.size, 2),
        "angular_acceleration_command": (time.size, 3),
        "angular_acceleration_actual": (time.size, 3),
    }
    for name, shape in expected_shapes.items():
        if data[name].shape != shape:
            raise ValueError(
                f"{path.name}: {name} 应为 {shape}，实际为 {data[name].shape}"
            )
        if not np.all(np.isfinite(data[name])):
            raise ValueError(f"{path.name}: {name} 包含非有限值")
    return data


def _maximum_window_lap(
    time: np.ndarray, lap_duration: float, phase_fraction: float
) -> int:
    sample_period = float(np.median(np.diff(time)))
    duration = float(time[-1] - time[0] + sample_period)
    return int(np.floor(duration / lap_duration - phase_fraction + 1e-8))


def _select_window(
    time: np.ndarray,
    lap_duration: float,
    lap: int,
    phase_fraction: float,
) -> tuple[np.ndarray, np.ndarray, float]:
    start = time[0] + (lap - 1 + phase_fraction) * lap_duration
    stop = start + lap_duration
    sample_period = float(np.median(np.diff(time)))
    if stop > time[-1] + sample_period:
        raise ValueError("请求的单圈时域窗口超出日志范围")
    mask = (time >= start - 0.25 * sample_period) & (
        time < stop - 0.25 * sample_period
    )
    if np.count_nonzero(mask) < 3:
        raise ValueError("单圈窗口内没有足够的采样点")
    return mask, time[mask] - start, start


def _estimate_delay_samples(
    command: np.ndarray,
    actual: np.ndarray,
    max_samples: int,
) -> int:
    """估计 actual 相对 command 的非负整数采样延迟。"""
    best_delay = 0
    best_loss = np.inf
    maximum = min(max_samples, command.size - 3)
    for delay in range(maximum + 1):
        if delay == 0:
            command_slice = command
            actual_slice = actual
        else:
            command_slice = command[:-delay]
            actual_slice = actual[delay:]

        command_std = float(np.std(command_slice))
        actual_std = float(np.std(actual_slice))
        if command_std < 1e-10 or actual_std < 1e-10:
            scale = max(command_std, actual_std, 1e-10)
            loss = float(
                np.mean((command_slice - actual_slice) ** 2) / scale**2
            )
        else:
            command_normalized = (
                command_slice - np.mean(command_slice)
            ) / command_std
            actual_normalized = (
                actual_slice - np.mean(actual_slice)
            ) / actual_std
            loss = float(np.mean((command_normalized - actual_normalized) ** 2))
        if loss < best_loss:
            best_loss = loss
            best_delay = delay
    return best_delay


def _style_axis(axis: plt.Axes, lap_duration: float) -> None:
    axis.set_xlim(0.0, lap_duration)
    axis.set_xlabel("Time [s]")
    axis.grid(True, color="0.84", linewidth=0.7)
    axis.tick_params(direction="in", top=True, right=True)


def _plot_rotors(
    axis: plt.Axes,
    data: dict[str, np.ndarray],
    mask: np.ndarray,
    local_time: np.ndarray,
    title: str,
    colors: tuple[str, str],
    lap_duration: float,
) -> None:
    rotor_speed = data["rotor_speed"][mask]
    axis.plot(local_time, rotor_speed[:, 0], color=colors[0], label=r"$\omega_1$")
    axis.plot(local_time, rotor_speed[:, 1], color=colors[1], label=r"$\omega_2$")
    axis.set_title(title, loc="left")
    axis.set_ylabel(r"$\omega$ [rad/s]")
    axis.legend(loc="best")
    _style_axis(axis, lap_duration)


def _plot_flaps(
    axis: plt.Axes,
    data: dict[str, np.ndarray],
    mask: np.ndarray,
    local_time: np.ndarray,
    title: str,
    colors: tuple[str, str],
    lap_duration: float,
) -> None:
    flap_rad = np.deg2rad(data["flap_deg"][mask])
    axis.plot(local_time, flap_rad[:, 0], color=colors[0], label=r"$\delta_1$")
    axis.plot(local_time, flap_rad[:, 1], color=colors[1], label=r"$\delta_2$")
    axis.set_title(title, loc="left")
    axis.set_ylabel(r"$\delta$ [rad]")
    axis.legend(loc="best")
    _style_axis(axis, lap_duration)


def _plot_angular_acceleration(
    axis: plt.Axes,
    data: dict[str, np.ndarray],
    mask: np.ndarray,
    local_time: np.ndarray,
    component: int,
    title: str,
    colors: tuple[str, str],
    lap_duration: float,
    max_shift_ms: float,
) -> float:
    command = data["angular_acceleration_command"][mask, component]
    actual = data["angular_acceleration_actual"][mask, component]
    sample_period = float(np.median(np.diff(data["time"])))
    max_samples = int(round(1e-3 * max_shift_ms / sample_period))
    delay_samples = _estimate_delay_samples(command, actual, max_samples)
    delay = delay_samples * sample_period

    axis.plot(
        local_time + delay,
        command,
        color=colors[0],
        linewidth=1.8,
        label="Command",
    )
    axis.plot(
        local_time,
        actual,
        color=colors[1],
        linewidth=1.25,
        label="Simulation",
    )
    component_symbol = "xyz"[component]
    axis.set_title(title, loc="left")
    axis.set_ylabel(
        rf"$\dot{{\Omega}}_{component_symbol}$ [rad/s$^2$]"
    )
    axis.legend(loc="best")
    _style_axis(axis, lap_duration)
    return delay


def _case_stats(
    data: dict[str, np.ndarray], mask: np.ndarray
) -> dict[str, np.ndarray]:
    command = data["angular_acceleration_command"][mask]
    actual = data["angular_acceleration_actual"][mask]
    return {
        "mean_rotor_speed": np.mean(data["rotor_speed"][mask], axis=0),
        "mean_flap_rad": np.mean(np.deg2rad(data["flap_deg"][mask]), axis=0),
        "angular_acceleration_rms_error": np.sqrt(
            np.mean((command - actual) ** 2, axis=0)
        ),
    }


def create_figure(
    coordinated: dict[str, np.ndarray],
    knife_edge: dict[str, np.ndarray],
    lap_duration: float,
    requested_lap: int | None,
    max_shift_ms: float,
) -> tuple[plt.Figure, dict[str, object]]:
    if lap_duration <= 0.0 or max_shift_ms < 0.0:
        raise ValueError("单圈时间必须大于 0，最大时移不能为负")

    coordinated_phase = 0.25
    knife_edge_phase = 0.0
    max_lap = min(
        _maximum_window_lap(
            coordinated["time"], lap_duration, coordinated_phase
        ),
        _maximum_window_lap(
            knife_edge["time"], lap_duration, knife_edge_phase
        ),
    )
    lap = max_lap if requested_lap is None else requested_lap
    if not 1 <= lap <= max_lap:
        raise ValueError(f"--lap 必须位于 1～{max_lap}，当前为 {lap}")

    coordinated_mask, coordinated_time, coordinated_start = _select_window(
        coordinated["time"], lap_duration, lap, coordinated_phase
    )
    knife_edge_mask, knife_edge_time, knife_edge_start = _select_window(
        knife_edge["time"], lap_duration, lap, knife_edge_phase
    )

    plt.rcParams.update(
        {
            "font.family": "serif",
            "font.size": 9.5,
            "axes.titlesize": 11,
            "axes.labelsize": 9.5,
            "legend.fontsize": 8,
            "lines.linewidth": 1.4,
        }
    )
    figure, axes = plt.subplots(
        5, 2, figsize=(14.0, 14.5), constrained_layout=True
    )
    figure.get_layout_engine().set(hspace=0.06, wspace=0.05)
    colors = ("#0072B2", "#D55E00")

    _plot_rotors(
        axes[0, 0], coordinated, coordinated_mask, coordinated_time,
        "a) Rotor speed — Coordinated flight", colors, lap_duration
    )
    _plot_rotors(
        axes[0, 1], knife_edge, knife_edge_mask, knife_edge_time,
        "b) Rotor speed — Knife-edge flight", colors, lap_duration
    )
    _plot_flaps(
        axes[1, 0], coordinated, coordinated_mask, coordinated_time,
        "c) Flap deflection", colors, lap_duration
    )
    _plot_flaps(
        axes[1, 1], knife_edge, knife_edge_mask, knife_edge_time,
        "d) Flap deflection", colors, lap_duration
    )

    coordinated_delays = np.zeros(3)
    knife_edge_delays = np.zeros(3)
    panel_letters = (("e", "f"), ("g", "h"), ("i", "j"))
    for component in range(3):
        left_letter, right_letter = panel_letters[component]
        axis_name = "xyz"[component]
        coordinated_delays[component] = _plot_angular_acceleration(
            axes[component + 2, 0],
            coordinated,
            coordinated_mask,
            coordinated_time,
            component,
            f"{left_letter}) Angular acceleration around $b_{axis_name}$",
            colors,
            lap_duration,
            max_shift_ms,
        )
        knife_edge_delays[component] = _plot_angular_acceleration(
            axes[component + 2, 1],
            knife_edge,
            knife_edge_mask,
            knife_edge_time,
            component,
            f"{right_letter}) Angular acceleration around $b_{axis_name}$",
            colors,
            lap_duration,
            max_shift_ms,
        )

    figure.suptitle(
        "Circle Flight Actuators and Angular Acceleration at 7.8 m/s",
        fontsize=15,
    )
    return figure, {
        "lap": lap,
        "coordinated_start": coordinated_start,
        "knife_edge_start": knife_edge_start,
        "coordinated_delays": coordinated_delays,
        "knife_edge_delays": knife_edge_delays,
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
        coordinated,
        knife_edge,
        lap_duration,
        args.lap,
        args.max_command_shift_ms,
    )

    output_path = args.output.resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=args.dpi, bbox_inches="tight")
    print(f"结果图已保存到：{output_path}")
    print(
        f"基础第 {stats['lap']} 圈；coordinated 起点 "
        f"{stats['coordinated_start']:.3f} s，knife-edge 起点 "
        f"{stats['knife_edge_start']:.3f} s"
    )
    for key, label in (
        ("coordinated", "Coordinated"),
        ("knife_edge", "Knife-edge"),
    ):
        case = stats[key]
        delays_ms = 1e3 * np.asarray(stats[f"{key}_delays"])
        print(
            f"{label}: 平均电机转速 {case['mean_rotor_speed']} rad/s，"
            f"平均舵偏 {case['mean_flap_rad']} rad，角加速度 RMS 误差 "
            f"{case['angular_acceleration_rms_error']} rad/s^2，"
            f"指令时移 {delays_ms} ms"
        )

    if args.show:
        plt.show()
    else:
        plt.close(figure)


if __name__ == "__main__":
    main()
