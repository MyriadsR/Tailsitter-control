#!/usr/bin/env python3
"""Visualize trajectory and open-loop planning metrics exported as CSV."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys
import tempfile

# The plotting command is often run in a restricted/headless environment. Keep
# Matplotlib and fontconfig caches out of an unwritable user configuration path.
_PLOT_CACHE = Path(tempfile.gettempdir()) / "tailsitter_planning_plot_cache"
os.environ.setdefault("MPLCONFIGDIR", str(_PLOT_CACHE / "matplotlib"))
os.environ.setdefault("XDG_CACHE_HOME", str(_PLOT_CACHE / "xdg"))
if "--show" not in sys.argv:
    os.environ.setdefault("MPLBACKEND", "Agg")

import matplotlib.pyplot as plt
from matplotlib import colors
from matplotlib.lines import Line2D
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import numpy as np


REQUIRED_COLUMNS = (
    "t",
    "px",
    "py",
    "pz",
    "vx",
    "vy",
    "vz",
    "snap_x",
    "snap_y",
    "snap_z",
    "yaw_acceleration",
    "omega1_pred",
    "omega2_pred",
    "delta1_pred",
    "delta2_pred",
)

ACTUATOR_COLUMNS = (
    "omega1_pred",
    "omega2_pred",
    "delta1_pred",
    "delta2_pred",
)

DYNAMIC_COLUMNS = (
    "body_rate_norm",
    "rotor_rate_1",
    "rotor_rate_2",
    "flap_rate_1",
    "flap_rate_2",
    "omega_cmd_req_1",
    "omega_cmd_req_2",
    "delta_cmd_req_1",
    "delta_cmd_req_2",
)

HIGH_LEVEL_COLUMNS = ("collective_thrust",)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "绘制规划 CSV：速度着色三维轨迹、Snap/yaw 加速度和开环执行器预测"
        )
    )
    parser.add_argument("csv", type=Path, help="generate_simple_trajectory 导出的 CSV")
    parser.add_argument(
        "--compare-csv",
        type=Path,
        help="另一份规划 CSV；给出后生成两种时间分配方法的对比图",
    )
    parser.add_argument(
        "--compare-csv-2",
        type=Path,
        help="可选的第三份规划 CSV；用于三种规划配置的对比图",
    )
    parser.add_argument(
        "--primary-label", default="Optimized time ratio", help="主 CSV 的图例名称"
    )
    parser.add_argument(
        "--compare-label", default="Fixed distance ratio", help="对比 CSV 的图例名称"
    )
    parser.add_argument(
        "--compare-label-2",
        default="No outer time scaling",
        help="第三份 CSV 的图例名称",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="输出目录；默认使用 CSV 同级目录下的 figures/",
    )
    parser.add_argument("--prefix", help="输出文件名前缀；默认使用 CSV 文件名")
    parser.add_argument("--title", help="图标题中的轨迹名称；默认使用 CSV 文件名")
    parser.add_argument(
        "--omega-max", type=float, default=2500.0, help="电机转速上限 [rad/s]"
    )
    parser.add_argument(
        "--delta-max", type=float, default=1.0, help="舵面绝对偏角上限 [rad]"
    )
    parser.add_argument(
        "--body-rate-max",
        type=float,
        default=650.0 * np.pi / 180.0,
        help="机体角速度范数上限 [rad/s]",
    )
    parser.add_argument(
        "--servo-rate-max", type=float, default=25.0, help="舵面角速度上限 [rad/s]"
    )
    parser.add_argument(
        "--velocity-max", type=float, default=8.0, help="速度范数上限 [m/s]"
    )
    parser.add_argument(
        "--no-velocity-limit",
        action="store_true",
        help="高层约束图只画速度曲线，不画速度限制线",
    )
    parser.add_argument(
        "--collective-thrust-max",
        type=float,
        default=20.25,
        help="总推力上限 [N]",
    )
    parser.add_argument(
        "--cmap", default="viridis", help="三维轨迹速度 colormap，默认 viridis"
    )
    parser.add_argument(
        "--raw-ned-z",
        action="store_true",
        help="三维图直接显示 NED z；默认显示正高度 -z",
    )
    parser.add_argument(
        "--format",
        choices=("png", "pdf", "svg"),
        default="png",
        help="输出格式，默认 png",
    )
    parser.add_argument("--dpi", type=int, default=200, help="位图分辨率，默认 200")
    parser.add_argument("--show", action="store_true", help="保存后显示窗口")
    return parser.parse_args()


def _load_csv(path: Path) -> dict[str, np.ndarray]:
    if not path.is_file():
        raise FileNotFoundError(f"找不到规划 CSV：{path}")

    table = np.genfromtxt(path, delimiter=",", names=True, dtype=float)
    if table.dtype.names is None:
        raise ValueError(f"{path} 没有可识别的 CSV 表头")
    missing = [name for name in REQUIRED_COLUMNS if name not in table.dtype.names]
    if missing:
        raise KeyError(f"{path.name} 缺少字段：{', '.join(missing)}")

    table = np.atleast_1d(table)
    if table.size < 2:
        raise ValueError(f"{path.name} 至少需要两个采样点")

    optional_columns = DYNAMIC_COLUMNS + HIGH_LEVEL_COLUMNS
    loaded_columns = list(REQUIRED_COLUMNS) + [
        name for name in optional_columns if name in table.dtype.names
    ]
    data = {
        name: np.asarray(table[name], dtype=float).reshape(-1)
        for name in loaded_columns
    }
    time = data["t"]
    if not np.all(np.isfinite(time)) or not np.all(np.diff(time) > 0.0):
        raise ValueError(f"{path.name} 的时间必须有限且严格递增")
    for name, values in data.items():
        if values.shape != time.shape:
            raise ValueError(f"{path.name} 的字段 {name} 维度错误")
        # An infeasible open-loop mapping deliberately exports NaN actuator
        # values. Preserve them as visible gaps instead of rejecting the valid
        # trajectory-state columns or drawing a misleading connecting line.
        if (
            name not in ACTUATOR_COLUMNS
            and name not in DYNAMIC_COLUMNS
            and name not in HIGH_LEVEL_COLUMNS
            and not np.all(np.isfinite(values))
        ):
            raise ValueError(f"{path.name} 的字段 {name} 包含非有限值")
    return data


def _apply_style() -> None:
    plt.rcParams.update(
        {
            "font.size": 10,
            "axes.labelsize": 10,
            "axes.titlesize": 12,
            "legend.fontsize": 9,
            "lines.linewidth": 1.5,
            "axes.grid": True,
            "grid.color": "0.86",
            "grid.linewidth": 0.7,
            "xtick.direction": "in",
            "ytick.direction": "in",
        }
    )


def _summary_metrics(data: dict[str, np.ndarray]) -> dict[str, float]:
    velocity = np.column_stack((data["vx"], data["vy"], data["vz"]))
    snap = np.column_stack((data["snap_x"], data["snap_y"], data["snap_z"]))

    def finite_max(values: np.ndarray, absolute: bool = False) -> float:
        finite = values[np.isfinite(values)]
        if finite.size == 0:
            return float("nan")
        return float(np.max(np.abs(finite) if absolute else finite))

    peak_rotor = finite_max(
        np.concatenate((data["omega1_pred"], data["omega2_pred"]))
    )
    peak_flap = finite_max(
        np.concatenate((data["delta1_pred"], data["delta2_pred"])),
        absolute=True,
    )
    actuator_values = np.column_stack([data[name] for name in ACTUATOR_COLUMNS])
    metrics = {
        "duration": float(data["t"][-1] - data["t"][0]),
        "speed": float(np.max(np.linalg.norm(velocity, axis=1))),
        "snap": float(np.max(np.linalg.norm(snap, axis=1))),
        "yaw_acceleration": float(np.max(np.abs(data["yaw_acceleration"]))),
        "rotor": peak_rotor,
        "flap": peak_flap,
        "invalid_actuator_samples": float(
            np.count_nonzero(~np.all(np.isfinite(actuator_values), axis=1))
        ),
    }
    if all(name in data for name in DYNAMIC_COLUMNS):
        metrics.update(
            {
                "body_rate": finite_max(data["body_rate_norm"]),
                "flap_rate": finite_max(
                    np.concatenate((data["flap_rate_1"], data["flap_rate_2"])),
                    absolute=True,
                ),
                "rotor_command": finite_max(
                    np.concatenate(
                        (data["omega_cmd_req_1"], data["omega_cmd_req_2"])
                    ),
                    absolute=True,
                ),
                "flap_command": finite_max(
                    np.concatenate(
                        (data["delta_cmd_req_1"], data["delta_cmd_req_2"])
                    ),
                    absolute=True,
                ),
            }
        )
    if "collective_thrust" in data:
        metrics["collective_thrust"] = finite_max(data["collective_thrust"])
    return metrics


def _print_summary(data: dict[str, np.ndarray], label: str | None = None) -> None:
    metrics = _summary_metrics(data)
    if label is not None:
        print(f"[{label}]")
    print(f"duration: {metrics['duration']:.6f} s")
    print(f"peak speed: {metrics['speed']:.6f} m/s")
    print(f"peak snap norm: {metrics['snap']:.6f} m/s^4")
    print(
        "peak absolute yaw acceleration: "
        f"{metrics['yaw_acceleration']:.6f} rad/s^2"
    )
    print(f"peak rotor speed: {metrics['rotor']:.6f} rad/s")
    print(f"peak absolute flap deflection: {metrics['flap']:.6f} rad")
    if "collective_thrust" in metrics:
        print(f"peak collective thrust: {metrics['collective_thrust']:.6f} N")
    if "body_rate" in metrics:
        print(f"peak body-rate norm: {metrics['body_rate']:.6f} rad/s")
        print(f"peak absolute flap rate: {metrics['flap_rate']:.6f} rad/s")
        print(
            "peak required rotor/flap command: "
            f"{metrics['rotor_command']:.6f} rad/s, "
            f"{metrics['flap_command']:.6f} rad"
        )
    print(
        "invalid actuator samples: "
        f"{int(metrics['invalid_actuator_samples'])}/{data['t'].size}"
    )


def _set_3d_limits(axis: plt.Axes, points: np.ndarray) -> None:
    lower = np.min(points, axis=0)
    upper = np.max(points, axis=0)
    spans = upper - lower
    maximum_span = max(float(np.max(spans)), 1.0)
    display_spans = np.maximum(spans, 0.18 * maximum_span)
    centers = 0.5 * (lower + upper)
    margins = 0.08 * display_spans
    half_widths = 0.5 * display_spans + margins
    axis.set_xlim(centers[0] - half_widths[0], centers[0] + half_widths[0])
    axis.set_ylim(centers[1] - half_widths[1], centers[1] + half_widths[1])
    axis.set_zlim(centers[2] - half_widths[2], centers[2] + half_widths[2])
    # set_box_aspect was added after Matplotlib 3.1, which is still present on
    # some ROS/Ubuntu workstations used by this project.
    if hasattr(axis, "set_box_aspect"):
        axis.set_box_aspect(display_spans)


def _plot_speed_trajectory(
    data: dict[str, np.ndarray],
    output: Path,
    title: str,
    cmap: str,
    raw_ned_z: bool,
    dpi: int,
) -> None:
    position = np.column_stack((data["px"], data["py"], data["pz"]))
    display_position = position.copy()
    if not raw_ned_z:
        display_position[:, 2] *= -1.0
    velocity = np.column_stack((data["vx"], data["vy"], data["vz"]))
    speed = np.linalg.norm(velocity, axis=1)

    segments = np.stack((display_position[:-1], display_position[1:]), axis=1)
    segment_speed = 0.5 * (speed[:-1] + speed[1:])
    speed_min = float(np.min(speed))
    speed_max = float(np.max(speed))
    color_max = speed_max if speed_max > speed_min else speed_min + 1e-12
    normalization = colors.Normalize(vmin=speed_min, vmax=color_max)

    figure = plt.figure(figsize=(8.2, 6.3), constrained_layout=True)
    axis = figure.add_subplot(111, projection="3d")
    curve = Line3DCollection(
        segments,
        cmap=cmap,
        norm=normalization,
        linewidth=3.2,
        capstyle="round",
    )
    curve.set_array(segment_speed)
    axis.add_collection3d(curve)
    axis.scatter(
        *display_position[0],
        color="black",
        marker="o",
        s=42,
        depthshade=False,
        label="Start",
        zorder=5,
    )
    axis.scatter(
        *display_position[-1],
        color="black",
        marker="X",
        s=55,
        depthshade=False,
        label="End",
        zorder=5,
    )
    _set_3d_limits(axis, display_position)
    axis.set_xlabel(r"$x_i$ [m]", labelpad=7)
    axis.set_ylabel(r"$y_i$ [m]", labelpad=7)
    axis.set_zlabel(r"$z_i$ [m]" if raw_ned_z else r"Altitude $-z_i$ [m]", labelpad=8)
    axis.set_title(f"{title}: trajectory colored by speed")
    axis.view_init(elev=24.0, azim=-58.0)
    axis.legend(loc="upper right")
    colorbar = figure.colorbar(curve, ax=axis, pad=0.09, shrink=0.78)
    colorbar.set_label(r"Speed $\|\mathbf{v}\|$ [m/s]")
    figure.savefig(output, dpi=dpi, bbox_inches="tight")


def _plot_smoothness(
    data: dict[str, np.ndarray], output: Path, title: str, dpi: int
) -> None:
    time = data["t"]
    snap = np.column_stack((data["snap_x"], data["snap_y"], data["snap_z"]))
    snap_norm = np.linalg.norm(snap, axis=1)

    figure, axes = plt.subplots(
        2,
        1,
        figsize=(9.2, 6.7),
        sharex=True,
        constrained_layout=True,
        gridspec_kw={"height_ratios": (1.4, 1.0)},
    )
    component_colors = ("#0072B2", "#D55E00", "#009E73")
    for index, (label, color) in enumerate(
        zip((r"$s_x$", r"$s_y$", r"$s_z$"), component_colors)
    ):
        axes[0].plot(time, snap[:, index], color=color, label=label)
    axes[0].plot(
        time,
        snap_norm,
        color="black",
        linewidth=1.8,
        label=r"$\|\mathbf{s}\|$",
    )
    axes[0].axhline(0.0, color="0.35", linewidth=0.8)
    axes[0].set_ylabel(r"Snap [m/s$^4$]")
    axes[0].set_title(f"{title}: flat-output smoothness", loc="left")
    axes[0].legend(ncol=4, loc="best")

    axes[1].plot(
        time,
        data["yaw_acceleration"],
        color="#CC79A7",
        linewidth=1.8,
        label=r"$\ddot{\psi}$",
    )
    axes[1].axhline(0.0, color="0.35", linewidth=0.8)
    axes[1].set_xlabel("Time [s]")
    axes[1].set_ylabel(r"Yaw acceleration [rad/s$^2$]")
    axes[1].legend(loc="best")
    axes[1].set_xlim(float(time[0]), float(time[-1]))
    figure.savefig(output, dpi=dpi, bbox_inches="tight")


def _plot_actuators(
    data: dict[str, np.ndarray],
    output: Path,
    title: str,
    omega_max: float,
    delta_max: float,
    dpi: int,
) -> None:
    time = data["t"]
    figure, axes = plt.subplots(
        2, 1, figsize=(9.2, 6.7), sharex=True, constrained_layout=True
    )

    axes[0].plot(time, data["omega1_pred"], color="#0072B2", label=r"$\omega_1$")
    axes[0].plot(time, data["omega2_pred"], color="#D55E00", label=r"$\omega_2$")
    axes[0].axhline(
        omega_max,
        color="#7B3294",
        linestyle="--",
        linewidth=1.2,
        label=fr"limit: {omega_max:g} rad/s",
    )
    axes[0].axhline(0.0, color="0.35", linewidth=0.8)
    axes[0].set_ylabel(r"Rotor speed [rad/s]")
    axes[0].set_title(f"{title}: raw open-loop actuator prediction", loc="left")
    axes[0].legend(ncol=3, loc="best")

    axes[1].plot(time, data["delta1_pred"], color="#009E73", label=r"$\delta_1$")
    axes[1].plot(time, data["delta2_pred"], color="#CC79A7", label=r"$\delta_2$")
    axes[1].axhline(
        delta_max,
        color="#7B3294",
        linestyle="--",
        linewidth=1.2,
        label=fr"limits: $\pm${delta_max:g} rad",
    )
    axes[1].axhline(
        -delta_max, color="#7B3294", linestyle="--", linewidth=1.2
    )
    axes[1].axhline(0.0, color="0.35", linewidth=0.8)
    axes[1].set_xlabel("Time [s]")
    axes[1].set_ylabel(r"Flap deflection [rad]")
    axes[1].legend(ncol=3, loc="best")
    axes[1].set_xlim(float(time[0]), float(time[-1]))
    figure.savefig(output, dpi=dpi, bbox_inches="tight")


def _display_position(
    data: dict[str, np.ndarray], raw_ned_z: bool
) -> np.ndarray:
    position = np.column_stack((data["px"], data["py"], data["pz"]))
    if not raw_ned_z:
        position[:, 2] *= -1.0
    return position


def _add_speed_curve(
    axis: plt.Axes,
    data: dict[str, np.ndarray],
    cmap: str,
    normalization: colors.Normalize,
    raw_ned_z: bool,
) -> Line3DCollection:
    position = _display_position(data, raw_ned_z)
    velocity = np.column_stack((data["vx"], data["vy"], data["vz"]))
    speed = np.linalg.norm(velocity, axis=1)
    segments = np.stack((position[:-1], position[1:]), axis=1)
    curve = Line3DCollection(
        segments,
        cmap=cmap,
        norm=normalization,
        linewidth=3.0,
        capstyle="round",
    )
    curve.set_array(0.5 * (speed[:-1] + speed[1:]))
    axis.add_collection3d(curve)
    axis.scatter(*position[0], color="black", marker="o", s=34, depthshade=False)
    axis.scatter(*position[-1], color="black", marker="X", s=46, depthshade=False)
    _set_3d_limits(axis, position)
    axis.set_xlabel(r"$x_i$ [m]", labelpad=5)
    axis.set_ylabel(r"$y_i$ [m]", labelpad=5)
    axis.set_zlabel(
        r"$z_i$ [m]" if raw_ned_z else r"Altitude $-z_i$ [m]", labelpad=6
    )
    axis.view_init(elev=24.0, azim=-58.0)
    return curve


def _plot_speed_trajectory_comparison(
    datasets: list[dict[str, np.ndarray]],
    output: Path,
    title: str,
    labels: tuple[str, ...],
    cmap: str,
    raw_ned_z: bool,
    dpi: int,
) -> None:
    speeds = []
    for data in datasets:
        velocity = np.column_stack((data["vx"], data["vy"], data["vz"]))
        speeds.append(np.linalg.norm(velocity, axis=1))
    speed_min = min(float(np.min(speed)) for speed in speeds)
    speed_max = max(float(np.max(speed)) for speed in speeds)
    normalization = colors.Normalize(
        vmin=speed_min,
        vmax=speed_max if speed_max > speed_min else speed_min + 1e-12,
    )

    figure = plt.figure(
        figsize=(6.2 * len(datasets), 5.8), constrained_layout=True
    )
    axes = [
        figure.add_subplot(1, len(datasets), index + 1, projection="3d")
        for index in range(len(datasets))
    ]
    curve = None
    for axis, data, label in zip(axes, datasets, labels):
        curve = _add_speed_curve(axis, data, cmap, normalization, raw_ned_z)
        axis.set_title(f"{label}\n$T={_summary_metrics(data)['duration']:.3f}$ s")
    figure.suptitle(f"{title}: trajectory colored by speed")
    colorbar = figure.colorbar(curve, ax=axes, pad=0.07, shrink=0.78)
    colorbar.set_label(r"Speed $\|\mathbf{v}\|$ [m/s]")
    figure.savefig(output, dpi=dpi, bbox_inches="tight")


def _plot_smoothness_comparison(
    datasets: list[dict[str, np.ndarray]],
    output: Path,
    title: str,
    labels: tuple[str, ...],
    dpi: int,
) -> None:
    figure, axes = plt.subplots(
        2,
        1,
        figsize=(10.2, 7.2),
        sharex=False,
        constrained_layout=True,
        gridspec_kw={"height_ratios": (1.45, 1.0)},
    )
    component_names = ("snap_x", "snap_y", "snap_z")
    component_labels = (r"$s_x$", r"$s_y$", r"$s_z$")
    component_colors = ("#0072B2", "#D55E00", "#009E73")
    method_styles = ("-", "--", "-.")
    for data, linestyle in zip(datasets, method_styles):
        snap = np.column_stack([data[name] for name in component_names])
        for index, color in enumerate(component_colors):
            axes[0].plot(
                data["t"], snap[:, index], color=color, linestyle=linestyle
            )
        axes[0].plot(
            data["t"],
            np.linalg.norm(snap, axis=1),
            color="black",
            linestyle=linestyle,
            linewidth=1.8,
        )
        axes[1].plot(
            data["t"],
            data["yaw_acceleration"],
            color="#CC79A7",
            linestyle=linestyle,
            linewidth=1.8,
        )
    axes[0].axhline(0.0, color="0.35", linewidth=0.8)
    axes[0].set_ylabel(r"Snap [m/s$^4$]")
    axes[0].set_title(f"{title}: smoothness comparison", loc="left")
    component_handles = [
        Line2D([0], [0], color=color, label=label)
        for color, label in zip(component_colors, component_labels)
    ]
    component_handles.append(Line2D([0], [0], color="black", label=r"$\|\mathbf{s}\|$"))
    component_legend = axes[0].legend(handles=component_handles, ncol=4, loc="best")
    axes[0].add_artist(component_legend)

    axes[1].axhline(0.0, color="0.35", linewidth=0.8)
    axes[1].set_xlabel("Time [s]")
    axes[1].set_ylabel(r"Yaw acceleration [rad/s$^2$]")
    method_handles = [
        Line2D([0], [0], color="0.2", linestyle=style, label=label)
        for style, label in zip(method_styles, labels)
    ]
    axes[1].legend(handles=method_handles, ncol=len(labels), loc="best")
    max_time = max(float(data["t"][-1]) for data in datasets)
    axes[0].set_xlim(0.0, max_time)
    axes[1].set_xlim(0.0, max_time)
    figure.savefig(output, dpi=dpi, bbox_inches="tight")


def _plot_actuators_comparison(
    datasets: list[dict[str, np.ndarray]],
    output: Path,
    title: str,
    labels: tuple[str, ...],
    omega_max: float,
    delta_max: float,
    dpi: int,
) -> None:
    figure, axes = plt.subplots(
        2, 1, figsize=(10.2, 7.2), sharex=True, constrained_layout=True
    )
    method_styles = ("-", "--", "-.")
    for data, linestyle in zip(datasets, method_styles):
        axes[0].plot(
            data["t"], data["omega1_pred"], color="#0072B2", linestyle=linestyle
        )
        axes[0].plot(
            data["t"], data["omega2_pred"], color="#D55E00", linestyle=linestyle
        )
        axes[1].plot(
            data["t"], data["delta1_pred"], color="#009E73", linestyle=linestyle
        )
        axes[1].plot(
            data["t"], data["delta2_pred"], color="#CC79A7", linestyle=linestyle
        )
    axes[0].axhline(omega_max, color="#7B3294", linestyle=":", linewidth=1.4)
    axes[0].axhline(0.0, color="0.35", linewidth=0.8)
    axes[0].set_ylabel(r"Rotor speed [rad/s]")
    axes[0].set_title(f"{title}: open-loop actuator comparison", loc="left")
    channel_handles = [
        Line2D([0], [0], color="#0072B2", label=r"$\omega_1$"),
        Line2D([0], [0], color="#D55E00", label=r"$\omega_2$"),
        Line2D([0], [0], color="#7B3294", linestyle=":", label="upper limit"),
    ]
    axes[0].legend(handles=channel_handles, ncol=3, loc="best")

    axes[1].axhline(delta_max, color="#7B3294", linestyle=":", linewidth=1.4)
    axes[1].axhline(-delta_max, color="#7B3294", linestyle=":", linewidth=1.4)
    axes[1].axhline(0.0, color="0.35", linewidth=0.8)
    axes[1].set_xlabel("Time [s]")
    axes[1].set_ylabel(r"Flap deflection [rad]")
    flap_handles = [
        Line2D([0], [0], color="#009E73", label=r"$\delta_1$"),
        Line2D([0], [0], color="#CC79A7", label=r"$\delta_2$"),
    ]
    method_handles = [
        Line2D([0], [0], color="0.2", linestyle=style, label=label)
        for style, label in zip(method_styles, labels)
    ]
    axes[1].legend(handles=flap_handles + method_handles, ncol=3, loc="best")
    axes[1].set_xlim(0.0, max(float(data["t"][-1]) for data in datasets))
    figure.savefig(output, dpi=dpi, bbox_inches="tight")


def _plot_dynamic_constraints(
    datasets: list[dict[str, np.ndarray]],
    output: Path,
    title: str,
    labels: tuple[str, ...],
    omega_max: float,
    delta_max: float,
    body_rate_max: float,
    servo_rate_max: float,
    dpi: int,
) -> None:
    figure, axes = plt.subplots(
        4, 1, figsize=(10.2, 10.2), sharex=True, constrained_layout=True
    )
    method_styles = ("-", "--", "-.")
    channel_colors = ("#0072B2", "#D55E00")
    for data, linestyle in zip(datasets, method_styles):
        axes[0].plot(
            data["t"], data["body_rate_norm"], color="black", linestyle=linestyle
        )
        for channel, color in enumerate(channel_colors, start=1):
            axes[1].plot(
                data["t"],
                data[f"omega_cmd_req_{channel}"],
                color=color,
                linestyle=linestyle,
            )
            axes[2].plot(
                data["t"],
                data[f"flap_rate_{channel}"],
                color=color,
                linestyle=linestyle,
            )
            axes[3].plot(
                data["t"],
                data[f"delta_cmd_req_{channel}"],
                color=color,
                linestyle=linestyle,
            )

    axes[0].axhline(body_rate_max, color="#7B3294", linestyle=":", linewidth=1.4)
    axes[0].set_ylabel(r"$\|\Omega\|$ [rad/s]")
    axes[0].set_title(f"{title}: rate and first-order dynamic constraints", loc="left")

    axes[1].axhline(omega_max, color="#7B3294", linestyle=":", linewidth=1.4)
    axes[1].axhline(0.0, color="0.35", linewidth=0.8)
    axes[1].set_ylabel(r"Required $\omega_{cmd}$ [rad/s]")

    axes[2].axhline(servo_rate_max, color="#7B3294", linestyle=":", linewidth=1.4)
    axes[2].axhline(-servo_rate_max, color="#7B3294", linestyle=":", linewidth=1.4)
    axes[2].axhline(0.0, color="0.35", linewidth=0.8)
    axes[2].set_ylabel(r"$\dot{\delta}$ [rad/s]")

    axes[3].axhline(delta_max, color="#7B3294", linestyle=":", linewidth=1.4)
    axes[3].axhline(-delta_max, color="#7B3294", linestyle=":", linewidth=1.4)
    axes[3].axhline(0.0, color="0.35", linewidth=0.8)
    axes[3].set_ylabel(r"Required $\delta_{cmd}$ [rad]")
    axes[3].set_xlabel("Time [s]")

    channel_handles = [
        Line2D([0], [0], color=channel_colors[0], label="actuator 1"),
        Line2D([0], [0], color=channel_colors[1], label="actuator 2"),
        Line2D([0], [0], color="#7B3294", linestyle=":", label="limit"),
    ]
    method_handles = [
        Line2D([0], [0], color="0.2", linestyle=style, label=label)
        for style, label in zip(method_styles, labels)
    ]
    axes[3].legend(handles=channel_handles + method_handles, ncol=3, loc="best")
    axes[3].set_xlim(0.0, max(float(data["t"][-1]) for data in datasets))
    figure.savefig(output, dpi=dpi, bbox_inches="tight")


def _plot_high_level_constraints(
    datasets: list[dict[str, np.ndarray]],
    output: Path,
    title: str,
    labels: tuple[str, ...],
    velocity_max: float | None,
    collective_thrust_max: float,
    body_rate_max: float,
    dpi: int,
) -> None:
    figure, axes = plt.subplots(
        3, 1, figsize=(10.2, 8.0), sharex=True, constrained_layout=True
    )
    method_styles = ("-", "--", "-.")
    method_colors = ("#0072B2", "#D55E00", "#009E73")
    for data, label, linestyle, color in zip(
        datasets, labels, method_styles, method_colors
    ):
        speed = np.linalg.norm(
            np.column_stack((data["vx"], data["vy"], data["vz"])), axis=1
        )
        axes[0].plot(data["t"], speed, color=color, linestyle=linestyle, label=label)
        axes[1].plot(
            data["t"],
            data["collective_thrust"],
            color=color,
            linestyle=linestyle,
            label=label,
        )
        axes[2].plot(
            data["t"],
            data["body_rate_norm"],
            color=color,
            linestyle=linestyle,
            label=label,
        )

    limits = (velocity_max, collective_thrust_max, body_rate_max)
    ylabels = (
        r"$\|\mathbf{v}\|$ [m/s]",
        r"Collective thrust $T$ [N]",
        r"$\|\Omega\|$ [rad/s]",
    )
    for axis, limit, ylabel in zip(axes, limits, ylabels):
        if limit is not None:
            axis.axhline(limit, color="#7B3294", linestyle=":", linewidth=1.4,
                         label="planning limit")
        axis.axhline(0.0, color="0.35", linewidth=0.8)
        axis.set_ylabel(ylabel)
    axes[0].set_title(f"{title}: high-level constraints", loc="left")
    axes[0].legend(ncol=min(3, len(labels) + 1), loc="best")
    axes[2].set_xlabel("Time [s]")
    axes[2].set_xlim(0.0, max(float(data["t"][-1]) for data in datasets))
    figure.savefig(output, dpi=dpi, bbox_inches="tight")


def main() -> int:
    args = _parse_args()
    if (
        args.omega_max <= 0.0
        or args.delta_max <= 0.0
        or args.body_rate_max <= 0.0
        or args.servo_rate_max <= 0.0
        or args.velocity_max <= 0.0
        or args.collective_thrust_max <= 0.0
        or args.dpi <= 0
    ):
        raise ValueError("各约束上限和 dpi 必须为正数")

    csv_path = args.csv.resolve()
    output_dir = (
        args.output_dir.resolve()
        if args.output_dir is not None
        else csv_path.parent / "figures"
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    compare_path = args.compare_csv.resolve() if args.compare_csv is not None else None
    compare_path_2 = (
        args.compare_csv_2.resolve() if args.compare_csv_2 is not None else None
    )
    if compare_path_2 is not None and compare_path is None:
        raise ValueError("使用 --compare-csv-2 时必须同时给出 --compare-csv")
    default_prefix = (
        f"{csv_path.stem}_vs_{compare_path.stem}"
        if compare_path is not None
        else csv_path.stem
    )
    prefix = args.prefix or default_prefix
    title = args.title or csv_path.stem.replace("_", " ")
    suffix = args.format
    outputs = {
        "trajectory": output_dir / f"{prefix}_trajectory_speed.{suffix}",
        "smoothness": output_dir / f"{prefix}_smoothness.{suffix}",
        "actuators": output_dir / f"{prefix}_actuators.{suffix}",
    }

    _apply_style()
    data = _load_csv(csv_path)
    dynamic_datasets: list[dict[str, np.ndarray]] = []
    dynamic_labels: tuple[str, ...] = ()
    high_level_datasets: list[dict[str, np.ndarray]] = []
    high_level_labels: tuple[str, ...] = ()
    if compare_path is None:
        _print_summary(data)
        _plot_speed_trajectory(
            data,
            outputs["trajectory"],
            title,
            args.cmap,
            args.raw_ned_z,
            args.dpi,
        )
        _plot_smoothness(data, outputs["smoothness"], title, args.dpi)
        _plot_actuators(
            data,
            outputs["actuators"],
            title,
            args.omega_max,
            args.delta_max,
            args.dpi,
        )
        if all(name in data for name in DYNAMIC_COLUMNS):
            dynamic_datasets = [data]
            dynamic_labels = (args.primary_label,)
        if (
            all(name in data for name in HIGH_LEVEL_COLUMNS)
            and "body_rate_norm" in data
        ):
            high_level_datasets = [data]
            high_level_labels = (args.primary_label,)
    else:
        comparison = _load_csv(compare_path)
        datasets = [data, comparison]
        labels = [args.primary_label, args.compare_label]
        if compare_path_2 is not None:
            datasets.append(_load_csv(compare_path_2))
            labels.append(args.compare_label_2)
        label_tuple = tuple(labels)
        for dataset, label in zip(datasets, label_tuple):
            _print_summary(dataset, label)
        _plot_speed_trajectory_comparison(
            datasets,
            outputs["trajectory"],
            title,
            label_tuple,
            args.cmap,
            args.raw_ned_z,
            args.dpi,
        )
        _plot_smoothness_comparison(
            datasets, outputs["smoothness"], title, label_tuple, args.dpi
        )
        _plot_actuators_comparison(
            datasets,
            outputs["actuators"],
            title,
            label_tuple,
            args.omega_max,
            args.delta_max,
            args.dpi,
        )
        if all(all(name in dataset for name in DYNAMIC_COLUMNS) for dataset in datasets):
            dynamic_datasets = datasets
            dynamic_labels = label_tuple
        if all(
            all(name in dataset for name in HIGH_LEVEL_COLUMNS)
            and "body_rate_norm" in dataset
            for dataset in datasets
        ):
            high_level_datasets = datasets
            high_level_labels = label_tuple

    if dynamic_datasets:
        outputs["dynamic_constraints"] = (
            output_dir / f"{prefix}_dynamic_constraints.{suffix}"
        )
        _plot_dynamic_constraints(
            dynamic_datasets,
            outputs["dynamic_constraints"],
            title,
            dynamic_labels,
            args.omega_max,
            args.delta_max,
            args.body_rate_max,
            args.servo_rate_max,
            args.dpi,
        )

    if high_level_datasets:
        outputs["high_level_constraints"] = (
            output_dir / f"{prefix}_high_level_constraints.{suffix}"
        )
        _plot_high_level_constraints(
            high_level_datasets,
            outputs["high_level_constraints"],
            title,
            high_level_labels,
            None if args.no_velocity_limit else args.velocity_max,
            args.collective_thrust_max,
            args.body_rate_max,
            args.dpi,
        )

    for label, path in outputs.items():
        print(f"{label}: {path}")
    if args.show:
        plt.show()
    else:
        plt.close("all")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
