#!/usr/bin/env python3
"""绘制两条联合优化 hover-to-hover 轨迹的闭环跟踪结果。"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
PLANNING_DIR = SCRIPT_DIR.parent
DEFAULT_LOGS = (
    PLANNING_DIR / "output" / "joint_hover_yaw0_tracking.npz",
    PLANNING_DIR / "output" / "joint_hover_yaw_pi_tracking.npz",
)
DEFAULT_OUTPUT_DIR = PLANNING_DIR / "output" / "figures"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--yaw0-log", type=Path, default=DEFAULT_LOGS[0])
    parser.add_argument("--yaw-pi-log", type=Path, default=DEFAULT_LOGS[1])
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--prefix", default="joint_hover_closed_loop")
    parser.add_argument("--dpi", type=int, default=180)
    parser.add_argument("--show", action="store_true")
    return parser.parse_args()


def load_log(path: Path) -> dict[str, np.ndarray | float]:
    required = (
        "time", "position", "velocity", "position_ref", "velocity_ref",
        "psi_ref", "euler_deg", "angular_velocity", "angular_velocity_ref",
        "rotor_speed", "rotor_speed_command", "flap_deg", "flap_command_deg",
        "planned_duration",
    )
    if not path.is_file():
        raise FileNotFoundError(f"找不到闭环日志：{path}")
    with np.load(path, allow_pickle=False) as archive:
        missing = [name for name in required if name not in archive]
        if missing:
            raise KeyError(f"{path.name} 缺少字段：{', '.join(missing)}")
        data: dict[str, np.ndarray | float] = {
            name: np.asarray(archive[name], dtype=float) for name in required
        }
    data["planned_duration"] = float(data["planned_duration"])
    time = np.asarray(data["time"])
    if time.ndim != 1 or time.size < 2 or not np.all(np.diff(time) > 0.0):
        raise ValueError(f"{path.name} 的 time 必须一维且严格递增")
    return data


def wrapped_yaw_error(actual: np.ndarray, reference: np.ndarray) -> np.ndarray:
    return (actual - reference + np.pi) % (2.0 * np.pi) - np.pi


def statistics(data: dict[str, np.ndarray | float]) -> dict[str, float]:
    time = np.asarray(data["time"])
    maneuver = time <= float(data["planned_duration"]) + 1e-12
    position_error = np.asarray(data["position_ref"]) - np.asarray(data["position"])
    velocity_error = np.asarray(data["velocity_ref"]) - np.asarray(data["velocity"])
    position_error_norm = np.linalg.norm(position_error, axis=1)
    velocity_error_norm = np.linalg.norm(velocity_error, axis=1)
    yaw = np.unwrap(np.deg2rad(np.asarray(data["euler_deg"])[:, 0]))
    yaw_reference = np.unwrap(np.asarray(data["psi_ref"]))
    yaw_error = wrapped_yaw_error(yaw, yaw_reference)
    return {
        "position_rms": float(np.sqrt(np.mean(position_error_norm[maneuver] ** 2))),
        "position_max": float(np.max(position_error_norm[maneuver])),
        "position_final": float(position_error_norm[-1]),
        "velocity_rms": float(np.sqrt(np.mean(velocity_error_norm[maneuver] ** 2))),
        "velocity_max": float(np.max(velocity_error_norm[maneuver])),
        "yaw_rms_deg": float(np.rad2deg(np.sqrt(np.mean(yaw_error[maneuver] ** 2)))),
        "yaw_max_deg": float(np.rad2deg(np.max(np.abs(yaw_error[maneuver])))),
        "yaw_final_deg": float(np.rad2deg(abs(yaw_error[-1]))),
        "rotor_max": float(np.max(np.asarray(data["rotor_speed"]))),
        "rotor_command_max": float(np.max(np.asarray(data["rotor_speed_command"]))),
        "flap_max_deg": float(np.max(np.abs(np.asarray(data["flap_deg"])))),
        "flap_command_max_deg": float(
            np.max(np.abs(np.asarray(data["flap_command_deg"])))
        ),
        "body_rate_max": float(
            np.max(np.linalg.norm(np.asarray(data["angular_velocity"]), axis=1))
        ),
        "body_rate_reference_max": float(
            np.max(np.linalg.norm(np.asarray(data["angular_velocity_ref"]), axis=1))
        ),
    }


def style_time_axis(axis: plt.Axes, data: dict[str, np.ndarray | float]) -> None:
    time = np.asarray(data["time"])
    planned_duration = float(data["planned_duration"])
    axis.axvline(
        planned_duration, color="0.25", linestyle=":", linewidth=1.1,
        label="Maneuver end",
    )
    axis.set_xlim(float(time[0]), float(time[-1]))
    axis.grid(True, color="0.86", linewidth=0.7)
    axis.tick_params(direction="in", top=True, right=True)


def create_overview(
    cases: tuple[tuple[str, dict[str, np.ndarray | float]], ...]
) -> plt.Figure:
    figure, axes = plt.subplots(4, 2, figsize=(13.2, 12.0), sharex="col")
    colors = ("#0072B2", "#D55E00", "#009E73", "#4D4D4D")
    for column, (title, data) in enumerate(cases):
        time = np.asarray(data["time"])
        position = np.asarray(data["position"])
        position_ref = np.asarray(data["position_ref"])
        velocity = np.asarray(data["velocity"])
        velocity_ref = np.asarray(data["velocity_ref"])

        axis = axes[0, column]
        axis.plot(position_ref[:, 0], -position_ref[:, 2], color=colors[0],
                  linewidth=2.1, label="Reference")
        axis.plot(position[:, 0], -position[:, 2], color=colors[1],
                  linewidth=1.4, label="Closed loop")
        axis.set_title(title)
        axis.set_ylabel("Altitude $-z$ [m]")
        axis.set_xlabel("Forward position $x$ [m]")
        axis.grid(True, color="0.86", linewidth=0.7)
        axis.legend(loc="best")

        axis = axes[1, column]
        error = position_ref - position
        labels = (r"$e_x$", r"$e_y$", r"$e_z$")
        for index, label in enumerate(labels):
            axis.plot(time, error[:, index], color=colors[index], label=label)
        axis.plot(time, np.linalg.norm(error, axis=1), color=colors[3],
                  linewidth=1.7, label=r"$\|e_p\|$")
        axis.set_ylabel("Position error [m]")
        style_time_axis(axis, data)
        axis.legend(loc="best", ncol=2)

        axis = axes[2, column]
        axis.plot(time, np.linalg.norm(velocity_ref, axis=1), color=colors[0],
                  linewidth=2.0, label="Reference")
        axis.plot(time, np.linalg.norm(velocity, axis=1), color=colors[1],
                  linewidth=1.35, label="Closed loop")
        axis.set_ylabel(r"Speed $\|v\|$ [m/s]")
        style_time_axis(axis, data)
        axis.legend(loc="best")

        axis = axes[3, column]
        yaw = np.unwrap(np.deg2rad(np.asarray(data["euler_deg"])[:, 0]))
        yaw_reference = np.unwrap(np.asarray(data["psi_ref"]))
        axis.plot(time, np.rad2deg(yaw_reference), color=colors[0],
                  linewidth=2.0, label="Reference")
        axis.plot(time, np.rad2deg(yaw), color=colors[1],
                  linewidth=1.35, label="Closed loop")
        axis.set_ylabel("Yaw [deg]")
        axis.set_xlabel("Time [s]")
        style_time_axis(axis, data)
        axis.legend(loc="best")

    figure.suptitle("Closed-loop tracking of jointly optimized trajectories", y=0.995)
    figure.tight_layout()
    return figure


def create_actuators(
    cases: tuple[tuple[str, dict[str, np.ndarray | float]], ...]
) -> plt.Figure:
    figure, axes = plt.subplots(3, 2, figsize=(13.2, 9.5), sharex="col")
    colors = ("#0072B2", "#D55E00")
    for column, (title, data) in enumerate(cases):
        time = np.asarray(data["time"])
        rotor = np.asarray(data["rotor_speed"])
        rotor_command = np.asarray(data["rotor_speed_command"])
        flap = np.asarray(data["flap_deg"])
        flap_command = np.asarray(data["flap_command_deg"])
        body_rate = np.linalg.norm(np.asarray(data["angular_velocity"]), axis=1)
        body_rate_ref = np.linalg.norm(
            np.asarray(data["angular_velocity_ref"]), axis=1
        )

        axis = axes[0, column]
        for index, color in enumerate(colors):
            axis.plot(time, rotor[:, index], color=color,
                      label=rf"$\omega_{index + 1}$")
            axis.plot(time, rotor_command[:, index], color=color, linestyle="--",
                      alpha=0.75, label=rf"$\omega_{{{index + 1},cmd}}$")
        axis.axhline(2500.0, color="0.25", linestyle="-.", label="Physical limit")
        axis.set_title(title)
        axis.set_ylabel("Rotor speed [rad/s]")
        style_time_axis(axis, data)
        axis.legend(loc="best", ncol=2)

        axis = axes[1, column]
        for index, color in enumerate(colors):
            axis.plot(time, flap[:, index], color=color,
                      label=rf"$\delta_{index + 1}$")
            axis.plot(time, flap_command[:, index], color=color, linestyle="--",
                      alpha=0.75, label=rf"$\delta_{{{index + 1},cmd}}$")
        axis.axhline(np.rad2deg(1.0), color="0.25", linestyle="-.")
        axis.axhline(-np.rad2deg(1.0), color="0.25", linestyle="-.",
                     label="Physical limit")
        axis.set_ylabel("Flap deflection [deg]")
        style_time_axis(axis, data)
        axis.legend(loc="best", ncol=2)

        axis = axes[2, column]
        axis.plot(time, body_rate_ref, color=colors[0], linewidth=2.0,
                  label="Feedforward reference")
        axis.plot(time, body_rate, color=colors[1], linewidth=1.35,
                  label="Actual")
        axis.axhline(11.344640137963141, color="0.25", linestyle="-.",
                     label="Reference envelope")
        axis.set_ylabel("Body-rate norm [rad/s]")
        axis.set_xlabel("Time [s]")
        style_time_axis(axis, data)
        axis.legend(loc="best")

    figure.suptitle("Closed-loop actuator and body-rate response", y=0.995)
    figure.tight_layout()
    return figure


def print_statistics(
    cases: tuple[tuple[str, dict[str, np.ndarray | float]], ...]
) -> None:
    for title, data in cases:
        values = statistics(data)
        print(f"{title}:")
        print(
            "  position RMS/max/final = "
            f"{values['position_rms']:.4f} / {values['position_max']:.4f} / "
            f"{values['position_final']:.4f} m"
        )
        print(
            "  velocity RMS/max = "
            f"{values['velocity_rms']:.4f} / {values['velocity_max']:.4f} m/s"
        )
        print(
            "  yaw RMS/max/final = "
            f"{values['yaw_rms_deg']:.4f} / {values['yaw_max_deg']:.4f} / "
            f"{values['yaw_final_deg']:.4f} deg"
        )
        print(
            "  max rotor actual/command = "
            f"{values['rotor_max']:.2f} / {values['rotor_command_max']:.2f} rad/s"
        )
        print(
            "  max flap actual/command = "
            f"{values['flap_max_deg']:.2f} / "
            f"{values['flap_command_max_deg']:.2f} deg"
        )
        print(
            "  max body-rate actual/reference = "
            f"{values['body_rate_max']:.4f} / "
            f"{values['body_rate_reference_max']:.4f} rad/s"
        )


def main() -> None:
    args = parse_args()
    cases = (
        (r"Yaw $0\rightarrow0$", load_log(args.yaw0_log)),
        (r"Yaw $0\rightarrow\pi$", load_log(args.yaw_pi_log)),
    )
    args.output_dir.mkdir(parents=True, exist_ok=True)
    overview_path = args.output_dir / f"{args.prefix}_overview.png"
    actuator_path = args.output_dir / f"{args.prefix}_actuators.png"
    overview = create_overview(cases)
    actuators = create_actuators(cases)
    overview.savefig(overview_path, dpi=args.dpi, bbox_inches="tight")
    actuators.savefig(actuator_path, dpi=args.dpi, bbox_inches="tight")
    print_statistics(cases)
    print(f"overview figure: {overview_path.resolve()}")
    print(f"actuator figure: {actuator_path.resolve()}")
    if args.show:
        plt.show()
    else:
        plt.close(overview)
        plt.close(actuators)


if __name__ == "__main__":
    main()
