"""Tailsitter full-system closed-loop simulation entry point."""

import argparse
from dataclasses import fields

import matplotlib.pyplot as plt
import numpy as np

from config import TailsitterConfig
from environment.closed_loop import SimulationSample, TailsitterClosedLoop
from trajectory.factory import TRAJECTORY_CHOICES, make_trajectory


LOG_FIELD_NAMES = tuple(field.name for field in fields(SimulationSample))


def parse_mode_scale(value):
    try:
        name, scale_text = value.rsplit("=", 1)
        scale = float(scale_text)
    except ValueError as error:
        raise argparse.ArgumentTypeError("格式必须为 TRAJECTORY=SCALE") from error
    if not name or not 0.0 <= scale <= 1.0:
        raise argparse.ArgumentTypeError("SCALE 必须位于 [0, 1]")
    return name, scale


def run_simulation(config, trajectory, residual_provider=None):
    """Run one trajectory and return the simulator plus column-oriented logs."""
    simulator = TailsitterClosedLoop(
        config,
        trajectory,
        residual_feedforward_consistency=getattr(
            residual_provider, "residual_feedforward_consistency", False
        ),
        residual_filter_cutoff_hz=getattr(
            residual_provider, "residual_filter_cutoff_hz", 10.0
        ),
    )
    records = {name: [] for name in LOG_FIELD_NAMES}
    while not simulator.done:
        residual = (
            np.zeros(3)
            if residual_provider is None
            else residual_provider(simulator)
        )
        sample = simulator.step(residual)
        for name in LOG_FIELD_NAMES:
            records[name].append(getattr(sample, name))
    logs = {name: np.asarray(values) for name, values in records.items()}
    return simulator, logs


def compute_metrics(config, simulator, logs):
    position_error = logs["position_ref"] - logs["position"]
    error_norm = np.linalg.norm(position_error, axis=1)
    speed_actual = np.linalg.norm(logs["velocity"], axis=1)
    speed_reference = np.linalg.norm(logs["velocity_ref"], axis=1)
    yaw_actual = np.deg2rad(logs["euler_deg"][:, 0])
    yaw_error = (
        yaw_actual - logs["psi_ref"] + np.pi
    ) % (2.0 * np.pi) - np.pi
    differential_thrust = config.c_T * (
        logs["rotor_speed"][:, 0] ** 2 - logs["rotor_speed"][:, 1] ** 2
    )
    reference_specific_force = np.linalg.norm(
        logs["acceleration_ref"] - np.array([0.0, 0.0, config.g]), axis=1
    ) / config.g
    rotor_saturation = np.any(
        logs["rotor_speed"] >= 0.999 * config.omega_max, axis=1
    )
    flap_saturation = np.any(
        np.abs(np.deg2rad(logs["flap_deg"])) >= 0.999 * config.delta_max,
        axis=1,
    )
    return {
        "rms_position_error": float(np.sqrt(np.mean(error_norm**2))),
        "max_position_error": float(error_norm.max()),
        "rms_yaw_error_deg": float(
            np.rad2deg(np.sqrt(np.mean(yaw_error**2)))
        ),
        "min_actual_speed": float(speed_actual.min()),
        "max_actual_speed": float(speed_actual.max()),
        "min_reference_speed": float(speed_reference.min()),
        "max_reference_speed": float(speed_reference.max()),
        "max_reference_specific_force_g": float(reference_specific_force.max()),
        "max_rotor_speed": float(np.max(logs["rotor_speed"])),
        "max_abs_flap_deg": float(np.max(np.abs(logs["flap_deg"]))),
        "max_abs_differential_thrust": float(
            np.max(np.abs(differential_thrust))
        ),
        "rotor_saturation_fraction": float(np.mean(rotor_saturation)),
        "flap_saturation_fraction": float(np.mean(flap_saturation)),
        "rms_residual_acceleration": float(
            np.sqrt(np.mean(logs["residual_acceleration_body"] ** 2))
        ),
        "finite": bool(
            np.all(np.isfinite(simulator.state))
            and all(np.all(np.isfinite(value)) for value in logs.values())
        ),
        "duration": float(logs["time"][-1]),
    }


def save_log(path, logs):
    np.savez_compressed(
        path,
        time=logs["time"],
        position=logs["position"],
        velocity=logs["velocity"],
        acceleration=logs["acceleration"],
        position_ref=logs["position_ref"],
        velocity_ref=logs["velocity_ref"],
        acceleration_ref=logs["acceleration_ref"],
        psi_ref=logs["psi_ref"],
        euler_deg=logs["euler_deg"],
        euler_command_deg=logs["euler_command_deg"],
        angular_velocity=logs["angular_velocity"],
        angular_velocity_ref=logs["angular_velocity_ref"],
        angular_acceleration=logs["angular_acceleration"],
        angular_acceleration_lpf=logs["angular_acceleration_lpf"],
        angular_acceleration_command=logs["angular_acceleration_command"],
        rotor_speed=logs["rotor_speed"],
        flap_deg=logs["flap_deg"],
        residual_acceleration_body=logs["residual_acceleration_body"],
        applied_residual_acceleration_inertial=(
            logs["applied_residual_acceleration_inertial"]
        ),
        residual_jerk_inertial=logs["residual_jerk_inertial"],
        coordinate_frame=np.array("NED"),
        euler_sequence=np.array("ZXY"),
    )


def plot_simulation(trajectory_name, logs, show_plot):
    plt.figure(figsize=(12, 10))
    plt.subplot(3, 1, 1)
    plt.plot(logs["time"], logs["position"][:, 0], label="X (Forward)")
    plt.plot(logs["time"], logs["position"][:, 1], label="Y (Right)")
    plt.plot(logs["time"], logs["position"][:, 2], label="Z (Down)")
    plt.axhline(
        -10, color="r", linestyle="--", alpha=0.5, label="Hover Alt (-10m)"
    )
    plt.ylabel("Position (m)")
    plt.legend()
    plt.grid()
    plt.title(f"Tailsitter Simulation: {trajectory_name}")

    plt.subplot(3, 1, 2)
    plt.plot(logs["time"], logs["euler_deg"][:, 0], label="Yaw (deg)")
    plt.plot(logs["time"], logs["euler_deg"][:, 1], label="Roll (deg)")
    plt.plot(logs["time"], logs["euler_deg"][:, 2], label="Pitch (deg)")
    plt.ylabel("Euler Angles (deg)")
    plt.legend()
    plt.grid()

    plt.subplot(3, 1, 3)
    plt.plot(
        logs["time"], logs["rotor_speed"][:, 0], label="Rotor 1 (rad/s)"
    )
    plt.plot(
        logs["time"], logs["rotor_speed"][:, 1], label="Rotor 2 (rad/s)"
    )
    plt.plot(logs["time"], logs["flap_deg"][:, 0] * 10, label="Flap 1 (deg x10)")
    plt.plot(logs["time"], logs["flap_deg"][:, 1] * 10, label="Flap 2 (deg x10)")
    plt.ylabel("Actuator Commands")
    plt.xlabel("Time (s)")
    plt.legend()
    plt.grid()
    plt.tight_layout()
    if show_plot:
        plt.show()
    else:
        plt.close()


def main(
    trajectory_name="segments",
    laps=None,
    speed=None,
    lap_time=None,
    log_path="flight_log.npz",
    show_plot=True,
    residual_provider=None,
):
    print("初始化仿真环境...")
    config = TailsitterConfig()
    trajectory = make_trajectory(trajectory_name, laps, speed, lap_time)
    print(f"开始仿真运行，轨迹: {trajectory_name}...")
    simulator, logs = run_simulation(config, trajectory, residual_provider)
    print("仿真结束，正在绘图...")
    save_log(log_path, logs)
    print(f"仿真日志已保存到 {log_path}")
    metrics = compute_metrics(config, simulator, logs)
    print(
        "位置跟踪误差:  "
        f"RMS = {metrics['rms_position_error']:.3f} m,  "
        f"最大 = {metrics['max_position_error']:.3f} m"
    )
    plot_simulation(trajectory_name, logs, show_plot)
    return {"trajectory": trajectory_name, **metrics}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Tailsitter 全系统闭环仿真")
    parser.add_argument(
        "--trajectory",
        choices=TRAJECTORY_CHOICES,
        default="segments",
        help="选择分段轨迹或论文中的试验轨迹",
    )
    parser.add_argument("--laps", type=int, help="闭合轨迹圈数；省略时使用论文默认值")
    parser.add_argument(
        "--speed", type=float, help="覆盖所选轨迹的默认参考速度 (m/s)"
    )
    parser.add_argument(
        "--lap-time", type=float, help="覆盖双纽线或椭圆跑道的单圈时间 (s)"
    )
    parser.add_argument("--log", default="flight_log.npz", help="NPZ 日志输出路径")
    parser.add_argument("--no-show", action="store_true", help="仿真后不打开绘图窗口")
    parser.add_argument("--preview-model", help="启用已训练的未来轨迹 CNN 残差模型")
    parser.add_argument("--preview-rate", type=float, help="覆盖模型的更新频率 (Hz)")
    parser.add_argument(
        "--residual-limit", type=float, help="覆盖机体系残差加速度限幅 (m/s^2)"
    )
    parser.add_argument(
        "--allow-unsafe-preview",
        action="store_true",
        help="仅离线诊断：允许 CNN 作用于已知退化的全程刀锋圆周",
    )
    parser.add_argument(
        "--residual-feedforward-consistency",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="覆盖模型元数据中的姿态/角速度前馈一致性设置",
    )
    parser.add_argument(
        "--residual-filter-cutoff",
        type=float,
        help="覆盖模型元数据中的一致性残差低通截止频率 (Hz)",
    )
    parser.add_argument(
        "--mode-scale",
        action="append",
        type=parse_mode_scale,
        default=[],
        metavar="TRAJECTORY=SCALE",
        help="覆盖指定轨迹/飞行模式的残差缩放，可重复指定",
    )
    args = parser.parse_args()
    residual_provider = None
    if args.preview_model:
        from learning.residual_policy import PreviewResidualProvider

        residual_provider = PreviewResidualProvider(
            args.preview_model,
            policy_rate_hz=args.preview_rate,
            residual_limit=args.residual_limit,
            allow_circular_knife_edge=args.allow_unsafe_preview,
            residual_feedforward_consistency=(
                args.residual_feedforward_consistency
            ),
            residual_filter_cutoff_hz=args.residual_filter_cutoff,
            residual_mode_scales=(
                dict(args.mode_scale) if args.mode_scale else None
            ),
        )
    main(
        args.trajectory,
        args.laps,
        args.speed,
        args.lap_time,
        args.log,
        not args.no_show,
        residual_provider,
    )
