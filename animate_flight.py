"""使用仿真日志绘制尾座飞行器位置与姿态的三维动画。"""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FFMpegWriter, FuncAnimation, PillowWriter


PROJECT_ROOT = Path(__file__).resolve().parent

# 论文 Sec. VI-B～VI-F 的七种闭环验证日志（另含 circle-knife-edge
# 兼容别名），以及针对各自时长设置的默认抽帧和播放倍率。命令行传入
# --stride/--speed 时会覆盖这里的值。
TRAJECTORY_PRESETS = {
    "lemniscate": {
        "log": PROJECT_ROOT / "validation_logs/lemniscate.npz",
        "stride": 20,
        "speed": 1.0,
        "title": "Lemniscate of Bernoulli (Sec. VI-B)",
    },
    "knife-edge-transition": {
        "log": PROJECT_ROOT / "validation_logs/knife-edge-transition.npz",
        "stride": 20,
        "speed": 1.0,
        "title": "Knife-Edge Transitioning Flight",
    },
    "circle-coordinated": {
        "log": PROJECT_ROOT / "validation_logs/circle-coordinated.npz",
        "stride": 10,
        "speed": 1.0,
        "title": "Circular Trajectory - Coordinated Flight",
    },
    "circle-knife-edge": {
        # 兼容旧名称：按用户所指的 Sec. VI-C，展示椭圆跑道上的 knife-edge
        # transitioning，而不是 Sec. VI-D 的定半径圆。
        "log": PROJECT_ROOT / "validation_logs/knife-edge-transition.npz",
        "stride": 20,
        "speed": 1.0,
        "title": "Oval Knife-Edge Transitioning Flight (Sec. VI-C)",
    },
    "circular-knife-edge": {
        "log": PROJECT_ROOT / "validation_logs/circular-knife-edge.npz",
        "stride": 10,
        "speed": 1.0,
        "title": "Circular Trajectory - Steady Knife-Edge (Sec. VI-D)",
    },
    "transition-to-forward": {
        "log": PROJECT_ROOT / "validation_logs/transition-to-forward.npz",
        "stride": 5,
        "speed": 1.0,
        "title": "Transition - Hover to Forward Flight",
    },
    "transition-to-hover": {
        "log": PROJECT_ROOT / "validation_logs/transition-to-hover.npz",
        "stride": 5,
        "speed": 1.0,
        "title": "Transition - Forward Flight to Hover",
    },
    "differential-turn": {
        "log": PROJECT_ROOT / "validation_logs/differential-turn.npz",
        "stride": 3,
        "speed": 1.5,
        "title": "Differential-Thrust Turning",
    },
}

# circle-knife-edge 是指向 VI-C 的兼容别名，不在批量输出中重复生成。
BATCH_TRAJECTORIES = (
    "lemniscate",
    "knife-edge-transition",
    "circle-coordinated",
    "circular-knife-edge",
    "transition-to-forward",
    "transition-to-hover",
    "differential-turn",
)


def zxy_euler_to_rotation_matrix(euler_deg):
    """由 [psi, phi, theta]（度）计算 R_b^i = Rz(psi) Rx(phi) Ry(theta)。"""
    psi, phi, theta = np.deg2rad(euler_deg)
    c_psi, s_psi = np.cos(psi), np.sin(psi)
    c_phi, s_phi = np.cos(phi), np.sin(phi)
    c_theta, s_theta = np.cos(theta), np.sin(theta)

    R_z = np.array(
        [
            [c_psi, -s_psi, 0.0],
            [s_psi, c_psi, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    R_x = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, c_phi, -s_phi],
            [0.0, s_phi, c_phi],
        ]
    )
    R_y = np.array(
        [
            [c_theta, 0.0, s_theta],
            [0.0, 1.0, 0.0],
            [-s_theta, 0.0, c_theta],
        ]
    )
    return R_z @ R_x @ R_y


def ned_to_display(vectors):
    """将 NED 数据转为便于观察的 North-East-Up 显示坐标。"""
    displayed = np.array(vectors, dtype=float, copy=True)
    displayed[..., 2] *= -1.0
    return displayed


def set_equal_3d_limits(ax, points, margin):
    """设置固定且等比例的三维坐标范围，避免动画过程中视图缩放。"""
    minimum = np.min(points, axis=0)
    maximum = np.max(points, axis=0)
    center = 0.5 * (minimum + maximum)
    half_range = max(0.5 * np.max(maximum - minimum) + margin, margin, 1.0)

    ax.set_xlim(center[0] - half_range, center[0] + half_range)
    ax.set_ylim(center[1] - half_range, center[1] + half_range)
    ax.set_zlim(center[2] - half_range, center[2] + half_range)
    ax.set_box_aspect((1.0, 1.0, 1.0))


def load_flight_log(path):
    """读取并检查 main_sim.py 生成的 NPZ 日志。"""
    with np.load(path) as data:
        required = {"time", "position", "euler_deg"}
        missing = required.difference(data.files)
        if missing:
            raise ValueError(f"日志缺少字段: {', '.join(sorted(missing))}")

        time = np.asarray(data["time"], dtype=float)
        position = np.asarray(data["position"], dtype=float)
        euler_deg = np.asarray(data["euler_deg"], dtype=float)
        position_ref = (
            np.asarray(data["position_ref"], dtype=float)
            if "position_ref" in data.files
            else None
        )

    if time.ndim != 1:
        raise ValueError("time 必须是一维数组")
    if position.shape != (time.size, 3):
        raise ValueError("position 必须为 N x 3 数组")
    if euler_deg.shape != (time.size, 3):
        raise ValueError("euler_deg 必须为 N x 3 数组，顺序为 [psi, phi, theta]")
    if position_ref is not None and position_ref.shape != position.shape:
        raise ValueError("position_ref 的形状必须与 position 相同")
    if time.size < 2:
        raise ValueError("日志至少需要两个时间采样点")
    if not np.all(np.isfinite(position)) or not np.all(np.isfinite(euler_deg)):
        raise ValueError("日志中包含 NaN 或无穷值")

    return time, position, euler_deg, position_ref


def create_animation(
    time,
    position_ned,
    euler_deg,
    position_ref_ned=None,
    axis_length=1.2,
    stride=5,
    speed=1.0,
    repeat=True,
    title="Tailsitter 3D Position and Attitude",
):
    """创建三维轨迹与机体轴动画，返回 figure、animation 和播放帧率。"""
    if stride < 1:
        raise ValueError("stride 必须大于等于 1")
    if speed <= 0.0:
        raise ValueError("speed 必须大于 0")
    if axis_length <= 0.0:
        raise ValueError("axis_length 必须大于 0")

    position = ned_to_display(position_ned)
    position_ref = (
        ned_to_display(position_ref_ned) if position_ref_ned is not None else None
    )

    frame_indices = np.arange(0, time.size, stride, dtype=int)
    if frame_indices[-1] != time.size - 1:
        frame_indices = np.append(frame_indices, time.size - 1)

    sampled_dt = np.diff(time[frame_indices])
    positive_dt = sampled_dt[sampled_dt > 0.0]
    if positive_dt.size == 0:
        raise ValueError("日志时间必须递增")
    interval_ms = 1000.0 * np.median(positive_dt) / speed
    fps = 1000.0 / interval_ms

    fig = plt.figure(figsize=(11, 9))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlabel("North X (m)")
    ax.set_ylabel("East Y (m)")
    ax.set_zlabel("Altitude -Z_NED (m)")
    ax.set_title(title)
    ax.grid(True)
    ax.view_init(elev=25.0, azim=-55.0)

    limit_points = position
    if position_ref is not None:
        limit_points = np.vstack((limit_points, position_ref))
        ax.plot(
            position_ref[:, 0],
            position_ref[:, 1],
            position_ref[:, 2],
            color="0.65",
            linestyle="--",
            linewidth=1.2,
            label="Reference trajectory",
        )
    set_equal_3d_limits(ax, limit_points, margin=1.5 * axis_length)

    trajectory_line, = ax.plot([], [], [], color="black", linewidth=1.5, label="Flown trajectory")
    aircraft_point, = ax.plot([], [], [], marker="o", color="black", markersize=5)
    body_x_line, = ax.plot([], [], [], color="red", linewidth=3.0, label=r"$b_x$")
    body_y_line, = ax.plot([], [], [], color="green", linewidth=3.0, label=r"$b_y$")
    body_z_line, = ax.plot([], [], [], color="blue", linewidth=3.0, label=r"$b_z$")
    body_lines = (body_x_line, body_y_line, body_z_line)
    time_text = ax.text2D(0.02, 0.97, "", transform=ax.transAxes, va="top")
    angle_text = ax.text2D(0.02, 0.91, "", transform=ax.transAxes, va="top")
    ax.legend(loc="upper right")

    def update(sample_index):
        position_now = position[sample_index]
        trajectory_line.set_data(position[: sample_index + 1, 0], position[: sample_index + 1, 1])
        trajectory_line.set_3d_properties(position[: sample_index + 1, 2])

        aircraft_point.set_data([position_now[0]], [position_now[1]])
        aircraft_point.set_3d_properties([position_now[2]])

        R_b_to_i = zxy_euler_to_rotation_matrix(euler_deg[sample_index])
        body_axes_display = ned_to_display(R_b_to_i.T).T
        for axis_index, line in enumerate(body_lines):
            endpoint = position_now + axis_length * body_axes_display[:, axis_index]
            line.set_data(
                [position_now[0], endpoint[0]],
                [position_now[1], endpoint[1]],
            )
            line.set_3d_properties([position_now[2], endpoint[2]])

        psi, phi, theta = euler_deg[sample_index]
        time_text.set_text(f"t = {time[sample_index]:.2f} s")
        angle_text.set_text(
            f"ZXY: psi={psi:.1f} deg, phi={phi:.1f} deg, theta={theta:.1f} deg"
        )
        return (
            trajectory_line,
            aircraft_point,
            *body_lines,
            time_text,
            angle_text,
        )

    animation = FuncAnimation(
        fig,
        update,
        frames=frame_indices,
        interval=interval_ms,
        repeat=repeat,
        blit=False,
    )
    return fig, animation, fps


def save_animation(animation, fps, output_path, dpi=120):
    """将动画保存为 GIF 或 MP4，并给出缺少编码器时的明确提示。"""
    output_path = Path(output_path)
    suffix = output_path.suffix.lower()
    if suffix == ".gif":
        if not PillowWriter.isAvailable():
            raise RuntimeError("当前 Matplotlib 环境中 Pillow GIF writer 不可用")
        writer = PillowWriter(fps=fps)
    elif suffix == ".mp4":
        if not FFMpegWriter.isAvailable():
            raise RuntimeError("保存 MP4 需要先安装 FFmpeg；也可以改用 .gif")
        writer = FFMpegWriter(fps=fps)
    else:
        raise ValueError("动画输出仅支持 .gif 或 .mp4 文件")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    animation.save(output_path, writer=writer, dpi=dpi)
    print(f"动画已保存到 {output_path}")


def _preset_value(args_value, preset, key, fallback):
    """命令行显式值优先，其次使用轨迹预设，最后使用通用默认值。"""
    if args_value is not None:
        return args_value
    if preset is not None:
        return preset[key]
    return fallback


def animate_log(
    log_path,
    title,
    axis_length,
    stride,
    speed,
    repeat,
    save_path=None,
    dpi=120,
    show=True,
):
    """读取一个日志并显示或保存对应三维动画。"""
    log_path = Path(log_path)
    if not log_path.exists():
        raise FileNotFoundError(
            f"找不到仿真日志 {log_path}。请先运行 python validate_paper_trajectories.py"
        )

    time, position, euler_deg, position_ref = load_flight_log(log_path)
    fig, animation, fps = create_animation(
        time,
        position,
        euler_deg,
        position_ref,
        axis_length=axis_length,
        stride=stride,
        speed=speed,
        repeat=repeat,
        title=title,
    )

    if save_path is not None:
        save_animation(animation, fps, save_path, dpi=dpi)

    if show:
        plt.show()
    else:
        plt.close(fig)


def parse_args():
    parser = argparse.ArgumentParser(
        description="将 main_sim.py 生成的位置和 ZXY 欧拉角日志绘制为三维动画"
    )
    parser.add_argument(
        "log",
        nargs="?",
        type=Path,
        help="直接指定 NPZ 日志；未指定轨迹预设时默认使用 flight_log.npz",
    )
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument(
        "--trajectory",
        choices=tuple(TRAJECTORY_PRESETS),
        help="直接选择论文轨迹及其 validation_logs 日志",
    )
    selection.add_argument(
        "--all-trajectories",
        action="store_true",
        help="分别导出全部七种论文轨迹动画",
    )
    parser.add_argument("--axis-length", type=float, default=1.2, help="机体轴显示长度，单位 m")
    parser.add_argument("--stride", type=int, help="抽帧间隔；默认使用各轨迹预设值")
    parser.add_argument("--speed", type=float, help="播放速度倍数；默认使用各轨迹预设值")
    parser.add_argument("--save", type=Path, help="保存为 .gif 或 .mp4")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("trajectory_animations"),
        help="--all-trajectories 的输出目录",
    )
    parser.add_argument(
        "--format",
        choices=("gif", "mp4"),
        default="gif",
        help="批量导出格式，默认 gif",
    )
    parser.add_argument("--dpi", type=int, default=120, help="保存动画的 DPI")
    parser.add_argument("--no-repeat", action="store_true", help="播放结束后不循环")
    parser.add_argument("--no-show", action="store_true", help="不打开交互窗口，通常与 --save 合用")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.all_trajectories:
        if args.log is not None or args.save is not None:
            raise ValueError(
                "--all-trajectories 不能与位置日志参数或 --save 同时使用；"
                "请使用 --output-dir 和 --format"
            )
        for trajectory_name in BATCH_TRAJECTORIES:
            preset = TRAJECTORY_PRESETS[trajectory_name]
            stride = _preset_value(args.stride, preset, "stride", 5)
            speed = _preset_value(args.speed, preset, "speed", 1.0)
            output_path = args.output_dir / f"{trajectory_name}.{args.format}"
            print(f"正在生成 {trajectory_name}...")
            animate_log(
                preset["log"],
                preset["title"],
                args.axis_length,
                stride,
                speed,
                repeat=False,
                save_path=output_path,
                dpi=args.dpi,
                show=False,
            )
        return

    if args.trajectory is not None and args.log is not None:
        raise ValueError("位置日志参数不能与 --trajectory 同时使用")

    preset = (
        TRAJECTORY_PRESETS[args.trajectory]
        if args.trajectory is not None
        else None
    )
    log_path = (
        preset["log"]
        if preset is not None
        else (args.log if args.log is not None else Path("flight_log.npz"))
    )
    title = preset["title"] if preset is not None else "Tailsitter 3D Position and Attitude"
    stride = _preset_value(args.stride, preset, "stride", 5)
    speed = _preset_value(args.speed, preset, "speed", 1.0)
    animate_log(
        log_path,
        title,
        args.axis_length,
        stride,
        speed,
        repeat=not args.no_repeat,
        save_path=args.save,
        dpi=args.dpi,
        show=not args.no_show,
    )


if __name__ == "__main__":
    main()
