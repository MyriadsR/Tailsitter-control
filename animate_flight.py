"""使用 PyVista/VTK 将仿真日志绘制为尾座飞行器三维轨迹动画。"""

import argparse
import time as wall_time
from pathlib import Path

import numpy as np
import pyvista as pv


PROJECT_ROOT = Path(__file__).resolve().parent
DEFAULT_MODEL_PATH = PROJECT_ROOT / "vis_vtol.stl"

# vis_vtol.stl 到项目机体系的固定坐标映射：
#   STL +x -> body +z, STL +y -> body -y, STL +z -> body +x。
# 矩阵的每一列分别是 STL 三个单位轴在机体系中的表示。
STL_TO_BODY_ROTATION = np.array(
    [
        [0.0, 0.0, 1.0],
        [0.0, -1.0, 0.0],
        [1.0, 0.0, 0.0],
    ]
)

# NED 世界系转为动画使用的 North-East-Up 显示坐标系。
NED_TO_DISPLAY_ROTATION = np.diag([1.0, 1.0, -1.0])

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


def prepare_stl_model(path, scale=1.0, origin_stl=None):
    """读取 STL，并将其居中、缩放、转换到项目机体系。"""
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f"找不到 STL 模型 {path}")
    if scale <= 0.0:
        raise ValueError("model_scale 必须大于 0")

    mesh = pv.read(path)
    if isinstance(mesh, pv.MultiBlock):
        mesh = mesh.combine().extract_surface()
    mesh = mesh.triangulate()
    if mesh.n_points == 0 or mesh.n_cells == 0:
        raise ValueError(f"{path} 中没有有效的 STL 三角面")

    vertices_stl = np.asarray(mesh.points, dtype=float)
    if not np.all(np.isfinite(vertices_stl)):
        raise ValueError(f"{path} 中包含 NaN 或无穷顶点")
    minimum = np.min(vertices_stl, axis=0)
    maximum = np.max(vertices_stl, axis=0)
    if origin_stl is None:
        origin_stl = 0.5 * (minimum + maximum)
    else:
        origin_stl = np.asarray(origin_stl, dtype=float)
        if origin_stl.shape != (3,) or not np.all(np.isfinite(origin_stl)):
            raise ValueError("model_origin 必须是三个有限数值")

    centered_scaled = scale * (vertices_stl - origin_stl)
    mesh.points = centered_scaled @ STL_TO_BODY_ROTATION.T
    mesh.compute_normals(
        cell_normals=True,
        point_normals=True,
        consistent_normals=False,
        auto_orient_normals=False,
        inplace=True,
    )
    extent_stl = maximum - minimum
    return mesh, origin_stl, extent_stl


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
    model_mesh_body=None,
    axis_length=1.2,
    stride=5,
    speed=1.0,
    repeat=True,
    show_body_axes=False,
    title="Tailsitter 3D Position and Attitude",
    off_screen=False,
    window_size=(1104, 912),
):
    """创建 PyVista/VTK 场景；模型网格保持不变，每帧只更新 Actor 变换。"""
    if stride < 1:
        raise ValueError("stride 必须大于等于 1")
    if speed <= 0.0:
        raise ValueError("speed 必须大于 0")
    if show_body_axes and axis_length <= 0.0:
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

    plotter = pv.Plotter(off_screen=off_screen, window_size=window_size)
    plotter.set_background("#FAFAFA")
    plotter.add_title(title, font_size=14, color="#202020")

    # 轨迹只上传到 VTK 一次。飞行器运动时不再重建线或三角面。
    limit_points = position
    if position_ref is not None:
        limit_points = np.vstack((limit_points, position_ref))
        plotter.add_mesh(
            pv.lines_from_points(position_ref),
            color="#A6A6A6",
            line_width=2.0,
            label="Ref.",
            lighting=False,
        )
    plotter.add_mesh(
        pv.lines_from_points(position),
        color="#202020",
        line_width=3.0,
        label="Flight",
        lighting=False,
    )

    if model_mesh_body is not None:
        aircraft_actor = plotter.add_mesh(
            model_mesh_body,
            color="#3988D1",
            smooth_shading=True,
            split_sharp_edges=True,
            ambient=0.20,
            diffuse=0.75,
            specular=0.30,
            specular_power=25.0,
            pickable=False,
        )
        model_radius = float(np.max(np.linalg.norm(model_mesh_body.points, axis=1)))
    else:
        marker_radius = max(0.03 * np.ptp(limit_points, axis=0).max(), 0.05)
        aircraft_actor = plotter.add_mesh(
            pv.Sphere(radius=marker_radius), color="#202020", pickable=False
        )
        model_radius = marker_radius

    body_axis_actors = []
    if show_body_axes:
        for direction, color in zip(np.eye(3), ("red", "green", "blue")):
            arrow = pv.Arrow(
                start=(0.0, 0.0, 0.0),
                direction=direction,
                scale=axis_length,
                tip_length=0.18,
                shaft_radius=0.02,
                tip_radius=0.05,
            )
            body_axis_actors.append(
                plotter.add_mesh(arrow, color=color, lighting=False, pickable=False)
            )

    telemetry = plotter.add_text(
        "",
        position=(18, 40),
        font_size=11,
        color="#202020",
        name="telemetry",
    )
    plotter.add_legend(
        bcolor=(1.0, 1.0, 1.0),
        face="rectangle",
        size=(0.22, 0.10),
        loc="upper right",
    )

    minimum = np.min(limit_points, axis=0)
    maximum = np.max(limit_points, axis=0)
    center = 0.5 * (minimum + maximum)
    margin = max(1.5 * model_radius, 1.5 * axis_length if show_body_axes else 0.0, 0.5)
    half_range = max(0.5 * np.max(maximum - minimum) + margin, 1.0)
    bounds = (
        center[0] - half_range,
        center[0] + half_range,
        center[1] - half_range,
        center[1] + half_range,
        center[2] - half_range,
        center[2] + half_range,
    )
    plotter.show_grid(
        bounds=bounds,
        xtitle="North X (m)",
        ytitle="East Y (m)",
        ztitle="Altitude -Z_NED (m)",
        color="#707070",
        grid="back",
        location="outer",
        ticks="outside",
    )

    camera_distance = 2.6 * half_range
    view_direction = np.array([0.52, -0.74, 0.42])
    plotter.camera_position = [
        center + camera_distance * view_direction,
        center,
        (0.0, 0.0, 1.0),
    ]
    plotter.camera.clipping_range = (0.01, 10.0 * camera_distance)

    def update(sample_index):
        position_now = position[sample_index]
        R_b_to_i = zxy_euler_to_rotation_matrix(euler_deg[sample_index])
        R_b_to_display = NED_TO_DISPLAY_ROTATION @ R_b_to_i
        pose_matrix = np.eye(4)
        pose_matrix[:3, :3] = R_b_to_display
        pose_matrix[:3, 3] = position_now
        aircraft_actor.user_matrix = pose_matrix
        for axis_actor in body_axis_actors:
            axis_actor.user_matrix = pose_matrix

        psi, phi, theta = euler_deg[sample_index]
        telemetry.input = (
            f"t = {time[sample_index]:.2f} s\n"
            f"ZXY: psi={psi:.1f} deg, phi={phi:.1f} deg, theta={theta:.1f} deg"
        )

    update(int(frame_indices[0]))
    return {
        "plotter": plotter,
        "update": update,
        "frame_indices": frame_indices,
        "interval_ms": interval_ms,
        "fps": fps,
        "repeat": repeat,
    }


def save_animation(scene, output_path):
    """使用 PyVista 的 imageio writer 导出 GIF 或 MP4。"""
    output_path = Path(output_path)
    suffix = output_path.suffix.lower()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    plotter = scene["plotter"]
    fps = scene["fps"]
    if suffix == ".gif":
        plotter.open_gif(output_path, fps=fps)
    elif suffix == ".mp4":
        plotter.open_movie(output_path, framerate=max(1, int(round(fps))), quality=7)
    else:
        raise ValueError("动画输出仅支持 .gif 或 .mp4 文件")

    start = wall_time.perf_counter()
    for sample_index in scene["frame_indices"]:
        scene["update"](int(sample_index))
        plotter.write_frame()
    elapsed = wall_time.perf_counter() - start
    frame_count = len(scene["frame_indices"])
    plotter.close()
    render_fps = frame_count / elapsed if elapsed > 0.0 else float("inf")
    print(
        f"动画已保存到 {output_path}（{frame_count} 帧，"
        f"平均导出速度 {render_fps:.1f} 帧/秒）"
    )


def show_animation(scene):
    """通过 VTK 定时器在交互窗口中播放动画。"""
    plotter = scene["plotter"]
    frame_indices = scene["frame_indices"]

    def timer_callback(step):
        scene["update"](int(frame_indices[step % len(frame_indices)]))
        plotter.render()

    max_steps = 2_147_483_647 if scene["repeat"] else len(frame_indices)
    plotter.add_timer_event(
        max_steps=max_steps,
        duration=max(1, int(round(scene["interval_ms"]))),
        callback=timer_callback,
    )
    plotter.show()


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
    model_path=DEFAULT_MODEL_PATH,
    model_scale=1.0,
    model_origin=None,
    show_body_axes=False,
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

    model_mesh_body = None
    if model_path is not None:
        model_mesh_body, origin_stl, extent_stl = prepare_stl_model(
            model_path, scale=model_scale, origin_stl=model_origin
        )
        print(
            f"已加载 STL 模型 {model_path}: {model_mesh_body.n_cells} 个三角面, "
            f"原始尺寸={extent_stl}, STL 原点={origin_stl}, 缩放={model_scale:g}"
        )

    time, position, euler_deg, position_ref = load_flight_log(log_path)
    window_scale = dpi / 120.0
    window_size = (
        max(320, 16 * int(round(69 * window_scale))),
        max(240, 16 * int(round(57 * window_scale))),
    )

    def build_scene(off_screen):
        return create_animation(
            time,
            position,
            euler_deg,
            position_ref,
            model_mesh_body=model_mesh_body,
            axis_length=axis_length,
            stride=stride,
            speed=speed,
            repeat=repeat,
            show_body_axes=show_body_axes,
            title=title,
            off_screen=off_screen,
            window_size=window_size,
        )

    if save_path is not None:
        save_animation(build_scene(off_screen=True), save_path)

    if show:
        show_animation(build_scene(off_screen=False))


def parse_args():
    parser = argparse.ArgumentParser(
        description="使用 STL 飞行器模型显示 main_sim.py 生成的三维轨迹与姿态"
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
    parser.add_argument(
        "--model",
        type=Path,
        default=DEFAULT_MODEL_PATH,
        help=f"STL 飞行器模型，默认 {DEFAULT_MODEL_PATH.name}",
    )
    parser.add_argument(
        "--no-model",
        action="store_true",
        help="不显示 STL 模型，仅显示位置点（用于排查问题）",
    )
    parser.add_argument(
        "--model-scale",
        type=float,
        default=1.0,
        help="STL 模型缩放倍数，默认 1.0",
    )
    parser.add_argument(
        "--model-origin",
        nargs=3,
        type=float,
        metavar=("X", "Y", "Z"),
        help="STL 坐标中的旋转中心；默认使用模型包围盒中心",
    )
    parser.add_argument(
        "--show-body-axes",
        action="store_true",
        help="在 STL 上叠加机体三轴，用于检查模型坐标映射",
    )
    parser.add_argument(
        "--axis-length",
        type=float,
        default=1.2,
        help="--show-body-axes 的机体轴长度，单位 m",
    )
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
    parser.add_argument(
        "--dpi",
        type=int,
        default=120,
        help="输出分辨率缩放基准；120 对应 1104x912",
    )
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
                model_path=None if args.no_model else args.model,
                model_scale=args.model_scale,
                model_origin=args.model_origin,
                show_body_axes=args.show_body_axes,
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
        model_path=None if args.no_model else args.model,
        model_scale=args.model_scale,
        model_origin=args.model_origin,
        show_body_axes=args.show_body_axes,
        save_path=args.save,
        dpi=args.dpi,
        show=not args.no_show,
    )


if __name__ == "__main__":
    main()
