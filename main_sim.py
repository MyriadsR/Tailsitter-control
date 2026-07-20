'''
Author: ZR
Description: Tailsitter 飞翼无人机全系统闭环仿真主程序
'''
import argparse

import numpy as np
import matplotlib.pyplot as plt

# 导入配置和物理环境
from config import TailsitterConfig
from environment.dynamics import UAVDynamics
from environment.aerodynamics import Aerodynamics

# 导入所有控制模块
from controller.pos_ctrl import PositionController
from controller.INDI_acc_ctrl import INDILinearController
from controller.att_ctrl import FlatnessAttitudeController, PDAttitudeController, INDIAngularController
from controller.actuator_ctrl import ControlAllocator
from trajectory.test import (
    CircularTrajectory,
    CircularTransitionTrajectory,
    DifferentialThrustTurnTrajectory,
    KnifeEdgeTransitionTrajectory,
    LemniscateTrajectory,
    TrajectoryGenerator,
)

# 导入数学工具
from utils.math_help import zxy_euler_to_quaternion


def zxy_euler_from_quaternion(q):
    """工具函数：将四元数转换为 ZXY 欧拉角 [psi, phi, theta]。"""
    qw, qx, qy, qz = q
    # ZXY 提取（对应关系：phi=Roll, theta=Pitch, psi=Yaw）
    sin_phi = np.clip(2.0 * (qw * qx + qy * qz), -1.0, 1.0)
    phi = np.arcsin(sin_phi)
    theta = np.arctan2(2.0 * (qw * qy - qx * qz), 1.0 - 2.0 * (qx**2 + qy**2))
    psi = np.arctan2(2.0 * (qw * qz - qx * qy), 1.0 - 2.0 * (qx**2 + qz**2))
    return np.array([psi, phi, theta])


def rk4_step(dynamics_model, state, f_alpha, f_ext, moments, m_ext, R_b_alpha, dt):
    """自定义 4 阶龙格-库塔步进器，提升仿真速度"""
    def f(s):
        return dynamics_model.compute_state_derivatives(0.0, s, f_alpha, f_ext, moments, m_ext, R_b_alpha)
    
    k1 = f(state)
    k2 = f(state + 0.5 * dt * k1)
    k3 = f(state + 0.5 * dt * k2)
    k4 = f(state + dt * k3)
    
    next_state = state + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
    # 四元数归一化保护
    q = next_state[6:10]
    next_state[6:10] = q / np.linalg.norm(q)
    return next_state


def main(
    trajectory_name="segments",
    laps=None,
    speed=None,
    lap_time=None,
    log_path="flight_log.npz",
    show_plot=True,
):
    print("初始化仿真环境...")
    cfg = TailsitterConfig()
    
    # 实例化环境
    dyn = UAVDynamics(cfg.mass, cfg.inertia_matrix, cfg.g)
    aero = Aerodynamics(cfg)
    
    # 实例化控制器
    pos_ctrl = PositionController(cfg)
    indi_lin = INDILinearController(cfg)
    att_flat = FlatnessAttitudeController(cfg)
    att_pd   = PDAttitudeController(cfg)
    indi_ang = INDIAngularController(cfg)
    allocator= ControlAllocator(cfg)
    if trajectory_name == "lemniscate":
        trajectory = LemniscateTrajectory(
            speed=6.0 if speed is None else speed,
            lap_time=7.0 if lap_time is None else lap_time,
            laps=8 if laps is None else laps,
        )
    elif trajectory_name in {"knife-edge-transition", "circle-knife-edge"}:
        # Sec. VI-C：椭圆跑道一侧 knife edge、另一侧 coordinated。
        # circle-knife-edge 保留为兼容别名，避免旧命令继续误入 Sec. VI-D
        # 的定半径圆；推荐使用语义明确的 knife-edge-transition。
        trajectory = KnifeEdgeTransitionTrajectory(
            speed=6.0 if speed is None else speed,
            lap_time=6.25 if lap_time is None else lap_time,
            laps=8 if laps is None else laps,
        )
    elif trajectory_name in {"circle-coordinated", "circular-knife-edge"}:
        # Sec. VI-D：半径 3.5 m 的定常圆轨迹。
        trajectory = CircularTrajectory(
            mode=(
                "coordinated"
                if trajectory_name == "circle-coordinated"
                else "knife_edge"
            ),
            speed=8.1 if speed is None else speed,
            laps=4 if laps is None else laps,
        )
    elif trajectory_name in {"transition-to-forward", "transition-to-hover"}:
        transition_duration = 3.0
        target_speed = 8.1 if speed is None else speed
        trajectory = CircularTransitionTrajectory(
            transition=(
                "to_forward"
                if trajectory_name == "transition-to-forward"
                else "to_hover"
            ),
            tangential_acceleration=target_speed / transition_duration,
            duration=transition_duration,
        )
    elif trajectory_name == "differential-turn":
        trajectory = DifferentialThrustTurnTrajectory(
            speed=7.0 if speed is None else speed,
        )
    elif trajectory_name == "segments":
        trajectory = TrajectoryGenerator()
    else:
        raise ValueError(f"未知轨迹类型: {trajectory_name}")
    
    # === 仿真参数 ===
    dt = 0.01  # 100Hz 控制频率
    t_end = trajectory.total_duration
    steps = int(t_end / dt) + 1

    # 预计算零升力系与机体系之间的常数旋转矩阵。
    ca0, sa0 = np.cos(cfg.alpha_0), np.sin(cfg.alpha_0)
    R_b_to_alpha = np.array([[ca0, 0, sa0], [0, 1, 0], [-sa0, 0, ca0]])
    R_alpha_to_b = R_b_to_alpha.T
    
    # === 初始状态 ===
    initial_state_mode = trajectory.initial_state_mode
    initial_flap_deflection = 0.0
    if initial_state_mode == "reference":
        # 论文试验是在已经进入双纽线的定常飞行状态下开始记录，而不是从
        # 地面静止瞬间跳变到 6 m/s。用 t=0 的平坦输出初始化状态和推力，
        # 使闭环误差主要反映跟踪能力而非不连续初值造成的巨大瞬态。
        pos_0, vel_0, acc_0, jerk_0, psi_0, psi_dot_0 = trajectory.sample(0.0)
        q_seed = zxy_euler_to_quaternion(psi_0, 0.0, 0.0)
        f_ref_0 = cfg.mass * (acc_0 - np.array([0.0, 0.0, cfg.g]))
        # 定常圆周飞行中 c_mu_T 需要由同向舵面力矩抵消。
        # 舵面配平又会改变平坦性变换的姿态和总推力，因此在
        # 启动仿真前做少量不动点迭代，避免从零舵偏瞬间跳到配平值。
        initial_flap_deflection = -0.27
        q_0 = q_seed
        for _ in range(12):
            q_0, thrust_0, phi_0, theta_bar_0 = (
                att_flat.compute_desired_attitude_and_thrust(
                    f_ref_0,
                    vel_0,
                    2.0 * initial_flap_deflection,
                    psi_0,
                    q_0,
                    return_angles=True,
                )
            )
            R_i_to_b_trim = dyn._quat_to_rot_matrix(q_0).T
            v_b_trim = R_i_to_b_trim @ vel_0
            v_alpha_x_trim = ca0 * v_b_trim[0] + sa0 * v_b_trim[2]
            flap_force_slope = (
                cfg.c_LT_delta * np.cos(cfg.alpha_0 + cfg.alpha_T) * thrust_0
                + 2.0
                * cfg.c_LV_delta
                * np.linalg.norm(vel_0)
                * v_alpha_x_trim
            )
            if abs(flap_force_slope) > 1e-8:
                trim_target = (
                    cfg.c_mu_T * thrust_0 / (cfg.l_dx * flap_force_slope)
                )
                trim_target = np.clip(
                    trim_target, -cfg.delta_max, cfg.delta_max
                )
                initial_flap_deflection = (
                    0.5 * initial_flap_deflection + 0.5 * trim_target
                )

        # 使最终姿态/推力严格对应迭代后的舵面配平值。
        q_0, thrust_0, phi_0, theta_bar_0 = (
            att_flat.compute_desired_attitude_and_thrust(
                f_ref_0,
                vel_0,
                2.0 * initial_flap_deflection,
                psi_0,
                q_0,
                return_angles=True,
            )
        )
        omega_0 = att_flat.compute_feedforward_angular_velocity(
            vel_0,
            acc_0,
            jerk_0,
            psi_0,
            psi_dot_0,
            f_ref_0,
            phi_0,
            theta_bar_0,
            2.0 * initial_flap_deflection,
        )
        initial_rotor_speed = np.sqrt(max(thrust_0, 0.0) / (2.0 * cfg.c_T))
        omega_rotors = np.full(2, initial_rotor_speed)
    elif initial_state_mode == "hover":
        # 加速 transition 的参考在 t=0 存在切向加速度阶跃，但论文试验从
        # 静止悬停进入该轨迹。因此状态按同一位置和 yaw 下的真实悬停初始化。
        pos_0, _, _, _, psi_0, _ = trajectory.sample(0.0)
        vel_0 = np.zeros(3)
        omega_0 = np.zeros(3)
        q_seed = zxy_euler_to_quaternion(psi_0, 0.0, np.pi / 2.0)
        f_hover = -cfg.mass * np.array([0.0, 0.0, cfg.g])
        initial_flap_deflection = -0.27
        q_0, thrust_0 = att_flat.compute_desired_attitude_and_thrust(
            f_hover,
            vel_0,
            2.0 * initial_flap_deflection,
            psi_0,
            q_seed,
        )
        initial_rotor_speed = np.sqrt(max(thrust_0, 0.0) / (2.0 * cfg.c_T))
        omega_rotors = np.full(2, initial_rotor_speed)
    elif initial_state_mode == "ground":
        # NED 坐标系下地面起飞，地面高度设为 Z = 0。
        pos_0 = np.array([0.0, 0.0, 0.0])
        vel_0 = np.array([0.0, 0.0, 0.0])

        # Tailsitter 悬停时机头朝上，推力抵消重力，即机体 X 轴指向世界 -Z。
        q_0 = zxy_euler_to_quaternion(0.0, 0.0, np.pi/2)
        omega_0 = np.array([0.0, 0.0, 0.0])
        omega_rotors = np.zeros(2)
    else:
        raise ValueError(f"未知初始状态模式: {initial_state_mode}")
    
    state = np.concatenate([pos_0, vel_0, q_0, omega_0])
    
    # 初始执行器状态
    delta_flaps = np.full(2, initial_flap_deflection)

    if initial_state_mode != "ground":
        R_i_to_b_0 = dyn._quat_to_rot_matrix(q_0).T
        v_b_0 = R_i_to_b_0 @ vel_0
        f_alpha_lpf, m_lpf, _ = aero.compute_forces_and_moments(
            v_b_0,
            omega_rotors[0],
            omega_rotors[1],
            delta_flaps[0],
            delta_flaps[1],
        )
        state_dot_0 = dyn.compute_state_derivatives(
            0.0,
            state,
            f_alpha_lpf,
            np.zeros(3),
            m_lpf,
            np.zeros(3),
            R_alpha_to_b,
        )
        a_lpf = state_dot_0[3:6].copy()
        a_lpf_tilde = a_lpf.copy()
        omega_dot_lpf = state_dot_0[10:13].copy()
    else:
        # === INDI 滤波器初始状态 (低通滤波后的历史值) ===
        a_lpf = np.array([0.0, 0.0, cfg.g])
        a_lpf_tilde = np.array([0.0, 0.0, cfg.g])
        omega_dot_lpf = np.zeros(3)
        f_alpha_lpf = np.zeros(3)
        m_lpf = np.zeros(3)

    # 执行器测量滤波状态。delta_flaps_slow 是去除瞬态后的低频舵面分量，
    # 同时用于平坦性变换和 Eq. (46) 中的 f_lpf，保证 INDI 两边信号一致。
    omega_rotors_lpf = omega_rotors.copy()
    delta_flaps_lpf = delta_flaps.copy()
    delta_flaps_slow = delta_flaps.copy()
    
    # 记录数据用的列表
    log_t, log_pos, log_vel, log_acceleration = [], [], [], []
    log_euler, log_euler_command = [], []
    log_angular_velocity, log_angular_velocity_ref = [], []
    log_angular_acceleration, log_angular_acceleration_lpf = [], []
    log_angular_acceleration_command = []
    log_rotors, log_flaps = [], []
    log_x_ref, log_v_ref, log_a_ref, log_psi_ref = [], [], [], []
    
    print(f"开始仿真运行，轨迹: {trajectory_name}...")
    for step in range(steps):
        t = step * dt
        
        # 1. 解析当前状态
        pos = state[0:3]
        vel = state[3:6]
        q_curr = state[6:10]
        omega_curr = state[10:13]
        R_i_to_b = dyn._quat_to_rot_matrix(q_curr).T
        R_b_to_i = R_i_to_b.T
        
        # 2. 从独立轨迹模块读取位置及其导数、偏航角和偏航角速度参考。
        x_ref, v_ref, a_ref, j_ref, psi_ref, psi_dot_ref = trajectory.sample(t)
        
        # 3. 外环控制 (位置 -> 期望加速度 -> INDI 期望力)
        a_c = pos_ctrl.compute_acceleration_command(x_ref, v_ref, a_ref, pos, vel, a_lpf_tilde, R_b_to_i)
        # a_c = a_c + gravity_comp_i  # 加上重力补偿
        f_c_i = indi_lin.compute_force_command(a_c, a_lpf_tilde, f_alpha_lpf, R_b_to_i, R_alpha_to_b)
        
        # 4. 微分平坦变换 (受力 -> 姿态与推力)
        # 姿态/推力指令基于 INDI 实际力指令 f_c_i (含反馈, 论文 Eq 46)
        # 论文 Sec. III.A 只在线加速度控制中使用近似恒定的低频舵面分量。
        delta_sum = np.sum(delta_flaps_slow)
        q_c, T_c = att_flat.compute_desired_attitude_and_thrust(f_c_i, vel, delta_sum, psi_ref, q_curr)

        # 平坦前馈角速度 Omega_ref: 严格由"参考轨迹"导出 (论文 Sec III.B), 与含反馈的
        # 指令路径解耦。参考力 f_ref^i = m(a_ref - g*i_z) (Eq 16, 设 f_ext=0), 并用参考力
        # 解算对应的参考姿态 (phi_ref, theta_bar_ref), 再据此求 Omega_ref。
        f_ref_i = cfg.mass * (a_ref - np.array([0.0, 0.0, cfg.g]))
        # 直接使用平坦性变换选取的原始角度分支。若先转换为四元数再提取欧拉角，
        # 倒飞时的 phi+k*pi 分支会被折叠到主值范围，导致 Omega_ref 与 q_ref 不一致。
        _, _, phi_ref, theta_bar_ref = att_flat.compute_desired_attitude_and_thrust(
            f_ref_i,
            v_ref,
            delta_sum,
            psi_ref,
            q_curr,
            return_angles=True,
        )
        omega_ref = att_flat.compute_feedforward_angular_velocity(
            v_ref, a_ref, j_ref, psi_ref, psi_dot_ref, f_ref_i, phi_ref, theta_bar_ref, delta_sum
        )
        
        # 5. 内环控制 (姿态 -> 期望角加速度 -> INDI 期望力矩)
        omega_dot_c = att_pd.compute_angular_acceleration_command(q_curr, q_c, omega_curr, omega_ref)
        m_c = indi_ang.compute_moment_command(omega_dot_c, omega_dot_lpf, m_lpf)
        
        # 6. 控制分配 (推力、力矩 -> 电机转速、舵偏)
        v_b = R_i_to_b @ vel
        omega_cmd, delta_cmd = allocator.allocate_controls(T_c, m_c, v_b)
        
        # 模拟电机和舵机的不同一阶动态。舵面另施加带载角速率
        # 限制，避免单个仿真步内产生不可实现的大幅跳变。
        motor_alpha = 1.0 - np.exp(-dt / cfg.motor_time_constant)
        servo_alpha = 1.0 - np.exp(-dt / cfg.servo_time_constant)
        omega_rotors += motor_alpha * (omega_cmd - omega_rotors)
        omega_rotors = np.clip(omega_rotors, cfg.omega_min, cfg.omega_max)

        delta_step = servo_alpha * (delta_cmd - delta_flaps)
        max_delta_step = cfg.servo_rate_max * dt
        delta_flaps += np.clip(delta_step, -max_delta_step, max_delta_step)
        delta_flaps = np.clip(delta_flaps, -cfg.delta_max, cfg.delta_max)
        
        # 7. 物理环境计算真实的受力与力矩
        f_alpha_real, m_body_real, _ = aero.compute_forces_and_moments(
            v_b, omega_rotors[0], omega_rotors[1], delta_flaps[0], delta_flaps[1]
        )
        
        # 8. 物理状态积分步进
        # 假设没有外界阵风扰动 f_ext = 0, m_ext = 0
        state_dot = dyn.compute_state_derivatives(t, state, f_alpha_real, np.zeros(3), m_body_real, np.zeros(3), R_alpha_to_b)
        state = rk4_step(dyn, state, f_alpha_real, np.zeros(3), m_body_real, np.zeros(3), R_alpha_to_b, dt)

        # 简化地面约束：NED 下 Z>0 代表穿入地面，做硬约束防穿地
        if state[2] > 0.0:
            state[2] = 0.0
            if state[5] > 0.0:
                state[5] = 0.0
        
        # 9. 更新 INDI 滤波器状态 (在实际代码中来自传感器)
        alpha_lpf = 0.35   # 主低通滤波系数（100 Hz 下离散 -3 dB 约 7 Hz）
        alpha_slow = 0.06  # 慢变低通系数（约 1 Hz），用于构造舵面带通分量

        # (a) 世界系实际加速度 (= v_dot, 已含重力, 对应论文 a_lpf)
        a_real_i = state_dot[3:6]
        a_lpf = alpha_lpf * a_real_i + (1 - alpha_lpf) * a_lpf

        # (b) 对执行器测量施加与 IMU 主低通相匹配的滤波，再从舵面低通量中
        #     分离慢变分量与带通瞬态分量。
        omega_rotors_lpf = alpha_lpf * omega_rotors + (1 - alpha_lpf) * omega_rotors_lpf
        delta_flaps_lpf = alpha_lpf * delta_flaps + (1 - alpha_lpf) * delta_flaps_lpf
        delta_flaps_slow = alpha_slow * delta_flaps_lpf + (1 - alpha_slow) * delta_flaps_slow
        delta_flaps_hpf = delta_flaps_lpf - delta_flaps_slow

        # Eq. (46) 的 f_lpf 必须使用“不含瞬态”的舵面分量；角加速度 INDI
        # 则使用普通低通舵面分量。二者均由滤波后的执行器测量重新计算，
        # 而不是直接低通仿真环境给出的真实合力/力矩。
        f_alpha_lpf, _, _ = aero.compute_forces_and_moments(
            v_b,
            omega_rotors_lpf[0],
            omega_rotors_lpf[1],
            delta_flaps_slow[0],
            delta_flaps_slow[1],
        )
        _, m_lpf, _ = aero.compute_forces_and_moments(
            v_b,
            omega_rotors_lpf[0],
            omega_rotors_lpf[1],
            delta_flaps_lpf[0],
            delta_flaps_lpf[1],
        )
        _, _, f_delta_hpf = aero.compute_forces_and_moments(
            v_b,
            omega_rotors_lpf[0],
            omega_rotors_lpf[1],
            delta_flaps_hpf[0],
            delta_flaps_hpf[1],
        )

        # (c) 瞬态襟翼加速度修正 (论文 Eq. 43):
        #     a_tilde_lpf = a_lpf - m^-1 * R_alpha^i * f_delta_hpf^alpha
        R_alpha_to_i = R_b_to_i @ R_alpha_to_b
        a_lpf_tilde = a_lpf - (1.0 / cfg.mass) * (R_alpha_to_i @ f_delta_hpf)

        # (d) 其余 INDI 反馈信号
        omega_dot_lpf = alpha_lpf * state_dot[10:13] + (1 - alpha_lpf) * omega_dot_lpf
        
        # 记录日志
        log_t.append(t)
        log_pos.append(pos)
        log_vel.append(vel)
        log_x_ref.append(x_ref)
        log_v_ref.append(v_ref)
        log_a_ref.append(a_ref)
        log_psi_ref.append(psi_ref)
        log_acceleration.append(a_real_i.copy())
        log_euler.append(np.rad2deg(zxy_euler_from_quaternion(q_curr)))
        log_euler_command.append(np.rad2deg(zxy_euler_from_quaternion(q_c)))
        log_angular_velocity.append(omega_curr.copy())
        log_angular_velocity_ref.append(omega_ref.copy())
        log_angular_acceleration.append(state_dot[10:13].copy())
        log_angular_acceleration_lpf.append(omega_dot_lpf.copy())
        log_angular_acceleration_command.append(omega_dot_c.copy())
        log_rotors.append(omega_rotors)
        log_flaps.append(np.rad2deg(delta_flaps))

    print("仿真结束，正在绘图...")
    
    # === 绘图 ===
    log_t = np.array(log_t)
    log_pos = np.array(log_pos)
    log_vel = np.array(log_vel)
    log_x_ref = np.array(log_x_ref)
    log_v_ref = np.array(log_v_ref)
    log_a_ref = np.array(log_a_ref)
    log_psi_ref = np.array(log_psi_ref)
    log_acceleration = np.array(log_acceleration)
    log_euler = np.array(log_euler)
    log_euler_command = np.array(log_euler_command)
    log_angular_velocity = np.array(log_angular_velocity)
    log_angular_velocity_ref = np.array(log_angular_velocity_ref)
    log_angular_acceleration = np.array(log_angular_acceleration)
    log_angular_acceleration_lpf = np.array(log_angular_acceleration_lpf)
    log_angular_acceleration_command = np.array(log_angular_acceleration_command)
    log_rotors = np.array(log_rotors)
    log_flaps = np.array(log_flaps)

    # 保存独立动画程序所需的时序日志。欧拉角顺序固定为 [psi, phi, theta]，
    # 单位为度；位置采用仿真使用的世界 NED 坐标系。
    np.savez_compressed(
        log_path,
        time=log_t,
        position=log_pos,
        velocity=log_vel,
        acceleration=log_acceleration,
        position_ref=log_x_ref,
        velocity_ref=log_v_ref,
        acceleration_ref=log_a_ref,
        psi_ref=log_psi_ref,
        euler_deg=log_euler,
        euler_command_deg=log_euler_command,
        angular_velocity=log_angular_velocity,
        angular_velocity_ref=log_angular_velocity_ref,
        angular_acceleration=log_angular_acceleration,
        angular_acceleration_lpf=log_angular_acceleration_lpf,
        angular_acceleration_command=log_angular_acceleration_command,
        rotor_speed=log_rotors,
        flap_deg=log_flaps,
        coordinate_frame=np.array("NED"),
        euler_sequence=np.array("ZXY"),
    )
    print(f"仿真日志已保存到 {log_path}")

    # 位置跟踪误差统计 (RMS / 最大)
    pos_err_norm = np.linalg.norm(log_x_ref - log_pos, axis=1)
    print(f"位置跟踪误差:  RMS = {np.sqrt(np.mean(pos_err_norm**2)):.3f} m,  最大 = {pos_err_norm.max():.3f} m")

    speed_actual = np.linalg.norm(log_vel, axis=1)
    speed_reference = np.linalg.norm(log_v_ref, axis=1)
    yaw_actual = np.deg2rad(log_euler[:, 0])
    yaw_error = (yaw_actual - log_psi_ref + np.pi) % (2.0 * np.pi) - np.pi
    differential_thrust = cfg.c_T * (
        log_rotors[:, 0] ** 2 - log_rotors[:, 1] ** 2
    )
    reference_specific_force = np.linalg.norm(
        log_a_ref - np.array([0.0, 0.0, cfg.g]), axis=1
    ) / cfg.g
    
    plt.figure(figsize=(12, 10))
    
    plt.subplot(3, 1, 1)
    plt.plot(log_t, log_pos[:, 0], label='X (Forward)')
    plt.plot(log_t, log_pos[:, 1], label='Y (Right)')
    plt.plot(log_t, log_pos[:, 2], label='Z (Down)')
    plt.axhline(-10, color='r', linestyle='--', alpha=0.5, label='Hover Alt (-10m)')
    plt.ylabel('Position (m)')
    plt.legend()
    plt.grid()
    plt.title(f'Tailsitter Simulation: {trajectory_name}')
    
    plt.subplot(3, 1, 2)
    plt.plot(log_t, log_euler[:, 0], label='Yaw (deg)')
    plt.plot(log_t, log_euler[:, 1], label='Roll (deg)')
    plt.plot(log_t, log_euler[:, 2], label='Pitch (deg)')
    plt.ylabel('Euler Angles (deg)')
    plt.legend()
    plt.grid()
    
    plt.subplot(3, 1, 3)
    plt.plot(log_t, log_rotors[:, 0], label='Rotor 1 (rad/s)')
    plt.plot(log_t, log_rotors[:, 1], label='Rotor 2 (rad/s)')
    plt.plot(log_t, log_flaps[:, 0]*10, label='Flap 1 (deg x10)')
    plt.plot(log_t, log_flaps[:, 1]*10, label='Flap 2 (deg x10)')
    plt.ylabel('Actuator Commands')
    plt.xlabel('Time (s)')
    plt.legend()
    plt.grid()
    
    plt.tight_layout()
    if show_plot:
        plt.show()
    else:
        plt.close()

    return {
        "trajectory": trajectory_name,
        "rms_position_error": float(np.sqrt(np.mean(pos_err_norm**2))),
        "max_position_error": float(pos_err_norm.max()),
        "rms_yaw_error_deg": float(
            np.rad2deg(np.sqrt(np.mean(yaw_error**2)))
        ),
        "min_actual_speed": float(speed_actual.min()),
        "max_actual_speed": float(speed_actual.max()),
        "min_reference_speed": float(speed_reference.min()),
        "max_reference_speed": float(speed_reference.max()),
        "max_reference_specific_force_g": float(reference_specific_force.max()),
        "max_rotor_speed": float(np.max(log_rotors)),
        "max_abs_flap_deg": float(np.max(np.abs(log_flaps))),
        "max_abs_differential_thrust": float(
            np.max(np.abs(differential_thrust))
        ),
        "finite": bool(
            np.all(np.isfinite(state))
            and np.all(np.isfinite(log_pos))
            and np.all(np.isfinite(log_euler))
        ),
        "duration": float(t_end),
    }

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Tailsitter 全系统闭环仿真")
    parser.add_argument(
        "--trajectory",
        choices=(
            "segments",
            "lemniscate",
            "knife-edge-transition",
            "circle-coordinated",
            "circle-knife-edge",
            "circular-knife-edge",
            "transition-to-forward",
            "transition-to-hover",
            "differential-turn",
        ),
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
    args = parser.parse_args()
    main(
        args.trajectory,
        args.laps,
        args.speed,
        args.lap_time,
        args.log,
        not args.no_show,
    )
