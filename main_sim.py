'''
Author: You & Gemini
Description: Tailsitter 飞翼无人机全系统闭环仿真主程序
'''
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

# 导入数学工具
from utils.math_help import zxy_euler_to_quaternion


def zxy_euler_from_quaternion(q):
    """工具函数：将四元数转换为 ZXY 欧拉角 [psi, phi, theta]。"""
    qw, qx, qy, qz = q
    # ZXY 提取（对应关系：phi=Roll, theta=Pitch, psi=Yaw）
    phi = np.arcsin(2.0 * (qw * qx + qy * qz))
    theta = np.arctan2(2.0 * (qw * qy - qx * qz), 1.0 - 2.0 * (qx**2 + qy**2))
    psi = np.arctan2(2.0 * (qw * qz - qx * qy), 1.0 - 2.0 * (qx**2 + qz**2))
    return np.array([psi, phi, theta])


def quintic_segment(distance, tau, duration):
    """五次多项式轨迹段，返回位置/速度/加速度/Jerk。"""
    s = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5
    s_dot = (30.0 * tau**2 - 60.0 * tau**3 + 30.0 * tau**4) / duration
    s_ddot = (60.0 * tau - 180.0 * tau**2 + 120.0 * tau**3) / (duration**2)
    s_dddot = (60.0 - 360.0 * tau + 360.0 * tau**2) / (duration**3)
    return distance * s, distance * s_dot, distance * s_ddot, distance * s_dddot

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


def main():
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
    
    # === 仿真参数 ===
    dt = 0.01  # 100Hz 控制频率
    takeoff_duration = 6.0
    hover_1_duration = 5.0
    forward_duration = 6.0
    hover_2_duration = 5.0
    right_duration = 6.0
    hover_3_duration = 5.0
    t_end = (
        takeoff_duration
        + hover_1_duration
        + forward_duration
        + hover_2_duration
        + right_duration
        + hover_3_duration
    )
    steps = int(t_end / dt) + 1
    
    # === 初始状态 ===
    # NED 坐标系下地面起飞，地面高度设为 Z = 0
    pos_0 = np.array([0.0, 0.0, 0.0])
    vel_0 = np.array([0.0, 0.0, 0.0])
    
    # Tailsitter 悬停时机头朝上，推力抵消重力，即机体 X 轴指向世界 -Z
    # 对应 ZXY 欧拉角的 Pitch = 90 度
    q_0 = zxy_euler_to_quaternion(0.0, 0.0, np.pi/2)
    omega_0 = np.array([0.0, 0.0, 0.0])
    
    state = np.concatenate([pos_0, vel_0, q_0, omega_0])
    
    # === INDI 滤波器初始状态 (低通滤波后的历史值) ===
    a_lpf_tilde = np.array([0.0, 0.0, cfg.g])  # 世界系下的线加速度
    omega_dot_lpf = np.array([0.0, 0.0, 0.0])
    f_alpha_lpf = np.array([0.0, 0.0, 0.0]) 
    m_lpf = np.array([0.0, 0.0, 0.0])
    
    # 初始执行器状态
    # hover_thrust_per_motor = (cfg.mass * cfg.g) / 2.0
    omega_rotors = np.array([0.0, 0.0])
    delta_flaps = np.array([0.0, 0.0])
    
    # 记录数据用的列表
    log_t, log_pos, log_euler, log_rotors, log_flaps = [], [], [], [], []
    
    # 预计算常数旋转矩阵（角度符号由 alpha_0 自身决定）
    # 注意这里 alpha_0 自带正负号
    ca0, sa0 = np.cos(cfg.alpha_0), np.sin(cfg.alpha_0)
    R_b_to_alpha = np.array([[ca0, 0, sa0], [0, 1, 0], [-sa0, 0, ca0]])
    R_alpha_to_b = R_b_to_alpha.T
    # gravity_comp_i = np.array([0.0, 0.0, -cfg.g])  # NED 下抵消重力需沿 -Z 方向补偿

    print("开始仿真运行...")
    for step in range(steps):
        t = step * dt
        
        # 1. 解析当前状态
        pos = state[0:3]
        vel = state[3:6]
        q_curr = state[6:10]
        omega_curr = state[10:13]
        R_i_to_b = dyn._quat_to_rot_matrix(q_curr).T
        R_b_to_i = R_i_to_b.T
        
        # 2. 任务轨迹：垂直起飞到 10m -> 悬停 5s -> 前飞 10m -> 悬停 5s -> 向右飞 10m -> 悬停 5s
        t_takeoff_end = takeoff_duration
        t_hover1_end = t_takeoff_end + hover_1_duration
        t_forward_end = t_hover1_end + forward_duration
        t_hover2_end = t_forward_end + hover_2_duration
        t_right_end = t_hover2_end + right_duration

        if t <= takeoff_duration:
            tau = t / takeoff_duration
            z_pos, z_vel, z_acc, z_jerk = quintic_segment(-10.0, tau, takeoff_duration)
            x_ref = np.array([0.0, 0.0, z_pos])
            v_ref = np.array([0.0, 0.0, z_vel])
            a_ref = np.array([0.0, 0.0, z_acc])
            j_ref = np.array([0.0, 0.0, z_jerk])
        elif t <= t_hover1_end:
            x_ref = np.array([0.0, 0.0, -10.0])
            v_ref = np.array([0.0, 0.0, 0.0])
            a_ref = np.array([0.0, 0.0, 0.0])
            j_ref = np.array([0.0, 0.0, 0.0])
        elif t <= t_forward_end:
            tau = (t - t_hover1_end) / forward_duration
            x_pos, x_vel, x_acc, x_jerk = quintic_segment(10.0, tau, forward_duration)
            x_ref = np.array([x_pos, 0.0, -10.0])
            v_ref = np.array([x_vel, 0.0, 0.0])
            a_ref = np.array([x_acc, 0.0, 0.0])
            j_ref = np.array([x_jerk, 0.0, 0.0])
        elif t <= t_hover2_end:
            x_ref = np.array([10.0, 0.0, -10.0])
            v_ref = np.array([0.0, 0.0, 0.0])
            a_ref = np.array([0.0, 0.0, 0.0])
            j_ref = np.array([0.0, 0.0, 0.0])
        elif t <= t_right_end:
            tau = (t - t_hover2_end) / right_duration
            y_pos, y_vel, y_acc, y_jerk = quintic_segment(10.0, tau, right_duration)
            x_ref = np.array([10.0, y_pos, -10.0])
            v_ref = np.array([0.0, y_vel, 0.0])
            a_ref = np.array([0.0, y_acc, 0.0])
            j_ref = np.array([0.0, y_jerk, 0.0])
        else:
            x_ref = np.array([10.0, 10.0, -10.0])
            v_ref = np.array([0.0, 0.0, 0.0])
            a_ref = np.array([0.0, 0.0, 0.0])
            j_ref = np.array([0.0, 0.0, 0.0])
            
        psi_ref = 0.0
        psi_dot_ref = 0.0
        
        # 3. 外环控制 (位置 -> 期望加速度 -> INDI 期望力)
        a_c = pos_ctrl.compute_acceleration_command(x_ref, v_ref, a_ref, pos, vel, a_lpf_tilde, R_b_to_i)
        # a_c = a_c + gravity_comp_i  # 加上重力补偿
        f_c_i = indi_lin.compute_force_command(a_c, a_lpf_tilde, f_alpha_lpf, R_b_to_i, R_alpha_to_b)
        
        # 4. 微分平坦变换 (受力 -> 姿态与推力)
        delta_sum = delta_flaps[0] + delta_flaps[1]
        q_c, T_c = att_flat.compute_desired_attitude_and_thrust(f_c_i, vel, delta_sum, psi_ref, q_curr)
        
        # 求解平坦前馈角速度（从 q_c 提取 ZXY 欧拉角 [psi, phi, theta]）
        _, phi, theta_c = zxy_euler_from_quaternion(q_c)
        theta_bar = theta_c - cfg.alpha_0
        omega_ref = att_flat.compute_feedforward_angular_velocity(
            v_ref, a_ref, j_ref, psi_ref, psi_dot_ref, f_c_i, phi, theta_bar, delta_sum
        )
        
        # 5. 内环控制 (姿态 -> 期望角加速度 -> INDI 期望力矩)
        omega_dot_c = att_pd.compute_angular_acceleration_command(q_curr, q_c, omega_curr, omega_ref)
        m_c = indi_ang.compute_moment_command(omega_dot_c, omega_dot_lpf, m_lpf)
        
        # 6. 控制分配 (推力、力矩 -> 电机转速、舵偏)
        v_b = R_i_to_b @ vel
        omega_cmd, delta_cmd = allocator.allocate_controls(T_c, m_c, v_b)
        
        # 模拟执行器延迟 (一阶低通滤波)
        alpha_act = 0.8
        omega_rotors = alpha_act * omega_cmd + (1 - alpha_act) * omega_rotors
        delta_flaps = alpha_act * delta_cmd + (1 - alpha_act) * delta_flaps
        
        # 7. 物理环境计算真实的受力与力矩
        f_alpha_real, m_body_real = aero.compute_forces_and_moments(
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
        # 世界系下的实际加速度 (不扣除重力，直接使用 v_dot)
        a_real_i = state_dot[3:6]
        alpha_lpf = 0.35  # 低通滤波系数
        a_lpf_tilde = alpha_lpf * a_real_i + (1 - alpha_lpf) * a_lpf_tilde
        omega_dot_lpf = alpha_lpf * state_dot[10:13] + (1 - alpha_lpf) * omega_dot_lpf
        f_alpha_lpf = alpha_lpf * f_alpha_real + (1 - alpha_lpf) * f_alpha_lpf
        m_lpf = alpha_lpf * m_body_real + (1 - alpha_lpf) * m_lpf
        
        # 记录日志
        log_t.append(t)
        log_pos.append(pos)
        log_euler.append(np.rad2deg(zxy_euler_from_quaternion(q_curr)))
        log_rotors.append(omega_rotors)
        log_flaps.append(np.rad2deg(delta_flaps))

    print("仿真结束，正在绘图...")
    
    # === 绘图 ===
    log_t = np.array(log_t)
    log_pos = np.array(log_pos)
    log_euler = np.array(log_euler)
    log_rotors = np.array(log_rotors)
    log_flaps = np.array(log_flaps)
    
    plt.figure(figsize=(12, 10))
    
    plt.subplot(3, 1, 1)
    plt.plot(log_t, log_pos[:, 0], label='X (Forward)')
    plt.plot(log_t, log_pos[:, 1], label='Y (Right)')
    plt.plot(log_t, log_pos[:, 2], label='Z (Down)')
    plt.axhline(-10, color='r', linestyle='--', alpha=0.5, label='Hover Alt (-10m)')
    plt.ylabel('Position (m)')
    plt.legend()
    plt.grid()
    plt.title('Tailsitter Trajectory & Attitude Simulation')
    
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
    plt.show()

if __name__ == "__main__":
    main()