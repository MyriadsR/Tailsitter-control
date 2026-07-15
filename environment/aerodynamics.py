'''
Author: xzr && zhenrxu@qq.com
Date: 2026-03-19 19:58:31
LastEditors: xzr && zhenrxu@qq.com
LastEditTime: 2026-03-20 12:43:40
FilePath: /Tailsitter_ws/environment/aerodynamics.py
Description: 气动力模型

Copyright (c) 2026 by zhenrxu@qq.com, All Rights Reserved. 
'''

# environment/aerodynamics.py
import numpy as np

class Aerodynamics:
    def __init__(self, config):
        self.cfg = config
        # 预计算 alpha_tilde = alpha_0 + alpha_T
        self.alpha_tilde = self.cfg.alpha_0 + self.cfg.alpha_T
        
    def compute_forces_and_moments(self, v_b, omega_1, omega_2, delta_1, delta_2):
        """
        计算空气动力和推力、以及力矩。
        :param v_b: 机体坐标系下的速度向量 [vx, vy, vz] (m/s)
        :param omega_1, omega_2: 左右电机转速 (rad/s)
        :param delta_1, delta_2: 左右襟翼偏角 (rad)
        :return: f_alpha (零升力系3D受力), m_body (机体系3D力矩), f_delta_alpha (零升力系襟翼力, 供 Eq 43 瞬态修正)
        """
        v_norm = np.linalg.norm(v_b)
        # 防止除以零
        if v_norm < 1e-3:
            v_norm = 1e-3
            
        # 1. 坐标系转换：计算零升力系下的速度 v_alpha
        # 绕 Y 轴旋转 alpha_0（角度符号由 alpha_0 自身决定）
        # 注意这里 alpha 自带正负号
        ca0 = np.cos(self.cfg.alpha_0)
        sa0 = np.sin(self.cfg.alpha_0)
        R_b_to_alpha = np.array([
            [ ca0, 0,  sa0],
            [   0, 1,    0],
            [-sa0, 0,  ca0]
        ])
        R_alpha_to_b = R_b_to_alpha.T

        v_alpha = R_b_to_alpha @ v_b
        vx_a, vy_a, vz_a = v_alpha[0], v_alpha[1], v_alpha[2]

        # 2. 计算推力幅值 (Eq 6)
        T1 = self.cfg.c_T * (omega_1 ** 2)
        T2 = self.cfg.c_T * (omega_2 ** 2)
        
        # ==========================================
        # 计算力 (在零升力坐标系 alpha 下)
        # ==========================================
        ca_t = np.cos(self.alpha_tilde)
        sa_t = np.sin(self.alpha_tilde)
        
        # 1. 推力在 alpha 系下的直接受力 (Eq 5)
        # 注意: 论文中公式(5) 是对两个电机求和
        f_T1_alpha = np.array([
            ca_t * (1 - self.cfg.c_DT),
            0.0,
            sa_t * (self.cfg.c_LT - 1)
        ]) * T1

        f_T2_alpha = np.array([
            ca_t * (1 - self.cfg.c_DT),
            0.0,
            sa_t * (self.cfg.c_LT - 1)
        ]) * T2
        
        f_T_alpha = f_T1_alpha + f_T2_alpha
        
        # 2. 襟翼带来的受力 (Eq 7)
        # 襟翼1
        f_delta1_z = -(self.cfg.c_LT_delta * ca_t * T1 + self.cfg.c_LV_delta * v_norm * vx_a) * delta_1
        f_delta1_alpha = np.array([0.0, 0.0, f_delta1_z])
        # 襟翼2
        f_delta2_z = -(self.cfg.c_LT_delta * ca_t * T2 + self.cfg.c_LV_delta * v_norm * vx_a) * delta_2
        f_delta2_alpha = np.array([0.0, 0.0, f_delta2_z])
        
        f_delta_alpha = f_delta1_alpha + f_delta2_alpha
        
        # 3. 机翼的空气动力 (Eq 8)
        f_w_alpha = -np.array([
            self.cfg.c_DV * vx_a,
            0.0,
            self.cfg.c_LV * vz_a
        ]) * v_norm
        
        # 4. 零升力系下的合力 (Eq 9)
        f_alpha = f_T_alpha + f_delta_alpha + f_w_alpha
        
        # ==========================================
        # 计算力矩 (在机体坐标系 b 下)
        # ==========================================
        
        # 1. 推力带来的力矩 (Eq 10)
        # 注意：此处包含了推力偏心引发的滚转、偏航，以及纵向的俯仰力矩
        i_x = np.array([1.0, 0.0, 0.0])
        i_z = np.array([0.0, 0.0, 1.0])

        m_T = np.array([
            self.cfg.l_Ty * (i_z @ R_alpha_to_b @ (f_T2_alpha - f_T1_alpha)),
            self.cfg.c_mu_T * (T1 + T2),
            self.cfg.l_Ty * (i_x @ R_alpha_to_b @ (f_T1_alpha - f_T2_alpha))
        ])
        
        # 2. 螺旋桨反扭矩引起的力矩 (Eq 11 & Eq 12)
        # 设电机 1 顺时针，电机 2 逆时针
        ca_T = np.cos(self.cfg.alpha_T)
        sa_T = np.sin(self.cfg.alpha_T)

        mu_1 = self.cfg.c_mu * (omega_1 ** 2)
        mu_2 = -self.cfg.c_mu * (omega_2 ** 2)
        mu_sum = mu_1 + mu_2
        
        m_mu = np.array([
             ca_T * mu_sum,
             0.0,
            -sa_T * mu_sum
        ])
        
        # 3. 襟翼受力引起的力矩 (Eq 13)
        # 注意: 公式(13)中的 f^z_delta1 是襟翼受力的Z分量
        ca_0 = np.cos(self.cfg.alpha_0)
        sa_0 = np.sin(self.cfg.alpha_0)
        
        m_delta = np.array([
            self.cfg.l_dy * ca_0 * (f_delta2_z - f_delta1_z),
            self.cfg.l_dx * (f_delta1_z + f_delta2_z),
            self.cfg.l_dy * sa_0 * (f_delta2_z - f_delta1_z)
        ])
        
        # 4. 机体坐标系下的合力矩 (Eq 14)
        m_body = m_T + m_mu + m_delta

        return f_alpha, m_body, f_delta_alpha