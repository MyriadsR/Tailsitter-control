'''
Author: xzr && zhenrxu@qq.com
Date: 2026-03-19 19:57:31
LastEditors: xzr && zhenrxu@qq.com
LastEditTime: 2026-03-19 20:30:38
FilePath: /Tailsitter_ws/environment/dynamics.py
Description: 动力学建模

Copyright (c) 2026 by zhenrxu@qq.com, All Rights Reserved. 
'''

import numpy as np

class UAVDynamics:
    def __init__(self, mass, inertia_matrix, gravity=9.81):
        """
        初始化物理模型的固定参数
        :param mass: 飞行器质量 (kg)
        :param inertia_matrix: 3x3 惯性张量矩阵 J (kg*m^2)
        :param gravity: 重力加速度 (m/s^2)
        """
        self.m = mass
        self.J = np.array(inertia_matrix, dtype=float)
        # 预先计算惯性张量的逆矩阵，方程(4)中要用到
        self.J_inv = np.linalg.inv(self.J) 
        self.g = gravity
        
        # 假设我们使用航空界标准的 NED（北-东-地）坐标系
        # 重力方向沿 Z 轴正方向向下
        self.i_z = np.array([0.0, 0.0, 1.0])

    def _quat_to_rot_matrix(self, quat):
        """
        工具函数：将四元数 [qw, qx, qy, qz] 转换为从机体坐标系到世界坐标系的旋转矩阵 R_i_b
        """
        qw, qx, qy, qz = quat
        # 归一化四元数以防止数值积分误差累积
        norm = np.sqrt(qw**2 + qx**2 + qy**2 + qz**2)
        qw, qx, qy, qz = qw/norm, qx/norm, qy/norm, qz/norm
        
        R = np.array([
            [1 - 2*qy**2 - 2*qz**2,     2*qx*qy - 2*qz*qw,     2*qx*qz + 2*qy*qw],
            [    2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2,     2*qy*qz - 2*qx*qw],
            [    2*qx*qz - 2*qy*qw,     2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
        ])
        return R

    def _quat_derivative(self, quat, omega):
        """
        实现公式 (3)：四元数运动学导数 (1/2 * xi \circ Omega)
        """
        qw, qx, qy, qz = quat
        p, q, r = omega
        
        # 将四元数乘法写成矩阵相乘形式，这是计算公式(3)最高效的方法
        omega_mat = np.array([
            [ 0, -p, -q, -r],
            [ p,  0,  r, -q],
            [ q, -r,  0,  p],
            [ r,  q, -p,  0]
        ])
        
        q_dot = 0.5 * omega_mat @ np.array([qw, qx, qy, qz])
        return q_dot

    def compute_state_derivatives(self, t, state, f_alpha, f_ext, moments, m_ext, R_b_alpha):
        """
        计算 13 个状态变量在当前时刻的微分向量 (x_dot)
        这个函数将被直接丢进 scipy.integrate.solve_ivp 中进行步进求解
        
        :param t: 当前仿真时间 (ODE 求解器需要)
        :param state: 13维状态向量 [x,y,z, vx,vy,vz, qw,qx,qy,qz, p,q,r]
        :param f_alpha: 论文中的 f^alpha，零升力坐标系下的气动+推力合力
        :param f_ext: 论文中的 f_ext，世界坐标系下的未建模外力 (如阵风)
        :param moments: 论文中的 m，机体坐标系下的气动+推力合力矩
        :param m_ext: 论文中的 m_ext，机体坐标系下的外力矩
        :param R_b_alpha: 从零升力坐标系(alpha)到机体坐标系(b)的固定转换矩阵
        """
        # --- 1. 拆解当前状态 ---
        pos   = state[0:3]
        vel   = state[3:6]
        quat  = state[6:10]
        omega = state[10:13]

        # --- 2. 坐标系转换 ---
        # R_i_b: 从机体坐标系(b)到世界坐标系(i)的旋转矩阵
        R_i_b = self._quat_to_rot_matrix(quat)
        # R_i_alpha: 从零升力坐标系(alpha)到世界坐标系(i)的旋转矩阵 (对应公式2中的 R_alpha^i)
        # 链式法则：R_i_alpha = R_i_b * R_b_alpha
        R_i_alpha = R_i_b @ R_b_alpha 

        # --- 3. 核心公式实现 ---
        
        # 公式 (1)：位置的导数就是速度
        pos_dot = vel
        
        # 公式 (2)：速度的导数（加速度）
        # v_dot = g*i_z + (1/m) * (R_alpha^i @ f_alpha + f_ext)
        v_dot = self.g * self.i_z + (1.0 / self.m) * (R_i_alpha @ f_alpha + f_ext)
        
        # 公式 (3)：姿态四元数的导数
        quat_dot = self._quat_derivative(quat, omega)
        
        # 公式 (4)：角速度的导数 (欧拉方程)
        # omega_dot = J^-1 @ (m + m_ext - omega x (J @ omega))
        m_total = moments + m_ext
        gyro_effect = np.cross(omega, self.J @ omega) # 陀螺力矩项
        omega_dot = self.J_inv @ (m_total - gyro_effect)

        # --- 4. 组装并返回状态微分向量 ---
        state_dot = np.concatenate([pos_dot, v_dot, quat_dot, omega_dot])
        return state_dot