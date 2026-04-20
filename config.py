'''
Author: xzr && zhenrxu@qq.com
Date: 2026-03-20 10:08:24
LastEditors: xzr && zhenrxu@qq.com
LastEditTime: 2026-04-20 20:13:35
FilePath: /Tailsitter_ws/config.py
Description: 无人机配置参数

Copyright (c) 2026 by zhenrxu@qq.com, All Rights Reserved. 
'''
# config.py
import numpy as np

class TailsitterConfig:
    def __init__(self):
        # === 基础物理参数 ===
        self.mass = 0.7  # 飞行器质量 (kg) [cite: 798]
        self.g = 9.81
        
        # === 几何参数 ===
        self.alpha_0 = np.deg2rad(-2.0)  # 零升力迎角 (估算值, 稍微带有弯度的翼型)
        self.alpha_T = np.deg2rad(-5.0)  # 推力安装角 [cite: 693]
        
        # 几何臂长 (m)
        self.l_Ty = 0.15     # 电机到纵轴(X轴)的横向距离 (估算值)
        self.l_dy = 0.15     # 襟翼中心到纵轴(X轴)的横向距离 (估算值)
        self.l_dx = 0.075    # 气动中心到横轴(Y轴)的距离 [cite: 688]
        
        # === 推力与电机参数 ===
        self.c_T = 1.2e-5    # 螺旋桨推力系数 (T = c_T * omega^2) (常规5寸桨估算)
        self.c_mu = 1.5e-7   # 螺旋桨反扭矩系数 (常规估算)
        self.c_mu_T = -0.025 # 推力俯仰力矩系数 [cite: 786]
        
        # === alpha-theory 气动参数 (基于论文 Table 2 实验测量值) ===
        self.c_LV = 0.29     # 机翼升力系数 [cite: 670]
        self.c_DV = 0.0      # 机翼阻力系数 (室内低速近似为0) [cite: 670, 718]
        
        self.c_LT = 2.23     # 螺旋桨滑流诱导升力系数 [cite: 670]
        self.c_DT = 0.0      # 螺旋桨滑流诱导阻力系数 [cite: 670]
        
        self.c_LV_delta = 0.18 # 襟翼偏转带来的升力导数 [cite: 670]
        self.c_LT_delta = 1.25 # 滑流中襟翼偏转的升力导数 [cite: 670]

        # === 外环位置与速度控制器增益 (在机体坐标系下定义) ===
        # 机体 X 轴 (纵向, 推力轴) 的控制通常可以稍微激进一点
        self.kx_x = 4.0   
        self.kv_x = 3.0   
        self.ka_x = 0.0   # 加速度反馈增益 (初始可以设为0，主要靠前馈)

        # 机体 Y 轴 (侧向) 没有任何气动面，响应最弱，全靠滚转带动，增益宜小
        self.kx_y = 2.0   
        self.kv_y = 1.5   
        self.ka_y = 0.0   

        # 机体 Z 轴 (法向) 依赖机翼升力和推力矢量，响应居中
        self.kx_z = 3.0   
        self.kv_z = 2.0   
        self.ka_z = 0.0

        # === 惯性张量 (估算值, 单位: kg*m^2) ===
        # Tailsitter 滚转惯量较小，偏航和俯仰较大
        self.inertia_matrix = [
            [0.005, 0.0,   0.0],
            [0.0,   0.015, 0.0],
            [0.0,   0.0,   0.018]
        ]

        # === 内环姿态 PD 控制器增益 ===
        # 滚转轴 (Roll - 靠差速推力，响应极快)
        self.k_xi_roll = 60.0
        self.k_omega_roll = 15.0
        
        # 俯仰轴 (Pitch - 靠升降舵，受空速影响，响应略慢)
        self.k_xi_pitch = 40.0
        self.k_omega_pitch = 10.0
        
        # 偏航轴 (Yaw - 靠副翼/差动舵面)
        self.k_xi_yaw = 40.0
        self.k_omega_yaw = 10.0