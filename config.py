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
        self.rho = 1.225  # 论文参数辨识使用的标准海平面空气密度 (kg/m^3)
        
        # === 几何参数 ===
        self.alpha_0 = np.deg2rad(-2.0)  # 零升力迎角 (论文未公布，按弯度翼型估算)
        self.alpha_T = np.deg2rad(-5.0)  # 推力安装角，论文 Table 1
        
        # 几何臂长 (m)
        self.l_Ty = 0.15     # 电机到对称面的横向距离，按 0.55 m 翼展和照片比例估算
        self.l_dy = 0.12     # 襟翼气动中心的横向距离，按照片和梯形翼面积中心估算
        self.l_dx = 0.075    # 气动中心到横轴(Y轴)的距离 [cite: 688]
        
        # === 推力与电机参数 ===
        # c_T 由论文悬停编码器频率、Fig. 11 的 6.1 N 差分推力和
        # 博士论文的最大转速交叉反算；作者未公布台架测量原值。
        self.c_T = 1.8e-6    # T_i = c_T * omega_i^2, N/(rad/s)^2
        self.c_mu = 2.2e-8   # Q_i = c_mu * omega_i^2, N*m/(rad/s)^2
        self.c_mu_T = -0.025 # 论文基于悬停配平辨识的推力俯仰力矩系数 (m)

        # === 执行器范围与动态 ===
        self.omega_min = 0.0
        self.omega_max = 2500.0          # 博士论文中接近最大值的转速 (rad/s)
        self.motor_time_constant = 0.04  # 依据 Fig. 11 响应速度估算 (s)
        self.delta_max = 1.0             # Fig. 11 中接近满舵的约 +/-1 rad
        self.servo_time_constant = 0.03  # MKS HV93i 响应量级 (s)
        self.servo_rate_max = 25.0       # 考虑气动负载后的舵面速率限制 (rad/s)
        
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

        # === 惯性张量 (几何/质量分布估算值, 单位: kg*m^2) ===
        # 论文定义 b_x 为弦向、b_y 为展向，因此飞翼应有 J_xx,J_zz > J_yy。
        self.inertia_matrix = [
            [0.0095, 0.0,    0.0],
            [0.0,    0.0030, 0.0],
            [0.0,    0.0,    0.0115]
        ]

        # === 内环姿态 PD 控制器增益 ===
        # 滚转轴 (Roll - 靠副翼/差动舵面)
        self.k_xi_roll = 40.0
        self.k_omega_roll = 10.0
        
        # 俯仰轴 (Pitch - 靠升降舵，受空速影响，响应略慢)
        self.k_xi_pitch = 28.0
        self.k_omega_pitch = 7.0
        
        # 偏航轴 (Yaw - 靠差速推力，响应极快)
        self.k_xi_yaw = 28.0
        self.k_omega_yaw = 7.0
