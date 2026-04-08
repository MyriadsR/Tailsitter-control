'''
Author: xzr && zhenrxu@qq.com
Date: 2026-03-20 10:08:24
LastEditors: xzr && zhenrxu@qq.com
LastEditTime: 2026-03-20 10:35:35
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