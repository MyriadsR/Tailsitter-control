'''
Author: xzr && zhenrxu@qq.com
Date: 2026-04-16 20:10:43
LastEditors: xzr && zhenrxu@qq.com
LastEditTime: 2026-04-16 20:10:51
FilePath: /Tailsitter_ws/utils/math_help.py
Description: 数学工具函数

Copyright (c) 2026 by zhenrxu@qq.com, All Rights Reserved. 
'''
# utils/math_utils.py
import numpy as np

def zxy_euler_to_quaternion(psi, phi, theta):
    """
    将 ZXY 顺序的欧拉角转换为四元数 [qw, qx, qy, qz]
    对应的物理意义：先绕 Z(Yaw) 旋转 psi，再绕 X(Roll) 旋转 phi，最后绕 Y(Pitch) 旋转 theta。
    """
    c_psi = np.cos(psi / 2.0)
    s_psi = np.sin(psi / 2.0)
    c_phi = np.cos(phi / 2.0)
    s_phi = np.sin(phi / 2.0)
    c_th  = np.cos(theta / 2.0)
    s_th  = np.sin(theta / 2.0)
    
    # 根据 ZXY 旋转顺序推导出的四元数乘法结果
    qw = c_psi * c_phi * c_th - s_psi * s_phi * s_th
    qx = c_psi * s_phi * c_th - s_psi * c_phi * s_th
    qy = c_psi * c_phi * s_th + s_psi * s_phi * c_th
    qz = s_psi * c_phi * c_th + c_psi * s_phi * s_th
    
    # 归一化以防精度误差
    norm = np.sqrt(qw**2 + qx**2 + qy**2 + qz**2)
    return np.array([qw/norm, qx/norm, qy/norm, qz/norm])