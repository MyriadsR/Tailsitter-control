'''
Author: xzr && zhenrxu@qq.com
Date: 2026-04-15 11:16:50
LastEditors: xzr && zhenrxu@qq.com
LastEditTime: 2026-04-15 11:30:09
FilePath: /Tailsitter_ws/controller/INDI_acc_ctrl.py
Description: 加速度内环控制器实现

Copyright (c) 2026 by zhenrxu@qq.com, All Rights Reserved. 
'''
# controller/indi_ctrl.py
import numpy as np

class INDILinearController:
    def __init__(self, config):
        """
        初始化 INDI 线性加速度控制器
        :param config: 包含飞行器质量等参数的配置对象
        """
        self.mass = config.mass

    def compute_force_command(self, a_c, a_lpf_tilde, f_alpha_lpf, R_b_to_i, R_alpha_to_b):
        """
        计算期望的力向量 f_c^i (对应论文公式 46)
        
        :param a_c: 外环传来的期望线性加速度 (世界系) [3,]
        :param a_lpf_tilde: 去除瞬态襟翼效应的滤波后实际加速度 (世界系) [3,]
        :param f_alpha_lpf: 基于滤波后的传感器数据(转速、舵偏)算出的当前气动力 (零升力系 alpha) [3,]
        :param R_b_to_i: 从机体系到世界系的旋转矩阵 (3x3), 即 R_b^i
        :param R_alpha_to_b: 从零升力系到机体系的旋转矩阵 (3x3), 即 R_alpha^b
        :return: f_c_i 期望力向量 (世界系) [3,]
        """
        
        # 1. 坐标系链式旋转：计算从零升力系 (alpha) 到世界系 (i) 的旋转矩阵 R_alpha^i
        R_alpha_to_i = R_b_to_i @ R_alpha_to_b
        
        # 2. 将当前滤波后的气动与推力合力，从零升力系转换到世界坐标系
        # 这就是公式 (46) 中的 f^i_lpf
        f_i_lpf = R_alpha_to_i @ f_alpha_lpf
        
        # 3. 核心 INDI 控制律 (公式 46)
        # f_c^i = m * (a_c - \tilde{a}_{lpf}) + f_{lpf}^i
        f_c_i = self.mass * (a_c - a_lpf_tilde) + f_i_lpf
        
        return f_c_i