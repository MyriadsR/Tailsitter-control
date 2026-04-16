'''
Author: xzr && zhenrxu@qq.com
Date: 2026-04-08 10:37:39
LastEditors: xzr && zhenrxu@qq.com
LastEditTime: 2026-04-15 11:03:37
FilePath: /Tailsitter_ws/controller/pos_ctrl.py
Description: 位置控制器实现

Copyright (c) 2026 by zhenrxu@qq.com, All Rights Reserved. 
'''
# controller/position_ctrl.py
import numpy as np

class PositionController:
    def __init__(self, config):
        """
        初始化位置控制器
        :param config: 包含控制器增益的配置对象
        """
        self.cfg = config
        
        # 增益矩阵 (对角阵)，在机体坐标系下定义
        # 对应论文中的 K_x, K_v, K_a
        self.K_x = np.diag([self.cfg.kx_x, self.cfg.kx_y, self.cfg.kx_z])
        self.K_v = np.diag([self.cfg.kv_x, self.cfg.kv_y, self.cfg.kv_z])
        self.K_a = np.diag([self.cfg.ka_x, self.cfg.ka_y, self.cfg.ka_z])

    def compute_acceleration_command(self, x_ref, v_ref, a_ref, x, v, a_lpf_tilde, R_b_to_i):
        """
        计算期望线性加速度 a_c (论文公式 41)
        
        :param x_ref: 期望位置 (世界系 NED) [3,]
        :param v_ref: 期望速度 (世界系 NED) [3,]
        :param a_ref: 期望加速度 (世界系 NED) [3,]
        :param x: 当前位置 (世界系 NED) [3,]
        :param v: 当前速度 (世界系 NED) [3,]
        :param a_lpf_tilde: 去除瞬态襟翼效应的滤波后实际加速度 (世界系) [3,]
        :param R_b_to_i: 从机体系到世界系的旋转矩阵 (3x3), 即 R_b^i
        :return: a_c 期望线性加速度 (世界系) [3,]
        """
        # 世界系到机体系的旋转矩阵 R_i^b (正交矩阵的转置即为逆矩阵)
        R_i_to_b = R_b_to_i.T
        
        # 1. 计算世界系下的误差
        e_x = x_ref - x
        e_v = v_ref - v
        e_a = a_ref - a_lpf_tilde
        
        # 2. 将误差转换到机体坐标系下，并乘以对应的机体系增益矩阵
        term_x = self.K_x @ (R_i_to_b @ e_x)
        term_v = self.K_v @ (R_i_to_b @ e_v)
        term_a = self.K_a @ (R_i_to_b @ e_a)
        
        # 3. 将机体系下的反馈控制量求和，并转换回世界坐标系
        feedback_acc = R_b_to_i @ (term_x + term_v + term_a)
        
        # 4. 加上前馈加速度 a_ref
        a_c = feedback_acc + a_ref
        
        return a_c