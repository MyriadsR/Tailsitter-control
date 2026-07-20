# controller/actuator_ctrl.py
import numpy as np

class ControlAllocator:
    def __init__(self, config):
        """
        初始化控制分配器 (实现公式 37-40)
        """
        self.cfg = config
        self.alpha_tilde = self.cfg.alpha_0 + self.cfg.alpha_T
        
        # 预计算高频调用的三角函数常数
        self.c_a0 = np.cos(self.cfg.alpha_0)
        self.s_a0 = np.sin(self.cfg.alpha_0)
        self.c_at = np.cos(self.alpha_tilde)
        self.s_at = np.sin(self.alpha_tilde)
        self.c_aT = np.cos(self.cfg.alpha_T)
        self.s_aT = np.sin(self.cfg.alpha_T)

    @staticmethod
    def _bounded_flap_allocation(M, b_vec, max_deflection):
        """求解二维有界最小二乘，避免逐元素截断破坏滚转/俯仰折中。"""
        candidates = []

        unconstrained = np.linalg.pinv(M) @ b_vec
        if np.all(np.abs(unconstrained) <= max_deflection):
            candidates.append(unconstrained)

        # 最优解若不在内部，必然位于某条边界；固定一个舵面后解析求另一个。
        for fixed_index in (0, 1):
            free_index = 1 - fixed_index
            free_column = M[:, free_index]
            free_norm_sq = free_column @ free_column
            for fixed_value in (-max_deflection, max_deflection):
                candidate = np.zeros(2)
                candidate[fixed_index] = fixed_value
                residual_target = b_vec - M[:, fixed_index] * fixed_value
                if free_norm_sq > 1e-12:
                    free_value = (free_column @ residual_target) / free_norm_sq
                else:
                    free_value = 0.0
                candidate[free_index] = np.clip(
                    free_value, -max_deflection, max_deflection
                )
                candidates.append(candidate)

        if not candidates:
            return np.zeros(2)
        return min(candidates, key=lambda delta: np.linalg.norm(M @ delta - b_vec))

    def allocate_controls(self, T_c, m_c, v_b):
        """
        根据期望总推力 T_c 和机体力矩 m_c，分配电机转速和舵偏角
        
        :param T_c: INDI/平坦变换算出的期望总推力 (N)
        :param m_c: INDI 算出的期望机体力矩 (N*m) [3,]
        :param v_b: 当前机体坐标系下的速度向量 (m/s) [3,]
        :return: omega_cmd [2,], delta_cmd [2,]
        """
        v_norm = np.linalg.norm(v_b)
        
        # ==========================================
        # 第一步：计算差速推力 \Delta T (Eq 37)
        # ==========================================
        # 提取目标 Z 轴力矩 (偏航力矩)
        m_z = m_c[2]
        
        # 公式 (37) 分母计算 
        term1 = self.c_a0 * self.c_at * (1 - self.cfg.c_DT) - self.s_a0 * self.s_at * (self.cfg.c_LT - 1)
        denom_T = self.cfg.l_Ty * term1 - self.s_aT * (self.cfg.c_mu / self.cfg.c_T)
        
        # 保护除以零
        if abs(denom_T) < 1e-6:
            delta_T = 0.0
        else:
            delta_T = m_z / denom_T
            
        # ==========================================
        # 第二步：计算单电机推力与转速 (Eq 38, Eq 6)
        # ==========================================
        # 在保持总推力的前提下同时约束单电机的上、下限。
        # T1=(T_c+delta_T)/2, T2=(T_c-delta_T)/2，因此差分推力
        # 的可行区间会随总推力收缩。
        thrust_min = self.cfg.c_T * self.cfg.omega_min**2
        thrust_max = self.cfg.c_T * self.cfg.omega_max**2
        T_c = np.clip(float(T_c), 2.0 * thrust_min, 2.0 * thrust_max)
        delta_lower = max(2.0 * thrust_min - T_c, T_c - 2.0 * thrust_max)
        delta_upper = min(T_c - 2.0 * thrust_min, 2.0 * thrust_max - T_c)
        delta_T = np.clip(delta_T, delta_lower, delta_upper)
        T1 = (T_c + delta_T) / 2.0
        T2 = (T_c - delta_T) / 2.0
        realized_delta_T = T1 - T2
        
        # 反解期望转速 omega_c = sqrt(T / c_T) 
        omega_1_cmd = np.sqrt(T1 / self.cfg.c_T)
        omega_2_cmd = np.sqrt(T2 / self.cfg.c_T)
        
        # ==========================================
        # 第三步：剥离推力力矩，提取期望舵面力矩 m_delta (Eq 39 之前)
        # ==========================================
        # 1. 重新计算电机推力在 机体系(b) 下产生的力矩 m_T
        # （复用 aerodynamics.py 里的逻辑）
        # 注意求旋转矩阵时的角度正负号，因为 alpha_0 是负的！！！！！！！！！！！！！！
        f_Ti_alpha_x = self.c_at * (1 - self.cfg.c_DT)
        f_Ti_alpha_z = self.s_at * (self.cfg.c_LT - 1)
        f_Ti_b_x = self.c_a0 * f_Ti_alpha_x - self.s_a0 * f_Ti_alpha_z
        f_Ti_b_z = self.s_a0 * f_Ti_alpha_x + self.c_a0 * f_Ti_alpha_z
        
        m_T = np.array([
            self.cfg.l_Ty * f_Ti_b_z * (T2 - T1),
            self.cfg.c_mu_T * (T1 + T2),
            self.cfg.l_Ty * f_Ti_b_x * (T1 - T2)
        ])
        
        # 2. 计算电机反扭矩 m_mu
        mu_sum = (self.cfg.c_mu / self.cfg.c_T) * realized_delta_T
        m_mu = np.array([
             self.c_aT * mu_sum,
             0.0,
            -self.s_aT * mu_sum
        ])
        
        # 3. 剥离得到襟翼必须提供的力矩 
        m_delta_c = m_c - m_T - m_mu
        
        # ==========================================
        # 第四步：求解期望襟翼偏角 delta (Eq 39, 40)
        # ==========================================
        # 获得零升力坐标系下的 X 轴速度 v_x^\alpha
        # 注意求旋转矩阵时的角度正负号，因为 alpha_0 是负的
        v_alpha_x = self.c_a0 * v_b[0] + self.s_a0 * v_b[2]
        
        # 计算 \nu_1 和 \nu_2 (Eq 40) 
        nu_1 = -self.cfg.c_LT_delta * self.c_at * T1 - self.cfg.c_LV_delta * v_norm * v_alpha_x
        nu_2 = -self.cfg.c_LT_delta * self.c_at * T2 - self.cfg.c_LV_delta * v_norm * v_alpha_x
        
        # 构造分配矩阵 M (2x2) 
        M = np.array([
            [-self.cfg.l_dy * self.c_a0 * nu_1,  self.cfg.l_dy * self.c_a0 * nu_2],
            [ self.cfg.l_dx * nu_1,              self.cfg.l_dx * nu_2            ]
        ])
        
        # 取出期望舵面力矩的 X (滚转) 和 Y (俯仰) 分量
        b_vec = np.array([m_delta_c[0], m_delta_c[1]])
        
        # 物理舵偏角限幅，并在边界上重新优化剩余自由度。
        max_deflection = self.cfg.delta_max
        delta_cmd = self._bounded_flap_allocation(M, b_vec, max_deflection)
        delta_1_cmd, delta_2_cmd = delta_cmd
        
        return np.array([omega_1_cmd, omega_2_cmd]), np.array([delta_1_cmd, delta_2_cmd])


# class IntegrativeMotorController:
#     def __init__(self, config):
#         """
#         初始化带积分的底层电机转速控制器 (Eq 53)
#         """
#         self.k_I = 0.005 # 积分增益
#         self.integral_error = np.zeros(2)
        
#     def _feedforward_poly(self, omega_cmd):
#         """ 二次多项式映射: 转速 -> 油门占空比 """
#         # 这些系数应根据实际拉力台数据配置
#         A = 1.2e-7
#         B = 1.5e-4
#         C = 0.05
#         return A * omega_cmd**2 + B * omega_cmd + C
        
#     def compute_throttle(self, omega_cmd, omega_meas, dt):
#         """
#         计算发送给电调的最终油门指令 
#         :param omega_cmd: 期望转速 [2,]
#         :param omega_meas: 实际测得转速 [2,]
#         """
#         # 计算误差并累加积分
#         error = omega_cmd - omega_meas
#         self.integral_error += error * dt
        
#         # 积分限幅 (防积分饱和 Anti-windup)
#         self.integral_error = np.clip(self.integral_error, -0.2, 0.2)
        
#         # 前馈 + 积分 (Eq 53) 
#         q_out = self._feedforward_poly(omega_cmd) + self.k_I * self.integral_error
        
#         # 输出限幅 0~1 (0~100% 油门)
#         return np.clip(q_out, 0.0, 1.0)
