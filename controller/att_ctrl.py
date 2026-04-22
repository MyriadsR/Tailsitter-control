# controller/attitude_ctrl.py
import numpy as np
from utils.math_help import zxy_euler_to_quaternion

class FlatnessAttitudeController:
    def __init__(self, config):
        self.cfg = config
        self.alpha_0 = config.alpha_0
        self.alpha_T = config.alpha_T
        self.alpha_bar = self.alpha_0 + self.alpha_T
        
        # 预计算一些常用的气动系数常数，提升运行效率
        self.c_a_bar = np.cos(self.alpha_bar)
        self.s_a_bar = np.sin(self.alpha_bar)
        self.c_DT = config.c_DT
        self.c_LT = config.c_LT
        self.c_DV = config.c_DV
        self.c_LV = config.c_LV
        self.c_LV_delta = config.c_LV_delta
        self.c_LT_delta = config.c_LT_delta

    def compute_desired_attitude_and_thrust(self, f_c_i, v_i, delta_sum, psi_ref, q_current):
        r"""
        利用微分平坦变换，从期望受力反解期望姿态和推力 (论文公式 17-23)
        
        :param f_c_i: INDI 计算出的期望力向量 (世界系 NED) [3,]
        :param v_i: 当前飞行器速度 (世界系 NED) [3,]
        :param delta_sum: 左右襟翼偏角之和 \delta_1 + \delta_2 (rad)
        :param psi_ref: 期望偏航角 (rad)
        :param q_current: 当前姿态四元数 (w, x, y, z)
        :return: q_c (期望姿态四元数), T_c (期望总推力)
        """
        v_norm = np.linalg.norm(v_i)
        
        # ==========================================
        # 第一步：计算中间坐标系的力与速度 (满足偏航约束)
        # ==========================================
        # 旋转矩阵 R_i_to_psi: 世界系绕 Z 轴旋转 psi_ref
        c_psi, s_psi = np.cos(psi_ref), np.sin(psi_ref)
        R_i_to_psi = np.array([
            [ c_psi, s_psi, 0],
            [-s_psi, c_psi, 0],
            [     0,     0, 1]
        ])
        f_psi = R_i_to_psi @ f_c_i

        # ==========================================
        # 提取当前机体 Y 轴在世界系下的方向向量 (b_y_current)
        # ==========================================
        qw, qx, qy, qz = q_current
        # 四元数转旋转矩阵 R_b^i 的第二列就是机体 Y 轴在世界系下的表示
        b_y_current = np.array([
            2 * (qx * qy - qz * qw),
            1 - 2 * (qx**2 + qz**2),
            2 * (qy * qz + qx * qw)
        ])
        
        # ==========================================
        # 第二步：求解滚转角 phi (含奇点保护与 k*pi 连续性保护)
        # ==========================================
        
        # 检查奇点：指令力完全在水平面且没有侧向分量 (两个参数趋近于0)
        # 1e-4 牛顿(约 10克) 是一个合理的死区容差阈值
        if abs(f_psi[1]) < 1e-4 and abs(f_c_i[2]) < 1e-4:
            # 【奇点触发】：任意 phi 均合法。
            # 为了姿态平滑，选取与当前机体 Y 轴最接近的角度。
            # 将 b_y_current 投影到当前的 psi 中间坐标系
            b_y_psi = R_i_to_psi @ b_y_current
            
            # 在 psi 系的 Y-Z 面上计算保持当前姿态所需的滚转角
            phi = np.arctan2(b_y_psi[2], b_y_psi[1])
            
        else:
            # 【正常情况】：求解使侧向力为 0 的 phi
            # 1. 算出基础解
            phi_0 = -np.arctan2(f_psi[1], f_c_i[2])
            
            # 2. 构造 R_phi_to_i 来测试这个基础解
            c_phi0, s_phi0 = np.cos(phi_0), np.sin(phi_0)
            R_psi_to_phi_0 = np.array([
                [1,      0,       0],
                [0, c_phi0,  s_phi0],
                [0,-s_phi0,  c_phi0]
            ])
            # R_i_to_phi_0 = R_psi_to_phi_0 @ R_i_to_psi
            # 新的 Y 轴方向是 R_phi_to_i_0 的第二列
            new_y_axis = (R_psi_to_phi_0 @ R_i_to_psi).T[:, 1]
            
            # 3. 点乘判断 (k*pi 保护)，防止飞机“倒扣”翻面
            if np.dot(b_y_current, new_y_axis) > 0:
                phi = phi_0
            else:
                phi = phi_0 + np.pi
        
        
        # 计算进入 phi 坐标系的旋转矩阵 R_psi_to_phi (绕 X 轴旋转 phi)
        c_phi, s_phi = np.cos(phi), np.sin(phi)
        R_psi_to_phi = np.array([
            [1,      0,      0],
            [0,  c_phi,  s_phi],
            [0, -s_phi,  c_phi]
        ])
        
        # 转换力与速度到 phi 中间坐标系
        R_i_to_phi = R_psi_to_phi @ R_i_to_psi
        f_phi = R_i_to_phi @ f_c_i
        v_phi = R_i_to_phi @ v_i
        
        fx, fz = f_phi[0], f_phi[2]
        vx, vz = v_phi[0], v_phi[2]
        
        # ==========================================
        # 第三步：求解零升力俯仰角 theta_bar (Eq 21, 23)
        # ==========================================
        # 1. 计算比例常数 eta (Eq 23)
        eta_num = self.s_a_bar * (self.c_LT - 1) - self.c_a_bar * self.c_LT_delta * delta_sum / 2.0
        eta_den = self.c_a_bar * (1 - self.c_DT)
        eta = eta_num / eta_den
        
        # 2. 收集 atan2 的 Y (对应 s_theta_bar) 和 X (对应 c_theta_bar)
        Y = (eta * (fx + self.c_DV * v_norm * vx) 
             - self.c_LV_delta * delta_sum * v_norm * vx 
             - self.c_LV * v_norm * vz 
             - fz)
             
        X = (eta * (fz + self.c_DV * v_norm * vz) 
             - self.c_LV_delta * delta_sum * v_norm * vz 
             + self.c_LV * v_norm * vx 
             + fx)
        
        # 3. 检查奇点：如果 X 和 Y 都趋近于 0，任意 theta_bar 都满足受力条件
        # 1e-4 为死区容差
        if abs(Y) < 1e-4 and abs(X) < 1e-4:
            # 【奇点触发】：为了姿态平滑，选取与当前机体最接近的俯仰角
            
            # 提取当前机体 X 轴 (b_x_current) 和 Z 轴 (b_z_current)
            b_x_current = np.array([
                1 - 2 * (qy**2 + qz**2),
                2 * (qx * qy + qz * qw),
                2 * (qx * qz - qy * qw)
            ])
            b_z_current = np.array([
                2 * (qx * qz + qy * qw),
                2 * (qy * qz - qx * qw),
                1 - 2 * (qx**2 + qy**2)
            ])
            
            # 还原当前零升力坐标系的 X 轴 (alpha_x_current)
            # 物理关系：从机体系旋转到零升力系是绕负 Y 轴旋转 alpha_0，
            # 等价于绕正 Y 轴旋转 -alpha_0。在矩阵投影下表现为如下组合：
            c_a0, s_a0 = np.cos(self.alpha_0), np.sin(self.alpha_0)
            alpha_x_current = c_a0 * b_x_current + s_a0 * b_z_current
            
            # 将 alpha_x_current 投影到刚刚算出的 phi 中间坐标系
            alpha_x_phi = R_i_to_phi @ alpha_x_current
            
            # 在 phi 坐标系的 X-Z 平面上，提取保持姿态所需的 theta_bar
            # 注意标准俯仰定义下，X 轴分量对应 cos，-Z 轴分量对应 sin
            theta_bar = np.arctan2(-alpha_x_phi[2], alpha_x_phi[0])
            
        else:
            # 【正常情况】：完全按照论文公式(21)求解
            theta_bar = np.arctan2(Y, X)
        
        # ==========================================
        # 第四步：求解总推力 T (Eq 22)
        # ==========================================
        c_th, s_th = np.cos(theta_bar), np.sin(theta_bar)
        
        T_num = (c_th * fx - s_th * fz) + self.c_DV * v_norm * (c_th * vx - s_th * vz)
        T_c = T_num / eta_den
        
        # 四象限及负推力保护：如果推力为负，说明解算到了背向运动学的奇点
        # 根据论文描述，通过加上 k*pi (这里 k=1) 将机头反转，推力变正
        if T_c < 0:
            theta_bar += np.pi
            c_th, s_th = np.cos(theta_bar), np.sin(theta_bar)
            T_num = (c_th * fx - s_th * fz) + self.c_DV * v_norm * (c_th * vx - s_th * vz)
            T_c = T_num / eta_den
            
        # ==========================================
        # 第五步：组装最终姿态并转换为四元数
        # ==========================================
        # 真实的机体俯仰角 = 零升力俯仰角 + 零升力迎角
        theta = theta_bar + self.alpha_0
        
        # 使用工具函数将 ZXY 欧拉角 (psi, phi, theta) 转换为四元数
        q_c = zxy_euler_to_quaternion(psi_ref, phi, theta)
        
        return q_c, T_c
    
    def compute_feedforward_angular_velocity(self, v_ref, a_ref, j_ref, psi_ref, psi_dot_ref, f_c_i, phi, theta_bar, delta_sum):
        r"""
        计算平坦变换的前馈角速度 \Omega_{ref} (论文公式 24-35)
        
        :param v_ref: 期望速度 [3,]
        :param a_ref: 期望加速度 [3,]
        :param j_ref: 期望 Jerk (加速度的导数) [3,]
        :param psi_ref: 期望偏航角
        :param psi_dot_ref: 期望偏航角速度
        :param f_c_i: 期望力 (世界系)
        :param phi: 已解算出的期望滚转角
        :param theta_bar: 已解算出的期望零升力俯仰角
        :param delta_sum: 襟翼偏角之和
        :return: omega_ref 期望机体角速度 [3,]
        """
        v_norm = np.linalg.norm(v_ref)
        if v_norm < 1e-3:
            v_norm = 1e-3
            
        # ==========================================
        # 0. 准备旋转矩阵与导数 (Eq 27, 33)
        # ==========================================
        # f_dot_i = m * j_ref (公式 27，假设外部扰动恒定，导数为0)
        f_dot_i = self.cfg.mass * j_ref
        
        # v_norm 的导数 (公式 33)
        v_norm_dot = np.dot(v_ref, a_ref) / v_norm
        
        c_psi, s_psi = np.cos(psi_ref), np.sin(psi_ref)
        c_phi, s_phi = np.cos(phi), np.sin(phi)
        
        # 构造中间旋转矩阵
        R_i_to_psi = np.array([
            [ c_psi, s_psi, 0],
            [-s_psi, c_psi, 0],
            [     0,     0, 1]
        ])
        R_psi_to_phi = np.array([
            [1,      0,      0],
            [0,  c_phi,  s_phi],
            [0, -s_phi,  c_phi]
        ])
        R_i_to_phi = R_psi_to_phi @ R_i_to_psi
        
        # ==========================================
        # 1. 求解滚转角速度 phi_dot (Eq 24-26)
        # ==========================================
        # beta_x = -sin(psi)*fx^i + cos(psi)*fy^i  (即 f_psi 的 y 分量)
        beta_x = -s_psi * f_c_i[0] + c_psi * f_c_i[1]
        beta_z = f_c_i[2]
        
        # 对 beta_x 和 beta_z 求导
        beta_x_dot = -psi_dot_ref * (c_psi * f_c_i[0] + s_psi * f_c_i[1]) - s_psi * f_dot_i[0] + c_psi * f_dot_i[1]
        beta_z_dot = f_dot_i[2]
        
        # phi_dot = - (beta_x*beta_z_dot - beta_x_dot*beta_z) / (beta_x^2 + beta_z^2) (Eq 24)
        denominator_phi = beta_x**2 + beta_z**2
        if denominator_phi < 1e-6:
            phi_dot = 0.0  # 奇点保护
        else:
            phi_dot = - (beta_x_dot * beta_z - beta_x * beta_z_dot) / denominator_phi

        # ==========================================
        # 2. 求解坐标系内的导数 v_phi_dot 和 f_phi_dot (Eq 34)
        # 这里还得多注意是不是对的！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！
        # ==========================================
        # phi 坐标系相对于世界系的角速度 (投影在 phi 坐标系下)
        omega_phi_i_in_phi = R_i_to_phi @ np.array([0, 0, psi_dot_ref]) + np.array([phi_dot, 0, 0])
        
        v_phi = R_i_to_phi @ v_ref
        f_phi = R_i_to_phi @ f_c_i
        
        # 根据输运定理 (Transport Theorem) 求局部导数
        v_phi_dot = R_i_to_phi @ a_ref - np.cross(omega_phi_i_in_phi, v_phi)
        f_phi_dot = R_i_to_phi @ f_dot_i - np.cross(omega_phi_i_in_phi, f_phi)
        
        fx, fz = f_phi[0], f_phi[2]
        vx, vz = v_phi[0], v_phi[2]
        fx_dot, fz_dot = f_phi_dot[0], f_phi_dot[2]
        vx_dot, vz_dot = v_phi_dot[0], v_phi_dot[2]

        # ==========================================
        # 3. 求解俯仰角速度 theta_dot (Eq 28-32)
        # ==========================================
        tau_x = v_norm_dot * vx + v_norm * vx_dot  # Eq 31
        tau_z = v_norm_dot * vz + v_norm * vz_dot  # Eq 32
        
        # 计算比例常数 eta
        eta_num = self.s_a_bar * (self.c_LT - 1) - self.c_a_bar * self.c_LT_delta * delta_sum / 2.0
        eta_den = self.c_a_bar * (1 - self.c_DT)
        eta = eta_num / eta_den
        
        # 对应 Eq 21 的 Y 和 X (论文中的 sigma_x 和 sigma_z)
        Y = eta * (fx + self.c_DV * v_norm * vx) - self.c_LV_delta * delta_sum * v_norm * vx - self.c_LV * v_norm * vz - fz
        X = eta * (fz + self.c_DV * v_norm * vz) - self.c_LV_delta * delta_sum * v_norm * vz + self.c_LV * v_norm * vx + fx
        
        # 求导数 (Eq 29, 30)
        Y_dot = eta * (fx_dot + self.c_DV * tau_x) - self.c_LV_delta * delta_sum * tau_x - self.c_LV * tau_z - fz_dot
        X_dot = eta * (fz_dot + self.c_DV * tau_z) - self.c_LV_delta * delta_sum * tau_z + self.c_LV * tau_x + fx_dot
        
        # theta_dot = (X*Y_dot - Y*X_dot) / (X^2 + Y^2) (对应 Eq 28 的标准导数形式)
        denominator_th = X**2 + Y**2
        if denominator_th < 1e-6:
            theta_dot = 0.0
        else:
            theta_dot = (X * Y_dot - Y * X_dot) / denominator_th

        # ==========================================
        # 4. 组装最终的前馈角速度 Omega_ref (Eq 35)
        # ==========================================
        # 需要将三部分的角速度全部统一投影到机体坐标系下
        # R_phi_to_b: 从 phi 坐标系转到最终机体系 (绕 Y 轴旋转 theta)
        # 注意这里 theta 自带正负号
        theta = theta_bar + self.alpha_0
        c_th, s_th = np.cos(theta), np.sin(theta)
        R_phi_to_b = np.array([
            [ c_th,  0,  s_th],
            [    0,  1,     0],
            [-s_th,  0,  c_th]
        ])
        
        R_i_to_b = R_phi_to_b @ R_i_to_phi
        
        # 公式 35：偏航角速度 + 滚转角速度 + 俯仰角速度
        omega_ref = R_i_to_b @ np.array([0, 0, psi_dot_ref]) + \
                    R_phi_to_b @ np.array([phi_dot, 0, 0]) + \
                    np.array([0, theta_dot, 0])
                    
        return omega_ref
    
class PDAttitudeController:
    def __init__(self, config):
        self.cfg = config
        # 姿态控制增益矩阵 (对角阵)
        self.K_xi = np.diag([self.cfg.k_xi_roll, self.cfg.k_xi_pitch, self.cfg.k_xi_yaw])
        self.K_omega = np.diag([self.cfg.k_omega_roll, self.cfg.k_omega_pitch, self.cfg.k_omega_yaw])

    def _quat_inverse(self, q):
        """四元数求逆 [qw, qx, qy, qz] -> [qw, -qx, -qy, -qz] (假设为单位四元数)"""
        return np.array([q[0], -q[1], -q[2], -q[3]])

    def _quat_multiply(self, q1, q2):
        r"""四元数乘法 q1 \circ q2"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        return np.array([
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        ])

    def compute_angular_acceleration_command(self, q_current, q_c, omega_lpf, omega_ref):
        r"""
        计算期望角加速度 \dot{\Omega}_c (论文公式 47, 48, 49)
        
        :param q_current: 当前姿态四元数 \xi
        :param q_c: 期望姿态四元数 \xi_c
        :param omega_lpf: 滤波后的当前机体角速度 \Omega_{lpf}
        :param omega_ref: 前馈期望角速度 \Omega_{ref} (基于平坦变换)
        :return: omega_dot_c 期望角加速度 [3,]
        """
        # 1. 计算四元数误差 (Eq 47): \xi_e = \xi^{-1} \circ \xi_c
        q_inv = self._quat_inverse(q_current)
        q_e = self._quat_multiply(q_inv, q_c)
        
        # 确保最短路径旋转 (如果 w 为负，翻转整个四元数)
        if q_e[0] < 0:
            q_e = -q_e
            
        qw, qx, qy, qz = q_e
        q_vec = np.array([qx, qy, qz])
        
        # 2. 将四元数误差转换为 3D 误差向量 \zeta_c (Eq 48)
        # 加入小角度(奇点)保护机制
        if qw > 0.9999:
            # 极限趋近于 1 时，系数退化为 2
            zeta_c = 2.0 * q_vec
        else:
            coefficient = 2.0 * np.arccos(qw) / np.sqrt(1.0 - qw**2)
            zeta_c = coefficient * q_vec
            
        # 3. PD 控制律算出期望角加速度 (Eq 49)
        # \dot{\Omega}_c = K_\xi \zeta_c + K_\Omega (\Omega_{ref} - \Omega_{lpf})
        omega_dot_c = self.K_xi @ zeta_c + self.K_omega @ (omega_ref - omega_lpf)
        
        return omega_dot_c
    
class INDIAngularController:
    def __init__(self, config):
        """
        初始化 INDI 角加速度控制器
        :param config: 包含飞行器转动惯量的配置对象
        """
        # 提取对角线转动惯量或完整的 3x3 转动惯量矩阵 J
        self.J = np.array(config.inertia_matrix, dtype=float)

    def compute_moment_command(self, omega_dot_c, omega_dot_lpf, m_lpf):
        r"""
        利用 INDI 计算期望机体物理力矩 m_c (论文公式 52)
        
        :param omega_dot_c: PD姿态控制器算出的期望角加速度 \dot{\Omega}_c [3,]
        :param omega_dot_lpf: 实际测得(并滤波)的当前角加速度 \dot{\Omega}_{lpf} [3,]
        :param m_lpf: 基于滤波后的传感器数据(转速、舵偏)算出的当前机体气动力矩 m_{lpf} [3,]
        :return: m_c 期望力矩 (机体坐标系) [3,]
        """
        # 核心 INDI 控制律 (Eq 52)
        # m_c = J(\dot{\Omega}_c - \dot{\Omega}_{lpf}) + m_{lpf}
        
        # 计算角加速度增量需求
        delta_omega_dot = omega_dot_c - omega_dot_lpf
        
        # 乘以转动惯量得到所需的力矩增量，并加上当前模型输出的力矩
        m_c = self.J @ delta_omega_dot + m_lpf
        
        return m_c