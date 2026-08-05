"""RotorPy 风格的世界坐标系风速模型。"""

import numpy as np


def _as_vector3(value, name):
    """将输入规范为有限的三维浮点向量。"""
    vector = np.asarray(value, dtype=float)
    if vector.shape != (3,):
        raise ValueError(f"{name} 必须是包含 3 个元素的向量，当前形状为 {vector.shape}")
    if not np.all(np.isfinite(vector)):
        raise ValueError(f"{name} 必须只包含有限数值")
    return vector.copy()


class NoWind:
    """无风环境。"""

    def update(self, t, position):
        """返回世界 NED 坐标系中的风速 [m/s]。"""
        return np.zeros(3)


class ConstantWind:
    """世界 NED 坐标系中的常值风。"""

    def __init__(self, wind_velocity):
        self.wind_velocity = _as_vector3(wind_velocity, "wind_velocity")

    def update(self, t, position):
        """返回世界 NED 坐标系中的风速 [m/s]。"""
        return self.wind_velocity.copy()


class SinusoidalWind:
    """常值风叠加逐轴正弦阵风。"""

    def __init__(
        self,
        mean_velocity=(0.0, 0.0, 0.0),
        amplitude=(0.0, 0.0, 0.0),
        frequency_hz=0.5,
        phase=(0.0, 0.0, 0.0),
    ):
        self.mean_velocity = _as_vector3(mean_velocity, "mean_velocity")
        self.amplitude = _as_vector3(amplitude, "amplitude")
        self.phase = _as_vector3(phase, "phase")

        frequency = np.asarray(frequency_hz, dtype=float)
        if frequency.ndim == 0:
            frequency = np.full(3, float(frequency))
        self.frequency_hz = _as_vector3(frequency, "frequency_hz")
        if np.any(self.frequency_hz < 0.0):
            raise ValueError("frequency_hz 不能为负数")

    def update(self, t, position):
        """返回世界 NED 坐标系中的风速 [m/s]。"""
        return self.mean_velocity + self.amplitude * np.sin(
            2.0 * np.pi * self.frequency_hz * float(t) + self.phase
        )


class TimeWindowWind:
    """只在指定的半开时间区间 ``[start_time, end_time)`` 启用风场。"""

    def __init__(self, wind_profile, start_time=0.0, end_time=np.inf):
        if not callable(getattr(wind_profile, "update", None)):
            raise TypeError("wind_profile 必须提供 update(t, position) 方法")

        self.start_time = float(start_time)
        self.end_time = float(end_time)
        if not np.isfinite(self.start_time):
            raise ValueError("start_time 必须是有限数值")
        if np.isnan(self.end_time):
            raise ValueError("end_time 不能是 NaN")
        if self.end_time <= self.start_time:
            raise ValueError("end_time 必须大于 start_time")

        self.wind_profile = wind_profile

    def update(self, t, position):
        """时间窗内返回被包装风场，时间窗外返回零风速。"""
        time = float(t)
        if self.start_time <= time < self.end_time:
            return _as_vector3(
                self.wind_profile.update(time, position),
                "wind_profile.update() 的返回值",
            )
        return np.zeros(3)
