"""将规划器导出的平坦输出 CSV 包装成闭环仿真的轨迹接口。"""

from __future__ import annotations

from pathlib import Path

import numpy as np


class PlanningCsvTrajectory:
    """读取 ``planning/output`` 中的轨迹并提供连续时间采样。

    规划 CSV 已经以不超过 2 ms 的间隔稠密导出位置到 jerk 以及 yaw 到 yaw
    acceleration。控制器同样以 2 ms 更新，因此对各个已导出的平坦输出分别做
    线性插值。不能在这里对相邻 CSV 行重新构造高阶 Hermite 多项式：规划器会
    在样条连接点额外插入采样，连接点和相邻规则采样可能只相差约 0.1 ms；CSV
    的有限输出精度会使这种极短区间上的七次重构产生人为 jerk 尖峰。
    """

    _REQUIRED_COLUMNS = (
        "t",
        "px", "py", "pz",
        "vx", "vy", "vz",
        "ax", "ay", "az",
        "jx", "jy", "jz",
        "yaw", "yaw_rate", "yaw_acceleration",
    )

    def __init__(self, csv_path: str | Path, hold_duration: float = 1.0):
        self.csv_path = Path(csv_path).expanduser().resolve()
        if not self.csv_path.is_file():
            raise FileNotFoundError(f"找不到规划轨迹 CSV：{self.csv_path}")
        if hold_duration < 0.0:
            raise ValueError("末端悬停时间不能为负")

        table = np.genfromtxt(self.csv_path, delimiter=",", names=True)
        if table.dtype.names is None:
            raise ValueError(f"{self.csv_path.name} 缺少 CSV 表头")
        missing = [
            name for name in self._REQUIRED_COLUMNS if name not in table.dtype.names
        ]
        if missing:
            raise ValueError(
                f"{self.csv_path.name} 缺少轨迹列：{', '.join(missing)}"
            )

        self._time = np.atleast_1d(np.asarray(table["t"], dtype=float))
        if self._time.size < 2:
            raise ValueError("规划轨迹至少需要两个采样点")
        if not np.all(np.isfinite(self._time)) or not np.all(np.diff(self._time) > 0):
            raise ValueError("规划轨迹时间必须有限且严格递增")
        if abs(self._time[0]) > 1e-9:
            raise ValueError("规划轨迹必须从 t=0 开始")

        position = self._columns(table, ("px", "py", "pz"))
        velocity = self._columns(table, ("vx", "vy", "vz"))
        acceleration = self._columns(table, ("ax", "ay", "az"))
        jerk = self._columns(table, ("jx", "jy", "jz"))
        yaw = np.unwrap(np.atleast_1d(np.asarray(table["yaw"], dtype=float)))
        yaw_rate = np.atleast_1d(np.asarray(table["yaw_rate"], dtype=float))
        yaw_acceleration = np.atleast_1d(
            np.asarray(table["yaw_acceleration"], dtype=float)
        )
        for name, values in (
            ("position", position),
            ("velocity", velocity),
            ("acceleration", acceleration),
            ("jerk", jerk),
            ("yaw", yaw),
            ("yaw_rate", yaw_rate),
            ("yaw_acceleration", yaw_acceleration),
        ):
            if not np.all(np.isfinite(values)):
                raise ValueError(f"规划轨迹的 {name} 含非有限值")

        self._position = position
        self._velocity = velocity
        self._acceleration = acceleration
        self._jerk = jerk
        self._yaw = yaw
        self._yaw_rate = yaw_rate

        self.planned_duration = float(self._time[-1])
        self.hold_duration = float(hold_duration)
        self.total_duration = self.planned_duration + self.hold_duration
        self.initial_state_mode = "hover"
        self._terminal_position = position[-1].copy()
        self._terminal_yaw = float(yaw[-1])

    @staticmethod
    def _columns(table: np.ndarray, names: tuple[str, str, str]) -> np.ndarray:
        return np.column_stack(
            [np.atleast_1d(np.asarray(table[name], dtype=float)) for name in names]
        )

    def sample(self, t: float):
        """返回位置、速度、加速度、jerk、yaw 和 yaw rate 参考。"""
        t = float(t)
        if t < 0.0:
            raise ValueError("轨迹采样时间不能为负")
        if t >= self.planned_duration:
            return (
                self._terminal_position.copy(),
                np.zeros(3),
                np.zeros(3),
                np.zeros(3),
                self._terminal_yaw,
                0.0,
            )

        def interpolate_vector(values: np.ndarray) -> np.ndarray:
            return np.array(
                [np.interp(t, self._time, values[:, axis]) for axis in range(3)]
            )

        return (
            interpolate_vector(self._position),
            interpolate_vector(self._velocity),
            interpolate_vector(self._acceleration),
            interpolate_vector(self._jerk),
            float(np.interp(t, self._time, self._yaw)),
            float(np.interp(t, self._time, self._yaw_rate)),
        )
