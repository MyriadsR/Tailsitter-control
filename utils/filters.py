"""实时控制循环使用的数字滤波器。"""

import numpy as np
from scipy.signal import butter, sosfilt_zi


class SecondOrderButterworthFilter:
    """逐采样更新的二阶 Butterworth 低通或高通滤波器。

    同一个实例可以同时处理多个独立通道；所有通道共享完全相同的
    Butterworth 系数，从而保证 INDI 惯性测量与执行器测量的相位匹配。
    """

    def __init__(
        self,
        cutoff_hz,
        sample_rate_hz,
        channels,
        filter_type,
        initial_value,
    ):
        if cutoff_hz <= 0.0:
            raise ValueError("截止频率必须大于 0")
        if sample_rate_hz <= 0.0:
            raise ValueError("采样频率必须大于 0")
        if cutoff_hz >= 0.5 * sample_rate_hz:
            raise ValueError("截止频率必须小于 Nyquist 频率")
        if int(channels) != channels or channels <= 0:
            raise ValueError("通道数必须为正整数")
        if filter_type not in {"lowpass", "highpass"}:
            raise ValueError("filter_type 必须是 lowpass 或 highpass")

        self.channels = int(channels)
        self.sos = butter(
            2,
            cutoff_hz,
            btype=filter_type,
            fs=sample_rate_hz,
            output="sos",
        )
        if self.sos.shape != (1, 6):
            raise RuntimeError("二阶 Butterworth 滤波器应只有一个二阶节")

        # scipy SOS 行格式为 [b0, b1, b2, a0, a1, a2]。butter 返回的
        # a0 通常为 1，这里仍显式归一化，避免更新公式依赖隐含条件。
        section = self.sos[0].copy()
        section[:3] /= section[3]
        section[4:] /= section[3]
        section[3] = 1.0
        self.b0, self.b1, self.b2 = section[:3]
        _, self.a1, self.a2 = section[3:]

        initial = self._as_sample(initial_value)
        steady_state = sosfilt_zi(self.sos)[0]
        self.state = steady_state[:, None] * initial[None, :]

    def _as_sample(self, sample):
        values = np.asarray(sample, dtype=float)
        if values.shape != (self.channels,):
            raise ValueError(
                f"输入形状必须为 ({self.channels},)，实际为 {values.shape}"
            )
        if not np.all(np.isfinite(values)):
            raise ValueError("滤波器输入必须全部为有限值")
        return values

    def update(self, sample):
        """输入一个采样并返回所有通道的当前滤波输出。"""
        values = self._as_sample(sample)
        state_1 = self.state[0].copy()
        state_2 = self.state[1].copy()

        output = self.b0 * values + state_1
        self.state[0] = self.b1 * values - self.a1 * output + state_2
        self.state[1] = self.b2 * values - self.a2 * output
        return output
