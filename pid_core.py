"""
PID 控制器 + 自适应参数调节
"""

import time
from collections import deque


class PIDController:
    """离散 PID 控制器"""

    def __init__(self, kp=1.0, ki=0.0, kd=0.0, setpoint=0.0, output_min=-100, output_max=100):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint

        self.output_min = output_min
        self.output_max = output_max

        self._prev_error = 0.0
        self._integral = 0.0
        self._last_time = None

    def reset(self):
        self._prev_error = 0.0
        self._integral = 0.0
        self._last_time = None

    def update(self, measurement: float, dt: float = None) -> float:
        """输入当前测量值，返回 PID 输出"""
        now = time.perf_counter()

        if self._last_time is None:
            self._last_time = now
            self._prev_error = self.setpoint - measurement
            return 0.0

        if dt is None:
            dt = now - self._last_time
        self._last_time = now

        if dt <= 0:
            return 0.0

        error = self.setpoint - measurement

        # Proportional
        p_term = self.kp * error

        # Integral (带抗饱和)
        self._integral += error * dt
        i_term = self.ki * self._integral

        # Derivative (对测量值微分，避免微分冲击)
        d_term = self.kd * (error - self._prev_error) / dt

        self._prev_error = error

        output = p_term + i_term + d_term
        output = max(self.output_min, min(self.output_max, output))
        return output


class PIDAutoTuner:
    """
    基于规则的 PID 参数自适应调节器

    调节策略：
      - 超调过大 → 增大 Kd，减小 Kp
      - 上升时间过长 → 增大 Kp
      - 稳态误差大 → 增大 Ki
      - 持续振荡 → 减小 Kp，增大 Kd
      - Ki 过大导致低频振荡 → 减小 Ki
    """

    def __init__(self, kp=1.0, ki=0.0, kd=0.0,
                 kp_range=(0.0, 100.0), ki_range=(0.0, 50.0), kd_range=(0.0, 50.0),
                 adapt_rate=0.05):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.kp_range = kp_range
        self.ki_range = ki_range
        self.kd_range = kd_range
        self.adapt_rate = adapt_rate

        self._error_history = deque(maxlen=200)
        self._output_history = deque(maxlen=200)
        self._peak_error = 0.0
        self._prev_sign = 0
        self._zero_crossings = 0
        self._tune_counter = 0
        self._tune_interval = 50  # 每 N 个采样点评估一次

    def feed(self, error: float, output: float):
        """喂入当前误差和 PID 输出，内部统计"""
        self._error_history.append(error)
        self._output_history.append(output)

    def should_tune(self) -> bool:
        self._tune_counter += 1
        if self._tune_counter >= self._tune_interval:
            self._tune_counter = 0
            return len(self._error_history) >= 20
        return False

    def tune(self) -> tuple:
        """执行一次参数调整，返回新的 (kp, ki, kd)"""
        errors = list(self._error_history)
        if len(errors) < 20:
            return self.kp, self.ki, self.kd

        n = len(errors)

        # --- 稳态误差 (最近一段的平均误差) ---
        recent = errors[-20:]
        steady_err = sum(abs(e) for e in recent) / len(recent)

        # --- 超调检测：误差符号翻转 ---
        overshoot_ratio = self._calc_overshoot_ratio(errors)

        # --- 振荡检测：过零点计数 ---
        zero_crossings = 0
        for i in range(1, n):
            if errors[i] * errors[i - 1] < 0:
                zero_crossings += 1
        oscillation_ratio = zero_crossings / max(n - 1, 1)

        # --- 调节规则 ---
        dkp, dki, dkd = 0.0, 0.0, 0.0

        # 1. 超调过大 → 减 Kp，增 Kd
        if overshoot_ratio > 0.3:
            dkp -= self.adapt_rate * overshoot_ratio * self.kp
            dkd += self.adapt_rate * overshoot_ratio * self.kd

        # 2. 稳态误差大 → 增 Ki
        if steady_err > abs(self._error_history[-1]) * 0.1 + 0.01:
            dki += self.adapt_rate * 0.5 * self.ki + 0.001

        # 3. 持续振荡 → 减 Kp，减 Ki，增 Kd
        if oscillation_ratio > 0.4:
            dkp -= self.adapt_rate * 0.3 * self.kp
            dki -= self.adapt_rate * 0.3 * self.ki
            dkd += self.adapt_rate * 0.2 * self.kd

        # 4. 误差偏小、无振荡 → 增大 Kp 加速收敛
        if steady_err < 0.1 and oscillation_ratio < 0.1 and overshoot_ratio < 0.1:
            dkp += self.adapt_rate * 0.1

        # 应用增量
        self.kp = self._clamp(self.kp + dkp, *self.kp_range)
        self.ki = self._clamp(self.ki + dki, *self.ki_range)
        self.kd = self._clamp(self.kd + dkd, *self.kd_range)

        return self.kp, self.ki, self.kd

    def _calc_overshoot_ratio(self, errors: list) -> float:
        """计算超调比例"""
        max_err = max(abs(e) for e in errors)
        if max_err == 0:
            return 0.0
        recent = errors[-10:]
        steady = sum(abs(e) for e in recent) / len(recent) if recent else 0
        if max_err == 0:
            return 0.0
        return max(0.0, steady / max_err - 0.5)

    @staticmethod
    def _clamp(val, lo, hi):
        return max(lo, min(hi, val))
