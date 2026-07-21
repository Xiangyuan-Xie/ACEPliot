"""Scripted piecewise-constant velocity profile."""

from dataclasses import dataclass
import math
from typing import Sequence, Tuple


@dataclass(frozen=True)
class VelocityCommand:
    linear_enu_m_s: Tuple[float, float, float]
    yaw_rate_enu_rad_s: float
    finished: bool


class VelocityProfile:
    """Validate and sample a YAML-defined velocity stage table."""

    def __init__(
        self,
        durations_s: Sequence[float],
        vx_m_s: Sequence[float],
        vy_m_s: Sequence[float],
        vz_m_s: Sequence[float],
        yaw_rate_rad_s: Sequence[float],
        loop: bool,
        max_linear_speed_m_s: float,
        max_yaw_rate_rad_s: float,
    ) -> None:
        self.durations_s = tuple(float(value) for value in durations_s)
        self.velocities = tuple(
            (float(vx), float(vy), float(vz))
            for vx, vy, vz in zip(vx_m_s, vy_m_s, vz_m_s)
        )
        self.yaw_rates = tuple(float(value) for value in yaw_rate_rad_s)
        self.loop = bool(loop)
        lengths = {
            len(self.durations_s),
            len(vx_m_s),
            len(vy_m_s),
            len(vz_m_s),
            len(self.yaw_rates),
        }
        if len(lengths) != 1 or not self.durations_s:
            raise ValueError("velocity profile arrays must have one matching non-zero length")
        if not math.isfinite(max_linear_speed_m_s) or max_linear_speed_m_s <= 0.0:
            raise ValueError("profile.max_linear_speed_m_s must be positive")
        if not math.isfinite(max_yaw_rate_rad_s) or max_yaw_rate_rad_s <= 0.0:
            raise ValueError("profile.max_yaw_rate_rad_s must be positive")
        for duration in self.durations_s:
            if not math.isfinite(duration) or duration <= 0.0:
                raise ValueError("profile durations must be finite and positive")
        for velocity, yaw_rate in zip(self.velocities, self.yaw_rates):
            if not all(math.isfinite(value) for value in (*velocity, yaw_rate)):
                raise ValueError("velocity profile commands must be finite")
            if math.sqrt(sum(value * value for value in velocity)) > max_linear_speed_m_s + 1.0e-9:
                raise ValueError("velocity profile exceeds profile.max_linear_speed_m_s")
            if abs(yaw_rate) > max_yaw_rate_rad_s + 1.0e-9:
                raise ValueError("velocity profile exceeds profile.max_yaw_rate_rad_s")
        self.total_duration_s = sum(self.durations_s)

    def sample(self, elapsed_s: float) -> VelocityCommand:
        if not math.isfinite(elapsed_s):
            raise ValueError("elapsed_s must be finite")
        elapsed_s = max(0.0, elapsed_s)
        if not self.loop and elapsed_s >= self.total_duration_s:
            return VelocityCommand((0.0, 0.0, 0.0), 0.0, True)
        if self.loop:
            elapsed_s %= self.total_duration_s
        cursor = 0.0
        for duration, velocity, yaw_rate in zip(
            self.durations_s, self.velocities, self.yaw_rates
        ):
            cursor += duration
            if elapsed_s < cursor:
                return VelocityCommand(velocity, yaw_rate, False)
        return VelocityCommand((0.0, 0.0, 0.0), 0.0, True)


class PoseVelocityEstimator:
    """Estimate world velocity from timestamped positions with low-pass filtering."""

    def __init__(self, min_dt_s: float, max_dt_s: float, low_pass_alpha: float) -> None:
        if min_dt_s <= 0.0 or max_dt_s <= min_dt_s:
            raise ValueError("velocity estimator dt limits are invalid")
        if not 0.0 < low_pass_alpha <= 1.0:
            raise ValueError("velocity_estimator.low_pass_alpha must be in (0, 1]")
        self.min_dt_s = float(min_dt_s)
        self.max_dt_s = float(max_dt_s)
        self.alpha = float(low_pass_alpha)
        self.reset()

    def reset(self) -> None:
        self._last_time_s = None
        self._last_position = None
        self._filtered = None

    def update(self, stamp_s: float, position: Sequence[float]):
        position = tuple(float(value) for value in position)
        if not math.isfinite(stamp_s) or not all(math.isfinite(value) for value in position):
            self.reset()
            return None
        if self._last_time_s is None:
            self._last_time_s = stamp_s
            self._last_position = position
            return None
        dt_s = stamp_s - self._last_time_s
        if dt_s <= 0.0 or dt_s > self.max_dt_s:
            self._last_time_s = stamp_s
            self._last_position = position
            self._filtered = None
            return None
        if dt_s < self.min_dt_s:
            return None
        raw = tuple(
            (current - previous) / dt_s
            for current, previous in zip(position, self._last_position)
        )
        if self._filtered is None:
            self._filtered = raw
        else:
            self._filtered = tuple(
                self.alpha * current + (1.0 - self.alpha) * previous
                for current, previous in zip(raw, self._filtered)
            )
        self._last_time_s = stamp_s
        self._last_position = position
        return self._filtered
