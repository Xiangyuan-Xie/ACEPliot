"""Pure Figure-8 kinematics with a smooth phase-speed transition."""

from dataclasses import dataclass
import math
from typing import Tuple


TWO_PI = 2.0 * math.pi


@dataclass(frozen=True)
class Figure8Parameters:
    period_s: float = 10.0
    amplitude_x_m: float = 1.0
    amplitude_y_m: float = 0.5
    max_linear_speed_m_s: float = 1.0
    transition_time_s: float = 1.0
    loops_to_run: int = 0


@dataclass(frozen=True)
class Figure8Sample:
    position_xy_m: Tuple[float, float]
    velocity_xy_m_s: Tuple[float, float]
    acceleration_xy_m_s2: Tuple[float, float]
    phase_rad: float
    finished: bool


def _smooth5(value: float) -> float:
    value = min(1.0, max(0.0, value))
    return value**3 * (10.0 - 15.0 * value + 6.0 * value**2)


def _smooth5_derivative(value: float) -> float:
    value = min(1.0, max(0.0, value))
    return 30.0 * value**2 * (1.0 - value) ** 2


def _smooth5_integral(value: float) -> float:
    value = min(1.0, max(0.0, value))
    return value**4 * (2.5 - 3.0 * value + value**2)


class Figure8Profile:
    """Generate position, velocity, and acceleration for one horizontal Figure-8."""

    def __init__(self, parameters: Figure8Parameters) -> None:
        self.parameters = parameters
        self._validate()

    def _validate(self) -> None:
        values = (
            self.parameters.period_s,
            self.parameters.amplitude_x_m,
            self.parameters.amplitude_y_m,
            self.parameters.max_linear_speed_m_s,
            self.parameters.transition_time_s,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Figure-8 parameters must be finite")
        if self.parameters.period_s <= 0.0:
            raise ValueError("period_s must be positive")
        if self.parameters.amplitude_x_m < 0.0 or self.parameters.amplitude_y_m < 0.0:
            raise ValueError("Figure-8 amplitudes must be non-negative")
        if self.parameters.max_linear_speed_m_s <= 0.0:
            raise ValueError("max_linear_speed_m_s must be positive")
        if self.parameters.transition_time_s < 0.0:
            raise ValueError("transition_time_s must be non-negative")
        if self.parameters.loops_to_run < 0:
            raise ValueError("loops_to_run must be non-negative")
        if self.theoretical_max_speed_m_s > self.parameters.max_linear_speed_m_s + 1.0e-9:
            raise ValueError(
                "Figure-8 theoretical maximum speed "
                f"{self.theoretical_max_speed_m_s:.6f} m/s exceeds "
                f"max_linear_speed_m_s {self.parameters.max_linear_speed_m_s:.6f} m/s"
            )

    @property
    def theoretical_max_speed_m_s(self) -> float:
        omega = TWO_PI / self.parameters.period_s
        return omega * math.sqrt(
            self.parameters.amplitude_x_m**2
            + 4.0 * self.parameters.amplitude_y_m**2
        )

    def sample(self, elapsed_s: float) -> Figure8Sample:
        if not math.isfinite(elapsed_s):
            raise ValueError("elapsed_s must be finite")
        elapsed_s = max(0.0, elapsed_s)
        omega = TWO_PI / self.parameters.period_s
        phase_rate = omega
        phase_acceleration = 0.0
        if (
            0.0 < self.parameters.transition_time_s
            and elapsed_s < self.parameters.transition_time_s
        ):
            ratio = elapsed_s / self.parameters.transition_time_s
            phase = omega * self.parameters.transition_time_s * _smooth5_integral(ratio)
            phase_rate = omega * _smooth5(ratio)
            phase_acceleration = (
                omega
                * _smooth5_derivative(ratio)
                / self.parameters.transition_time_s
            )
        elif self.parameters.transition_time_s > 0.0:
            phase = omega * (elapsed_s - 0.5 * self.parameters.transition_time_s)
        else:
            phase = omega * elapsed_s

        finished = False
        if self.parameters.loops_to_run > 0:
            final_phase = TWO_PI * self.parameters.loops_to_run
            if phase >= final_phase:
                phase = final_phase
                phase_rate = 0.0
                phase_acceleration = 0.0
                finished = True
        if finished:
            return Figure8Sample((0.0, 0.0), (0.0, 0.0), (0.0, 0.0), phase, True)

        sin_phase = math.sin(phase)
        cos_phase = math.cos(phase)
        sin_double = math.sin(2.0 * phase)
        cos_double = math.cos(2.0 * phase)
        position = (
            self.parameters.amplitude_x_m * sin_phase,
            self.parameters.amplitude_y_m * sin_double,
        )
        velocity = (
            self.parameters.amplitude_x_m * cos_phase * phase_rate,
            2.0 * self.parameters.amplitude_y_m * cos_double * phase_rate,
        )
        acceleration = (
            self.parameters.amplitude_x_m
            * (-sin_phase * phase_rate**2 + cos_phase * phase_acceleration),
            2.0
            * self.parameters.amplitude_y_m
            * (-2.0 * sin_double * phase_rate**2 + cos_double * phase_acceleration),
        )
        return Figure8Sample(position, velocity, acceleration, phase, False)
