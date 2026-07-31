"""State estimation and cascaded control for a one-dimensional roller."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Sequence


@dataclass(frozen=True)
class BallState:
    timestamp_ms: int
    position_mm: float
    velocity_mm_s: float
    acceleration_mm_s2: float


class BallStateEstimator:
    def __init__(self, *, velocity_alpha: float = 0.35, acceleration_alpha: float = 0.20,
                 max_gap_ms: int = 250) -> None:
        self.velocity_alpha = self._alpha(velocity_alpha, "velocity_alpha")
        self.acceleration_alpha = self._alpha(acceleration_alpha, "acceleration_alpha")
        if max_gap_ms <= 0:
            raise ValueError("max_gap_ms must be positive")
        self.max_gap_ms = max_gap_ms
        self._state: BallState | None = None

    def update(self, timestamp_ms: int, position_mm: float) -> BallState:
        if not math.isfinite(position_mm):
            raise ValueError("ball position must be finite")
        previous = self._state
        if previous is None or timestamp_ms - previous.timestamp_ms > self.max_gap_ms:
            self._state = BallState(timestamp_ms, position_mm, 0.0, 0.0)
            return self._state
        delta_ms = timestamp_ms - previous.timestamp_ms
        if delta_ms <= 0:
            raise ValueError("ball timestamps must increase")
        elapsed_s = delta_ms / 1000.0
        raw_velocity = (position_mm - previous.position_mm) / elapsed_s
        velocity = self._blend(previous.velocity_mm_s, raw_velocity, self.velocity_alpha)
        raw_acceleration = (velocity - previous.velocity_mm_s) / elapsed_s
        acceleration = self._blend(previous.acceleration_mm_s2, raw_acceleration,
                                   self.acceleration_alpha)
        self._state = BallState(timestamp_ms, position_mm, velocity, acceleration)
        return self._state

    @staticmethod
    def _alpha(value: float, label: str) -> float:
        if not 0 < value <= 1:
            raise ValueError(f"{label} must be in (0, 1]")
        return value

    @staticmethod
    def _blend(previous: float, current: float, alpha: float) -> float:
        return alpha * current + (1.0 - alpha) * previous


class LinearTable:
    """A bounded monotonic one-dimensional calibration table."""

    def __init__(self, points: Iterable[Sequence[float]], *, name: str) -> None:
        parsed = tuple((float(item[0]), float(item[1])) for item in points)
        if len(parsed) < 2:
            raise ValueError(f"{name} requires at least two points")
        if any(not math.isfinite(value) for point in parsed for value in point):
            raise ValueError(f"{name} values must be finite")
        if any(right[0] <= left[0] for left, right in zip(parsed, parsed[1:])):
            raise ValueError(f"{name} inputs must strictly increase")
        self.name = name
        self.points = parsed

    @property
    def minimum(self) -> float:
        return self.points[0][0]

    @property
    def maximum(self) -> float:
        return self.points[-1][0]

    def at(self, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError(f"{self.name} input must be finite")
        if value <= self.minimum:
            return self.points[0][1]
        if value >= self.maximum:
            return self.points[-1][1]
        for left, right in zip(self.points, self.points[1:]):
            if value <= right[0]:
                ratio = (value - left[0]) / (right[0] - left[0])
                return left[1] + ratio * (right[1] - left[1])
        raise RuntimeError("linear table bounds are inconsistent")


@dataclass(frozen=True)
class PidParameters:
    kp: float
    ki: float
    kd: float
    output_limit: float
    integral_limit: float


class Pid:
    def __init__(self, parameters: PidParameters) -> None:
        if parameters.output_limit <= 0 or parameters.integral_limit < 0:
            raise ValueError("PID limits must be valid")
        self.parameters = parameters
        self.integral = 0.0
        self.previous_error: float | None = None

    def reset(self) -> None:
        self.integral = 0.0
        self.previous_error = None

    def update(self, error: float, elapsed_s: float, feedforward: float = 0.0) -> float:
        if not math.isfinite(error) or not math.isfinite(feedforward) or elapsed_s <= 0:
            raise ValueError("PID inputs must be finite and elapsed_s positive")
        derivative = 0.0 if self.previous_error is None else (error - self.previous_error) / elapsed_s
        candidate_integral = max(-self.parameters.integral_limit,
                                 min(self.parameters.integral_limit,
                                     self.integral + error * elapsed_s))
        output = (self.parameters.kp * error + self.parameters.ki * candidate_integral
                  + self.parameters.kd * derivative + feedforward)
        bounded = max(-self.parameters.output_limit, min(self.parameters.output_limit, output))
        if output == bounded or (output > bounded and error < 0) or (output < bounded and error > 0):
            self.integral = candidate_integral
        self.previous_error = error
        return bounded


@dataclass(frozen=True)
class RollerCommand:
    enabled: bool
    target_tube_angle_deg: float
    target_motor_angle_deg: float
    velocity_reference_mm_s: float
    acceleration_command_mm_s2: float


class RollerController:
    """Position -> velocity -> tilt controller with a measured-angle correction."""

    def __init__(self, position_pid: Pid, velocity_pid: Pid, angle_pid: Pid,
                 slope_bias: LinearTable, motor_by_tube_angle: LinearTable, *,
                 target_min_mm: float, target_max_mm: float, tube_angle_min_deg: float,
                 tube_angle_max_deg: float, motor_angle_min_deg: float,
                 motor_angle_max_deg: float, tilt_sign: float = 1.0,
                 feedforward_enabled: bool = False, feedforward_gain: float = 0.0,
                 feedforward_limit_mm_s2: float = 0.0) -> None:
        if target_min_mm >= target_max_mm or tube_angle_min_deg >= tube_angle_max_deg:
            raise ValueError("roller control limits are invalid")
        if motor_angle_min_deg >= motor_angle_max_deg:
            raise ValueError("motor soft limits are invalid")
        if tilt_sign not in {-1.0, 1.0}:
            raise ValueError("tilt_sign must be -1 or 1")
        if (not math.isfinite(feedforward_gain)
                or not math.isfinite(feedforward_limit_mm_s2)
                or feedforward_gain < 0 or feedforward_limit_mm_s2 < 0):
            raise ValueError("acceleration feedforward parameters are invalid")
        self.position_pid = position_pid
        self.velocity_pid = velocity_pid
        self.angle_pid = angle_pid
        self.slope_bias = slope_bias
        self.motor_by_tube_angle = motor_by_tube_angle
        self.target_min_mm = target_min_mm
        self.target_max_mm = target_max_mm
        self.tube_angle_min_deg = tube_angle_min_deg
        self.tube_angle_max_deg = tube_angle_max_deg
        self.motor_angle_min_deg = motor_angle_min_deg
        self.motor_angle_max_deg = motor_angle_max_deg
        self.tilt_sign = tilt_sign
        self.feedforward_enabled = bool(feedforward_enabled)
        self.feedforward_gain = feedforward_gain
        self.feedforward_limit_mm_s2 = feedforward_limit_mm_s2

    def reset(self) -> None:
        self.position_pid.reset()
        self.velocity_pid.reset()
        self.angle_pid.reset()

    def step(self, target_mm: float, ball: BallState, tube_angle_deg: float,
             elapsed_s: float, *, target_acceleration_mm_s2: float = 0.0,
             vehicle_acceleration_mm_s2: float = 0.0) -> RollerCommand:
        if not self.target_min_mm <= target_mm <= self.target_max_mm:
            raise ValueError("target_mm is outside the calibrated tube range")
        position_error = target_mm - ball.position_mm
        velocity_reference = self.position_pid.update(position_error, elapsed_s)
        if not math.isfinite(vehicle_acceleration_mm_s2):
            raise ValueError("vehicle acceleration must be finite")
        feedforward = target_acceleration_mm_s2
        if self.feedforward_enabled:
            feedforward += self.feedforward_gain * vehicle_acceleration_mm_s2
            if self.feedforward_limit_mm_s2 > 0:
                feedforward = max(-self.feedforward_limit_mm_s2,
                                  min(self.feedforward_limit_mm_s2, feedforward))
        acceleration = self.velocity_pid.update(
            velocity_reference - ball.velocity_mm_s, elapsed_s, feedforward)
        ratio = max(-0.20, min(0.20, acceleration / 9_806.65))
        dynamic_tilt = self.tilt_sign * math.degrees(math.asin(ratio))
        desired_tube_angle = self.slope_bias.at(ball.position_mm) + dynamic_tilt
        desired_tube_angle = max(self.tube_angle_min_deg,
                                 min(self.tube_angle_max_deg, desired_tube_angle))
        motor_feedforward = self.motor_by_tube_angle.at(desired_tube_angle)
        motor_correction = self.angle_pid.update(desired_tube_angle - tube_angle_deg, elapsed_s)
        motor_angle = max(self.motor_angle_min_deg,
                          min(self.motor_angle_max_deg, motor_feedforward + motor_correction))
        return RollerCommand(True, desired_tube_angle, motor_angle,
                             velocity_reference, acceleration)
