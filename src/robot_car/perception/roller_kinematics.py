"""One-dimensional, timestamp-aware steel-ball motion estimation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class RollerKinematicState:
    """Filtered position, velocity and acceleration along the tube axis."""

    timestamp_monotonic_ms: int
    position_mm: float
    velocity_mm_s: float
    acceleration_mm_s2: float


class RollerKinematicsEstimator:
    """Smooth position measurements before exporting derivatives to the controller.

    This deliberately uses a small alpha-beta-gamma observer instead of raw second
    differences, whose acceleration output is dominated by detector pixel jitter.
    """

    def __init__(self, position_alpha: float = 0.65, velocity_alpha: float = 0.35,
                 acceleration_alpha: float = 0.20, max_gap_ms: int = 250) -> None:
        for label, value in (("position_alpha", position_alpha),
                             ("velocity_alpha", velocity_alpha),
                             ("acceleration_alpha", acceleration_alpha)):
            if not 0 < value <= 1:
                raise ValueError(f"{label} must be in (0, 1]")
        if max_gap_ms <= 0:
            raise ValueError("max_gap_ms must be positive")
        self.position_alpha = position_alpha
        self.velocity_alpha = velocity_alpha
        self.acceleration_alpha = acceleration_alpha
        self.max_gap_ms = max_gap_ms
        self._state: Optional[RollerKinematicState] = None

    def reset(self) -> None:
        self._state = None

    def update(self, timestamp_ms: int, measured_position_mm: float) -> RollerKinematicState:
        previous = self._state
        if previous is None or timestamp_ms <= previous.timestamp_monotonic_ms:
            self._state = RollerKinematicState(timestamp_ms, measured_position_mm, 0.0, 0.0)
            return self._state
        gap_ms = timestamp_ms - previous.timestamp_monotonic_ms
        if gap_ms > self.max_gap_ms:
            self._state = RollerKinematicState(timestamp_ms, measured_position_mm, 0.0, 0.0)
            return self._state

        dt = gap_ms / 1000.0
        predicted_position = (previous.position_mm + previous.velocity_mm_s * dt
                              + 0.5 * previous.acceleration_mm_s2 * dt * dt)
        position = predicted_position + self.position_alpha * (measured_position_mm - predicted_position)
        raw_velocity = (position - previous.position_mm) / dt
        velocity = previous.velocity_mm_s + self.velocity_alpha * (raw_velocity - previous.velocity_mm_s)
        raw_acceleration = (velocity - previous.velocity_mm_s) / dt
        acceleration = (previous.acceleration_mm_s2 + self.acceleration_alpha
                        * (raw_acceleration - previous.acceleration_mm_s2))
        self._state = RollerKinematicState(timestamp_ms, position, velocity, acceleration)
        return self._state
