"""Capture-target representation shared by decision and vehicle-link layers."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict

from robot_car.perception.events import VisionEvent


class ControlMode(str, Enum):
    RDK_MOTION_TARGET = "RDK_MOTION_TARGET"
    MCU_TARGET_SERVO = "MCU_TARGET_SERVO"


@dataclass(frozen=True)
class CaptureTarget:
    timestamp_monotonic_ms: int
    track_id: int = 0
    bearing_mdeg: int = 0
    range_mm: int = 0
    confidence_permille: int = 0
    measurement_age_ms: int = 0
    valid_for_ms: int = 200
    capture_armed: bool = False
    target_valid: bool = False

    @classmethod
    def from_ball_event(cls, event: VisionEvent, now_ms: int, valid_for_ms: int,
                        capture_armed: bool) -> "CaptureTarget":
        if event.event_type != "BALL_TARGET":
            raise ValueError("capture target requires a BALL_TARGET event")
        if event.is_expired(now_ms):
            raise ValueError("cannot create capture target from expired event")
        payload = event.payload
        try:
            track_id = int(payload["track_id"])
            bearing = int(payload["bearing_mdeg"])
            range_mm = int(payload["range_mm"])
            confidence = max(0, min(1000, int(round(event.confidence * 1000))))
        except (KeyError, TypeError, ValueError, OverflowError) as error:
            raise ValueError("BALL_TARGET has invalid capture coordinates") from error
        if (not 0 <= track_id <= 0xFFFF or not -0x80000000 <= bearing <= 0x7FFFFFFF
                or not 0 <= range_mm <= 0x7FFFFFFF):
            raise ValueError("BALL_TARGET capture coordinates are out of range")
        age_ms = max(0, min(0xFFFF, now_ms - event.timestamp_monotonic_ms))
        return cls(event.timestamp_monotonic_ms, track_id, bearing, range_mm, confidence,
                   age_ms, valid_for_ms, capture_armed, True)

    def is_expired(self, now_ms: int) -> bool:
        return not self.target_valid or now_ms - self.timestamp_monotonic_ms > self.valid_for_ms

    def safe(self) -> "CaptureTarget":
        return CaptureTarget(self.timestamp_monotonic_ms, valid_for_ms=self.valid_for_ms)
