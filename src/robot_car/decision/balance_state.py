"""Validated roller-balance state shared by the vision and UART layers."""

from __future__ import annotations

from dataclasses import dataclass

from robot_car.perception.events import VisionEvent


@dataclass(frozen=True)
class BalanceState:
    timestamp_monotonic_ms: int
    track_id: int = 0
    position_mm: int = 0
    target_mm: int = 0
    error_mm: int = 0
    velocity_mm_s: int = 0
    acceleration_mm_s2: int = 0
    confidence_permille: int = 0
    measurement_age_ms: int = 0
    valid_for_ms: int = 120
    valid: bool = False

    @classmethod
    def from_event(cls, event: VisionEvent, now_ms: int, valid_for_ms: int) -> "BalanceState":
        if event.event_type != "BALL_BALANCE_STATE":
            raise ValueError("balance state requires a BALL_BALANCE_STATE event")
        if event.is_expired(now_ms):
            raise ValueError("cannot create balance state from an expired event")
        try:
            values = {
                "track_id": int(event.payload["track_id"]),
                "position_mm": int(event.payload["position_mm"]),
                "target_mm": int(event.payload["target_mm"]),
                "error_mm": int(event.payload["error_mm"]),
                "velocity_mm_s": int(event.payload["velocity_mm_s"]),
                "acceleration_mm_s2": int(event.payload["acceleration_mm_s2"]),
            }
        except (KeyError, TypeError, ValueError, OverflowError) as error:
            raise ValueError("BALL_BALANCE_STATE has invalid kinematic fields") from error
        if not 0 <= values["track_id"] <= 0xFFFF:
            raise ValueError("BALL_BALANCE_STATE track_id is out of range")
        if any(not -0x8000 <= values[key] <= 0x7FFF for key in (
                "position_mm", "target_mm", "error_mm", "velocity_mm_s", "acceleration_mm_s2")):
            raise ValueError("BALL_BALANCE_STATE kinematic value is out of int16 range")
        if values["error_mm"] != values["position_mm"] - values["target_mm"]:
            raise ValueError("BALL_BALANCE_STATE error_mm is inconsistent")
        confidence = max(0, min(1000, int(round(event.confidence * 1000))))
        age_ms = max(0, min(0xFFFF, now_ms - event.timestamp_monotonic_ms))
        return cls(event.timestamp_monotonic_ms, confidence_permille=confidence,
                   measurement_age_ms=age_ms, valid_for_ms=valid_for_ms, valid=True, **values)

    def is_expired(self, now_ms: int) -> bool:
        return not self.valid or now_ms - self.timestamp_monotonic_ms > self.valid_for_ms
