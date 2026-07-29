"""Validated roller-balance state shared by the vision and UART layers."""

from __future__ import annotations

from dataclasses import dataclass

from robot_car.perception.events import VisionEvent


@dataclass(frozen=True)
class BalanceState:
    timestamp_monotonic_ms: int
    error_mm: int = 0
    valid_for_ms: int = 120
    valid: bool = False

    @classmethod
    def from_event(cls, event: VisionEvent, now_ms: int, valid_for_ms: int) -> "BalanceState":
        if event.event_type != "BALL_BALANCE_STATE":
            raise ValueError("balance state requires a BALL_BALANCE_STATE event")
        if event.is_expired(now_ms):
            raise ValueError("cannot create balance state from an expired event")
        try:
            error_mm = int(event.payload["error_mm"])
        except (KeyError, TypeError, ValueError, OverflowError) as error:
            raise ValueError("BALL_BALANCE_STATE has an invalid error_mm") from error
        if not -0x8000 <= error_mm <= 0x7FFF:
            raise ValueError("BALL_BALANCE_STATE error_mm is out of int16 range")
        return cls(event.timestamp_monotonic_ms, error_mm=error_mm,
                   valid_for_ms=valid_for_ms, valid=True)

    def is_expired(self, now_ms: int) -> bool:
        return not self.valid or now_ms - self.timestamp_monotonic_ms > self.valid_for_ms
