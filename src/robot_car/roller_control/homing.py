"""One-shot, absolute-zero homing for the Y42 roller actuator."""

from __future__ import annotations

import time
from pathlib import Path
from threading import Event
from typing import Callable

from .config import load_roller_control
from .y42_actuator import Y42Actuator


class HomeCancelled(RuntimeError):
    """Raised after sending the Y42 command that interrupts absolute homing."""


def home_from_config_file(path: str | Path, timeout_ms: int,
                          *, hold_enabled: bool = True, cancel_event: Event | None = None,
                          show_tx: bool = False,
                          actuator_factory: Callable[..., Y42Actuator] = Y42Actuator) -> float:
    """Home the configured Y42 to its stored absolute coordinate zero.

    This never writes a new encoder zero.  It only uses Y42 command 9A with
    HomeMode 04, then verifies the completion state through command 3B.
    """
    if not 1 <= int(timeout_ms) <= 0xFFFFFFFF:
        raise ValueError("roller home timeout_ms must be positive")
    config = load_roller_control(path)
    if not bool(config.get("enabled", False)):
        raise RuntimeError("roller_control.yaml enabled must be true before homing")
    motor = config["motor"]
    actuator = actuator_factory(
        interface=str(motor["can_interface"]), address=int(motor["address"]),
        firmware=str(motor.get("firmware", "x")),
        packet_gap_ms=float(motor.get("packet_gap_ms", 3.0)),
        pulses_per_revolution=int(motor.get("pulses_per_revolution", 3200)),
        soft_limit_min_deg=float(motor["soft_limit_min_deg"]),
        soft_limit_max_deg=float(motor["soft_limit_max_deg"]),
        show_tx=show_tx,
    )
    deadline = time.monotonic() + int(timeout_ms) / 1000.0
    completed = False
    actuator.open()
    try:
        actuator.enable()
        # 0x02 means Y42 accepted 9A. Completion remains governed by 3B,
        # because a 9F frame may be disabled by the motor response setting.
        actuator.home_absolute_zero(confirm=True)
        while time.monotonic() < deadline:
            if cancel_event is not None and cancel_event.is_set():
                actuator.cancel_home()
                raise HomeCancelled("Y42 absolute home cancelled")
            remaining = max(0.01, deadline - time.monotonic())
            status = actuator.read_home_status(timeout_s=min(2.0, remaining))
            if status & 0x08:
                raise RuntimeError("Y42 absolute home failed (status 0x%02X)" % status)
            if not status & 0x04:
                position = actuator.read_position_deg(timeout_s=min(2.0, remaining))
                completed = True
                return position
            time.sleep(0.05)
        raise RuntimeError("Y42 absolute home timed out")
    finally:
        # Preserve holding torque only after a verified successful home.  All
        # failures and cancellation paths still stop and disable the motor.
        actuator.close(disable=not (completed and hold_enabled))
