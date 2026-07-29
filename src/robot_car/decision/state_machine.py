"""Fail-safe high-level state machine independent of model implementations."""

from __future__ import annotations

import time
from enum import Enum
from typing import Any, Dict, Optional

from robot_car.perception.events import VisionEvent
from robot_car.protocol.messages import MotionMode

from .motion_target import MotionTarget
from .rules import action_for


class VehicleState(str, Enum):
    BOOT = "BOOT"
    IDLE = "IDLE"
    LINE_FOLLOW = "LINE_FOLLOW"
    VISION_ASSIST = "VISION_ASSIST"
    CAPTURE_TARGET_POLAR = "CAPTURE_TARGET_POLAR"
    BALANCE_ROLLER = "BALANCE_ROLLER"
    FAILSAFE = "FAILSAFE"
    E_STOP = "E_STOP"
    FAULT = "FAULT"


class VehicleStateMachine:
    def __init__(self, config: Dict[str, Any]) -> None:
        self.config = config
        self.state = VehicleState.BOOT
        self.last_vision_ms: Optional[int] = None
        self.vision_watch_started_ms: Optional[int] = None
        self.stop_until_ms = 0
        self.fault = ""
        self.failsafe_reason = ""

    def start(self) -> None:
        initial = str(self.config.get("initial_mode", "IDLE"))
        if initial == "LINE_FOLLOW":
            self.state = VehicleState.LINE_FOLLOW
        elif initial == "BALANCE_ROLLER":
            self.state = VehicleState.BALANCE_ROLLER
        else:
            self.state = VehicleState.IDLE

    def handle_event(self, event: VisionEvent, now_ms: Optional[int] = None) -> None:
        now = monotonic_ms() if now_ms is None else now_ms
        if event.is_expired(now):
            return
        self.last_vision_ms = now
        if self.state == VehicleState.FAILSAFE and self.failsafe_reason == "vision_timeout":
            self.start()
            self.failsafe_reason = ""
        action = action_for(event)
        if action == "STOP":
            self.stop_until_ms = now + int(self.config.get("stop_hold_ms", 1000))
            self.state = VehicleState.VISION_ASSIST
        elif action == "SLOW_DOWN":
            self.state = VehicleState.VISION_ASSIST
        elif action == "INTERSECTION":
            self.state = VehicleState.VISION_ASSIST
        elif event.event_type == "BALL_TARGET" and self.config.get("capture", {}).get("enabled"):
            self.state = VehicleState.CAPTURE_TARGET_POLAR
        elif event.event_type == "BALL_BALANCE_STATE" and self.config.get("balance", {}).get("enabled"):
            self.state = VehicleState.BALANCE_ROLLER
        elif event.event_type == "CAPTURE_CANCEL" and self.state == VehicleState.CAPTURE_TARGET_POLAR:
            self.start()

    def update_safety(self, link_ok: bool, estop: bool = False, fault: str = "", now_ms: Optional[int] = None) -> None:
        now = monotonic_ms() if now_ms is None else now_ms
        if estop:
            self.state = VehicleState.E_STOP
            return
        if fault:
            self.fault = fault
            self.state = VehicleState.FAULT
            return
        if not link_ok:
            self.failsafe_reason = "link_timeout"
            self.state = VehicleState.FAILSAFE
            return
        timeout = int(self.config.get("vision_timeout_ms", 500))
        if self.last_vision_ms is None:
            if self.vision_watch_started_ms is None:
                self.vision_watch_started_ms = now
            elif now - self.vision_watch_started_ms > timeout:
                self.failsafe_reason = "vision_timeout"
                self.state = VehicleState.FAILSAFE
        elif now - self.last_vision_ms > timeout:
            self.failsafe_reason = "vision_timeout"
            self.state = VehicleState.FAILSAFE

    def target(self, now_ms: Optional[int] = None) -> MotionTarget:
        now = monotonic_ms() if now_ms is None else now_ms
        valid_for = int(self.config.get("default_valid_for_ms", 200))
        control_enabled = bool(self.config.get("control_enabled", False))
        if self.state in {VehicleState.BOOT, VehicleState.IDLE, VehicleState.FAILSAFE,
                          VehicleState.E_STOP, VehicleState.FAULT}:
            return MotionTarget(mode=MotionMode.DISABLED, enabled=False, valid_for_ms=valid_for)
        if now < self.stop_until_ms:
            return MotionTarget(mode=MotionMode.VISION_ASSIST, enabled=False, valid_for_ms=valid_for)
        if self.state == VehicleState.CAPTURE_TARGET_POLAR:
            return MotionTarget(mode=MotionMode.CAPTURE_TARGET_POLAR, enabled=control_enabled,
                                valid_for_ms=valid_for)
        if self.state == VehicleState.BALANCE_ROLLER:
            return MotionTarget(mode=MotionMode.BALANCE_ROLLER, enabled=control_enabled,
                                valid_for_ms=valid_for)
        if self.state == VehicleState.VISION_ASSIST:
            return MotionTarget(mode=MotionMode.VISION_ASSIST, enabled=control_enabled,
                                valid_for_ms=valid_for)
        return MotionTarget(mode=MotionMode.LINE_FOLLOW, enabled=control_enabled,
                            valid_for_ms=valid_for)


def monotonic_ms() -> int:
    return time.monotonic_ns() // 1_000_000
