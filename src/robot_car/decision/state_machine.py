"""Fail-safe high-level state machine independent of model implementations."""

from __future__ import annotations

import time
from enum import Enum
from typing import Any, Dict, Optional

from robot_car.perception.events import VisionEvent

from .motion_target import MotionTarget
from .rules import action_for


class VehicleState(str, Enum):
    BOOT = "BOOT"
    IDLE = "IDLE"
    LINE_FOLLOW = "LINE_FOLLOW"
    VISION_ASSIST = "VISION_ASSIST"
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
        self.speed_limit_override: Optional[int] = None
        self.fault = ""
        self.failsafe_reason = ""

    def start(self) -> None:
        initial = str(self.config.get("initial_mode", "IDLE"))
        self.state = VehicleState.LINE_FOLLOW if initial == "LINE_FOLLOW" else VehicleState.IDLE

    def handle_event(self, event: VisionEvent, now_ms: Optional[int] = None) -> None:
        now = monotonic_ms() if now_ms is None else now_ms
        if event.is_expired(now):
            return
        self.last_vision_ms = now
        if self.state == VehicleState.FAILSAFE and self.failsafe_reason == "vision_timeout":
            initial = str(self.config.get("initial_mode", "IDLE"))
            self.state = VehicleState.LINE_FOLLOW if initial == "LINE_FOLLOW" else VehicleState.IDLE
            self.failsafe_reason = ""
        action = action_for(event)
        if action == "STOP":
            self.stop_until_ms = now + int(self.config.get("stop_hold_ms", 1000))
            self.state = VehicleState.VISION_ASSIST
        elif action == "SLOW_DOWN":
            self.speed_limit_override = int(self.config.get("slow_speed_limit_mm_s", 100))
            self.state = VehicleState.VISION_ASSIST
        elif action == "INTERSECTION":
            self.state = VehicleState.VISION_ASSIST

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
            return MotionTarget(mode=self.state.value, enable=False, valid_for_ms=valid_for)
        if now < self.stop_until_ms:
            return MotionTarget(mode="VISION_ASSIST", enable=False, valid_for_ms=valid_for)
        speed = int(self.config.get("line_follow_speed_mm_s", self.config.get("default_speed_mm_s", 0)))
        limit = int(self.config.get("speed_limit_mm_s", 0))
        if self.speed_limit_override is not None:
            limit = min(limit, self.speed_limit_override) if limit > 0 else self.speed_limit_override
            speed = min(speed, limit)
        return MotionTarget(
            mode=self.state.value,
            enable=control_enabled,
            target_speed_mm_s=speed if control_enabled else 0,
            speed_limit_mm_s=limit if control_enabled else 0,
            valid_for_ms=valid_for,
        )


def monotonic_ms() -> int:
    return time.monotonic_ns() // 1_000_000
