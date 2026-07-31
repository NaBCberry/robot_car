"""Shared action dispatcher for UART requests and the interactive TUI."""

from __future__ import annotations

import time
from dataclasses import dataclass
from enum import IntEnum
from typing import Any, Dict, Optional

from robot_car.perception.events import VisionEvent
from robot_car.protocol.messages import MotionMode


class ActionId(IntEnum):
    STOP = 0
    ROLLER_HOME = 1
    # Match the five numbered tasks in H.pdf so TUI/UART action IDs are
    # directly recognizable at the competition site.
    LINE_LAP_TO_A = 2
    ROLLER_SWEEP = 3
    LINE_TO_B_BALANCE_CENTER = 4
    LINE_LAP_BALANCE_CENTER = 5
    LINE_LAP_BALANCE_TARGET = 6


@dataclass(frozen=True)
class ActionSnapshot:
    action_id: Optional[int]
    status: str
    source: str
    phase: str
    started_ms: Optional[int]
    elapsed_ms: int
    target_mm: Optional[float]
    last_remote_action_id: Optional[int]
    last_action_id: Optional[int]
    ball_error_mm: Optional[float]
    reason: str

    def to_dict(self) -> Dict[str, Any]:
        return {
            "action_id": self.action_id,
            "status": self.status,
            "source": self.source,
            "phase": self.phase,
            "started_ms": self.started_ms,
            "elapsed_ms": self.elapsed_ms,
            "target_mm": self.target_mm,
            "last_remote_action_id": self.last_remote_action_id,
            "last_action_id": self.last_action_id,
            "ball_error_mm": self.ball_error_mm,
            "reason": self.reason,
        }


class ActionDispatcher:
    """Translate action IDs into semantic motion modes and balance targets."""

    def __init__(self, vehicle_config: Dict[str, Any]) -> None:
        self.config = vehicle_config
        self.active_action: Optional[ActionId] = None
        self.status = "IDLE"
        self.source = ""
        self.phase = ""
        self.started_ms: Optional[int] = None
        self.deadline_ms: Optional[int] = None
        self.target_mm: Optional[float] = None
        self.last_remote_action_id: Optional[int] = None
        self.last_action_id: Optional[int] = None
        self.ball_error_mm: Optional[float] = None
        self.reason = ""
        self._phase_started_ms: Optional[int] = None
        self._stable_since_ms: Optional[int] = None

    def request(self, action_id: int, parameters: Optional[Dict[str, Any]] = None,
                *, source: str = "tui", now_ms: Optional[int] = None,
                current_ball_error_mm: Optional[float] = None) -> None:
        now = _now_ms() if now_ms is None else now_ms
        try:
            action = ActionId(int(action_id))
        except (TypeError, ValueError) as error:
            raise ValueError(f"unknown action_id: {action_id}") from error
        if not bool(self.config.get("actions", {}).get("enabled", True)):
            raise RuntimeError("vehicle actions are disabled")
        parameters = parameters or {}
        self.last_remote_action_id = int(action) if source == "uart" else self.last_remote_action_id
        if action == ActionId.STOP:
            self.stop("requested", now)
            return
        self.active_action = action
        self.last_action_id = int(action)
        self.status = "RUNNING"
        self.source = source
        self.started_ms = now
        self._phase_started_ms = now
        self._stable_since_ms = None
        self.reason = ""
        self.target_mm = None
        if action == ActionId.ROLLER_HOME:
            self.phase = "MOTOR_HOME"
            self.deadline_ms = now + int(parameters.get("timeout_ms", 30000))
        elif action == ActionId.ROLLER_SWEEP:
            self.phase = "TO_POSITIVE"
            self.deadline_ms = now + int(parameters.get("timeout_ms", 5000))
            self.target_mm = float(parameters.get("positive_mm", 50.0))
        elif action == ActionId.LINE_LAP_TO_A:
            self.phase = "LINE_LAP"
            self.deadline_ms = now + int(parameters.get("timeout_ms", 20000))
        elif action == ActionId.LINE_TO_B_BALANCE_CENTER:
            self.phase = "TO_B"
            self.deadline_ms = now + int(parameters.get("timeout_ms", 8000))
            self.target_mm = 0.0
        elif action == ActionId.LINE_LAP_BALANCE_CENTER:
            self.phase = "LAP_TO_A"
            self.deadline_ms = now + int(parameters.get("timeout_ms", 30000))
            self.target_mm = 0.0
        elif action == ActionId.LINE_LAP_BALANCE_TARGET:
            supplied = parameters.get("target_mm", current_ball_error_mm)
            if supplied is None:
                raise ValueError("action 5 requires target_mm or a current ball measurement")
            self.phase = "LAP_TO_A_TARGET"
            self.deadline_ms = now + int(parameters.get("timeout_ms", 30000))
            self.target_mm = float(supplied)
        if self.deadline_ms <= now:
            raise ValueError("action timeout must be positive")

    def stop(self, reason: str = "stopped", now_ms: Optional[int] = None) -> None:
        self.status = "STOPPED"
        self.reason = reason
        self.active_action = None
        self.phase = ""
        self.deadline_ms = None
        self.target_mm = None
        self._stable_since_ms = None

    def handle_event(self, event: VisionEvent, now_ms: Optional[int] = None) -> None:
        now = _now_ms() if now_ms is None else now_ms
        if event.event_type == "BALL_BALANCE_STATE":
            try:
                self.ball_error_mm = float(event.payload["error_mm"])
            except (KeyError, TypeError, ValueError):
                pass
        if self.active_action is None or event.is_expired(now):
            return
        if event.event_type in {"CHECKPOINT_A", "LAP_COMPLETE"} and self.active_action in {
                ActionId.LINE_LAP_TO_A, ActionId.LINE_LAP_BALANCE_CENTER,
                ActionId.LINE_LAP_BALANCE_TARGET}:
            self.complete("checkpoint_a", now)
        elif event.event_type == "CHECKPOINT_B" and self.active_action == ActionId.LINE_TO_B_BALANCE_CENTER:
            self.complete("checkpoint_b", now)

    def update(self, now_ms: Optional[int] = None) -> None:
        now = _now_ms() if now_ms is None else now_ms
        if self.active_action is None:
            return
        if self.deadline_ms is not None and now >= self.deadline_ms:
            self.status = "FAILED"
            self.reason = "timeout"
            self.active_action = None
            return
        if self.active_action != ActionId.ROLLER_SWEEP:
            return
        error = self.ball_error_mm
        if error is None:
            return
        if self.phase == "TO_POSITIVE" and abs(error - 50.0) <= 10.0:
            self.phase = "RETURN_CENTER"
            self.target_mm = 0.0
            self._phase_started_ms = now
        elif self.phase == "RETURN_CENTER" and abs(error) <= 10.0:
            self.phase = "TO_NEGATIVE"
            self.target_mm = -50.0
            self._phase_started_ms = now
        elif self.phase == "TO_NEGATIVE" and abs(error + 50.0) <= 10.0:
            if self._stable_since_ms is None:
                self._stable_since_ms = now
            elif now - self._stable_since_ms >= 250:
                self.complete("negative_target_stable", now)
        else:
            self._stable_since_ms = None

    def handle_telemetry(self, telemetry: Dict[str, Any], now_ms: Optional[int] = None) -> None:
        """Consume optional M0 checkpoint telemetry without coupling to CAN details."""
        checkpoint = telemetry.get("checkpoint", telemetry.get("line_checkpoint"))
        if not isinstance(checkpoint, str):
            return
        now = _now_ms() if now_ms is None else now_ms
        checkpoint = checkpoint.upper()
        if checkpoint in {"A", "LAP_COMPLETE"} and self.active_action in {
                ActionId.LINE_LAP_TO_A, ActionId.LINE_LAP_BALANCE_CENTER,
                ActionId.LINE_LAP_BALANCE_TARGET}:
            self.complete("checkpoint_a", now)
        elif checkpoint == "B" and self.active_action == ActionId.LINE_TO_B_BALANCE_CENTER:
            self.complete("checkpoint_b", now)

    def complete(self, reason: str, now_ms: Optional[int] = None) -> None:
        now = _now_ms() if now_ms is None else now_ms
        self.status = "COMPLETE"
        self.reason = reason
        self.active_action = None
        self.phase = ""
        self.deadline_ms = None

    def fail(self, reason: str, now_ms: Optional[int] = None) -> None:
        """Finish the active action as failed without leaving a motion request live."""
        self.status = "FAILED"
        self.reason = reason
        self.active_action = None
        self.phase = ""
        self.deadline_ms = None
        self.target_mm = None
        self._stable_since_ms = None

    def motion_mode(self) -> Optional[MotionMode]:
        if self.status != "RUNNING" or self.active_action is None:
            return None
        if self.active_action == ActionId.ROLLER_HOME:
            return None
        if self.active_action == ActionId.LINE_LAP_TO_A:
            return MotionMode.LINE_FOLLOW
        return MotionMode.BALANCE_ROLLER

    def snapshot(self, now_ms: Optional[int] = None) -> ActionSnapshot:
        now = _now_ms() if now_ms is None else now_ms
        elapsed = 0 if self.started_ms is None else max(0, now - self.started_ms)
        return ActionSnapshot(None if self.active_action is None else int(self.active_action),
                              self.status, self.source, self.phase, self.started_ms, elapsed,
                              self.target_mm, self.last_remote_action_id, self.last_action_id,
                              self.ball_error_mm, self.reason)


def _now_ms() -> int:
    return time.monotonic_ns() // 1_000_000
