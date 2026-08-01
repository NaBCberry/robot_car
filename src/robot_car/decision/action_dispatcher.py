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
    target_revision: int
    request_id: Optional[int]
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
            "target_revision": self.target_revision,
            "request_id": self.request_id,
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
        self.saved_target_mm: Optional[float] = None
        self.target_revision = 0
        self.request_id: Optional[int] = None
        self._target_capture_after_ms: Optional[int] = None
        self.last_remote_action_id: Optional[int] = None
        self.last_action_id: Optional[int] = None
        self.ball_error_mm: Optional[float] = None
        self.reason = ""
        self._phase_started_ms: Optional[int] = None
        self._stable_since_ms: Optional[int] = None

    def request(self, action_id: int, parameters: Optional[Dict[str, Any]] = None,
                *, source: str = "tui", now_ms: Optional[int] = None,
                current_ball_error_mm: Optional[float] = None,
                request_id: Optional[int] = None) -> None:
        now = _now_ms() if now_ms is None else now_ms
        try:
            action = ActionId(int(action_id))
        except (TypeError, ValueError) as error:
            raise ValueError(f"unknown action_id: {action_id}") from error
        if not bool(self.config.get("actions", {}).get("enabled", True)):
            raise RuntimeError("vehicle actions are disabled")
        parameters = parameters or {}
        if action == ActionId.LINE_LAP_BALANCE_TARGET:
            operation = parameters.get("operation")
            if operation not in {"set", "run"}:
                raise ValueError("action 6 operation must be 'set' or 'run'")
            if operation == "run":
                if self.saved_target_mm is None:
                    raise ValueError("action 6 target_not_set")
                try:
                    revision = int(parameters["target_revision"])
                except (KeyError, TypeError, ValueError) as error:
                    raise ValueError("action 6 run requires target_revision") from error
                if revision != self.target_revision:
                    raise ValueError("action 6 target_revision_mismatch")
        self.last_remote_action_id = int(action) if source == "uart" else self.last_remote_action_id
        self.request_id = request_id if source == "uart" else None
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
        default_timeout_ms = self._default_timeout_ms()
        if action == ActionId.ROLLER_HOME:
            self.phase = "MOTOR_HOME"
            self.deadline_ms = now + int(parameters.get("timeout_ms", default_timeout_ms))
        elif action == ActionId.ROLLER_SWEEP:
            self.phase = "TO_POSITIVE"
            self.deadline_ms = now + int(parameters.get("timeout_ms", 5000))
            self.target_mm = float(parameters.get("positive_mm", 50.0))
        elif action == ActionId.LINE_LAP_TO_A:
            self.phase = "LINE_LAP"
            self.deadline_ms = now + int(parameters.get("timeout_ms", default_timeout_ms))
        elif action == ActionId.LINE_TO_B_BALANCE_CENTER:
            self.phase = "TO_B"
            self.deadline_ms = now + int(parameters.get("timeout_ms", default_timeout_ms))
            self.target_mm = 0.0
        elif action == ActionId.LINE_LAP_BALANCE_CENTER:
            self.phase = "LAP_TO_A"
            self.deadline_ms = now + int(parameters.get("timeout_ms", default_timeout_ms))
            self.target_mm = 0.0
        elif action == ActionId.LINE_LAP_BALANCE_TARGET:
            operation = parameters.get("operation")
            if operation == "set":
                self.phase = "TARGET_CAPTURE"
                self.deadline_ms = now + int(parameters.get("capture_timeout_ms", 500))
                self.target_mm = self.saved_target_mm
                self._target_capture_after_ms = now
            elif operation == "run":
                self.phase = "LAP_TO_A_TARGET"
                self.deadline_ms = now + int(parameters.get("timeout_ms", default_timeout_ms))
                self.target_mm = self.saved_target_mm
        if self.deadline_ms <= now:
            raise ValueError("action timeout must be positive")

    def _default_timeout_ms(self) -> int:
        return int(self.config.get("actions", {}).get("default_timeout_ms", 300000))

    def stop(self, reason: str = "stopped", now_ms: Optional[int] = None) -> None:
        self.status = "STOPPED"
        self.reason = reason
        self.active_action = None
        self.phase = ""
        self.deadline_ms = None
        self.target_mm = None
        self._target_capture_after_ms = None
        self._stable_since_ms = None

    def handle_event(self, event: VisionEvent, now_ms: Optional[int] = None) -> None:
        now = _now_ms() if now_ms is None else now_ms
        if event.event_type == "BALL_BALANCE_STATE":
            measurement: Optional[float] = None
            try:
                measurement = float(event.payload["error_mm"])
                self.ball_error_mm = measurement
            except (KeyError, TypeError, ValueError):
                pass
            if (self.active_action == ActionId.LINE_LAP_BALANCE_TARGET
                    and self.phase == "TARGET_CAPTURE"
                    and self._target_capture_after_ms is not None
                    and event.timestamp_monotonic_ms >= self._target_capture_after_ms
                    and not event.is_expired(now)
                    and measurement is not None):
                self._save_target(measurement, now)
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
            self.fail("target_capture_timeout" if self.phase == "TARGET_CAPTURE" else "timeout")
            return
        if self.active_action != ActionId.ROLLER_SWEEP or self.phase.startswith("DIRECT_"):
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
        self._target_capture_after_ms = None

    def fail(self, reason: str, now_ms: Optional[int] = None) -> None:
        """Finish the active action as failed without leaving a motion request live."""
        self.status = "FAILED"
        self.reason = reason
        self.active_action = None
        self.phase = ""
        self.deadline_ms = None
        self.target_mm = None
        self._stable_since_ms = None
        self._target_capture_after_ms = None

    def _save_target(self, value: Optional[float], now_ms: int) -> None:
        if value is None:
            return
        target = round(float(value), 1)
        if not -1000.0 <= target <= 1000.0:
            self.fail("target_out_of_range", now_ms)
            return
        self.saved_target_mm = target
        self.target_mm = target
        self.target_revision = 1 if self.target_revision >= 0xFFFF else self.target_revision + 1
        self.complete("target_saved", now_ms)

    def set_direct_roller_progress(self, phase: str, target_mm: float) -> None:
        """Expose the direct-CAN task-3 sequence without running M0 semantics."""
        if self.active_action != ActionId.ROLLER_SWEEP or self.status != "RUNNING":
            return
        self.phase = f"DIRECT_{phase}"
        self.target_mm = target_mm

    def motion_mode(self) -> Optional[MotionMode]:
        if self.status != "RUNNING" or self.active_action is None:
            return None
        if self.active_action == ActionId.ROLLER_HOME or self.phase == "TARGET_CAPTURE":
            return None
        if self.active_action in {
                ActionId.LINE_LAP_TO_A,
                ActionId.LINE_TO_B_BALANCE_CENTER,
                ActionId.LINE_LAP_BALANCE_CENTER,
                ActionId.LINE_LAP_BALANCE_TARGET}:
            return MotionMode.LINE_FOLLOW
        return MotionMode.BALANCE_ROLLER

    def snapshot(self, now_ms: Optional[int] = None) -> ActionSnapshot:
        now = _now_ms() if now_ms is None else now_ms
        elapsed = 0 if self.started_ms is None else max(0, now - self.started_ms)
        return ActionSnapshot(None if self.active_action is None else int(self.active_action),
                              self.status, self.source, self.phase, self.started_ms, elapsed,
                              self.target_mm, self.target_revision, self.request_id,
                              self.last_remote_action_id, self.last_action_id,
                              self.ball_error_mm, self.reason)


def _now_ms() -> int:
    return time.monotonic_ns() // 1_000_000
