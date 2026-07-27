"""Ensure only one high-level control chain owns vehicle motion at a time."""

from __future__ import annotations

import logging
from typing import Any, Dict, Optional, Tuple

from robot_car.perception.events import VisionEvent

from .capture_target import CaptureTarget, ControlMode
from .motion_target import MotionTarget


LOG = logging.getLogger(__name__)


class ControlArbiter:
    def __init__(self, vehicle_config: Dict[str, Any]) -> None:
        capture = vehicle_config.get("capture", {})
        self.capture_config = capture
        self.capture_enabled = bool(capture.get("enabled", False))
        self.mode = ControlMode(str(capture.get("control_mode", "RDK_MOTION_TARGET")))
        self.capture_armed = False
        self.latest_target: Optional[CaptureTarget] = None
        self._pending_capture_event: Optional[str] = None

    def handle_event(self, event: VisionEvent, now_ms: int) -> None:
        if event.is_expired(now_ms):
            return
        if event.event_type == "CAPTURE_ARM" and self.capture_enabled:
            self.capture_armed = True
            self._pending_capture_event = "CAPTURE_ARM"
            return
        if event.event_type == "CAPTURE_CANCEL" and self.capture_enabled:
            self.capture_armed = False
            self.latest_target = None
            self._pending_capture_event = "CAPTURE_CANCEL"
            return
        if (event.event_type == "BALL_TARGET" and self.capture_enabled
                and self.mode == ControlMode.MCU_TARGET_SERVO):
            valid_for_ms = int(self.capture_config.get("target_timeout_ms", 200))
            try:
                self.latest_target = CaptureTarget.from_ball_event(event, now_ms, valid_for_ms,
                                                                    self.capture_armed)
            except ValueError as error:
                self.latest_target = None
                LOG.warning("discarded invalid steel-ball target: %s", error)

    def select(self, now_ms: int, fallback_motion: MotionTarget) -> Tuple[Optional[MotionTarget],
                                                                            Optional[CaptureTarget]]:
        """Return exactly one command class, keeping a disabled command explicit."""
        if not self.capture_enabled or self.mode == ControlMode.RDK_MOTION_TARGET:
            return fallback_motion, None
        if not fallback_motion.enable or self.latest_target is None or self.latest_target.is_expired(now_ms):
            valid_for_ms = int(self.capture_config.get("target_timeout_ms", 200))
            return None, CaptureTarget(now_ms, valid_for_ms=valid_for_ms)
        return None, self.latest_target

    def consume_capture_event(self) -> Optional[str]:
        event_type = self._pending_capture_event
        self._pending_capture_event = None
        return event_type

    def capture_result(self, telemetry: Dict[str, Any]) -> str:
        """Do not infer capture success when physical feedback is disabled."""
        feedback = self.capture_config.get("feedback", {})
        if not feedback.get("enabled", False):
            return "CAPTURE_ATTEMPTED" if telemetry.get("capture_attempted") else "UNKNOWN"
        return str(telemetry.get("capture_state", "UNKNOWN"))
