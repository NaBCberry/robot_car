"""Ensure only one high-level control chain owns vehicle motion at a time."""

from __future__ import annotations

import logging
from typing import Any, Dict, Optional

from robot_car.perception.events import VisionEvent
from robot_car.protocol.messages import MotionMode

from .capture_target import CaptureTarget
from .balance_state import BalanceState
from .motion_target import MotionTarget


LOG = logging.getLogger(__name__)


class ControlArbiter:
    def __init__(self, vehicle_config: Dict[str, Any]) -> None:
        capture = vehicle_config.get("capture", {})
        self.capture_config = capture
        self.capture_enabled = bool(capture.get("enabled", False))
        self.capture_armed = False
        self.latest_target: Optional[CaptureTarget] = None
        balance = vehicle_config.get("balance", {})
        self.balance_config = balance
        self.balance_enabled = bool(balance.get("enabled", False))
        self.balance_output_only = bool(balance.get("output_only", False))
        self.latest_balance: Optional[BalanceState] = None

    def handle_event(self, event: VisionEvent, now_ms: int) -> None:
        if event.is_expired(now_ms):
            return
        if event.event_type == "CAPTURE_ARM" and self.capture_enabled:
            self.capture_armed = True
            return
        if event.event_type == "CAPTURE_CANCEL" and self.capture_enabled:
            self.capture_armed = False
            self.latest_target = None
            return
        if event.event_type == "BALL_TARGET" and self.capture_enabled:
            valid_for_ms = int(self.capture_config.get("target_timeout_ms", 200))
            try:
                self.latest_target = CaptureTarget.from_ball_event(event, now_ms, valid_for_ms,
                                                                    self.capture_armed)
            except ValueError as error:
                self.latest_target = None
                LOG.warning("discarded invalid steel-ball target: %s", error)
        if event.event_type == "BALL_BALANCE_STATE" and self.balance_enabled:
            valid_for_ms = int(self.balance_config.get("state_timeout_ms", 120))
            try:
                self.latest_balance = BalanceState.from_event(event, now_ms, valid_for_ms)
            except ValueError as error:
                self.latest_balance = None
                LOG.warning("discarded invalid roller-balance state: %s", error)

    def select(self, now_ms: int, fallback_motion: MotionTarget) -> MotionTarget:
        """Return the one semantic CMD_MOTION intent allowed for this cycle."""
        if (not self.capture_enabled
                or fallback_motion.mode != MotionMode.CAPTURE_TARGET_POLAR):
            if (not self.balance_enabled
                    or fallback_motion.mode != MotionMode.BALANCE_ROLLER):
                return fallback_motion
            if ((not fallback_motion.enabled and not self.balance_output_only)
                    or self.latest_balance is None
                    or self.latest_balance.is_expired(now_ms)):
                return MotionTarget(mode=MotionMode.BALANCE_ROLLER, enabled=False,
                                    valid_for_ms=fallback_motion.valid_for_ms)
            return MotionTarget(mode=MotionMode.BALANCE_ROLLER, enabled=fallback_motion.enabled,
                                valid_for_ms=self.latest_balance.valid_for_ms,
                                balance_state=self.latest_balance)
        if (not fallback_motion.enabled or self.latest_target is None
                or self.latest_target.is_expired(now_ms)):
            return MotionTarget(mode=MotionMode.CAPTURE_TARGET_POLAR, enabled=False,
                                valid_for_ms=fallback_motion.valid_for_ms)
        return MotionTarget(mode=MotionMode.CAPTURE_TARGET_POLAR, enabled=True,
                            valid_for_ms=self.latest_target.valid_for_ms,
                            capture_target=self.latest_target)

    def capture_result(self, telemetry: Dict[str, Any]) -> str:
        """Do not infer capture success when physical feedback is disabled."""
        feedback = self.capture_config.get("feedback", {})
        if not feedback.get("enabled", False):
            return "CAPTURE_ATTEMPTED" if telemetry.get("capture_attempted") else "UNKNOWN"
        return str(telemetry.get("capture_state", "UNKNOWN"))
