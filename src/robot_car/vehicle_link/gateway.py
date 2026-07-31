"""Protocol-aware exclusive vehicle link gateway."""

from __future__ import annotations

import logging
import threading
import time
from collections import deque
from queue import Empty, Queue
from typing import Any, Dict, Optional

from robot_car.decision.motion_target import MotionTarget
from robot_car.protocol.framing import FrameDecoder, encode_frame
from robot_car.protocol.messages import (ACK_STATUS_ACCEPTED, ACK_STATUS_DUPLICATE,
                                         ACK_STATUS_INVALID, HEARTBEAT_STRUCT, MessageType,
                                         MotionMode, ProtocolMessage, pack_ack, pack_json,
                                         pack_motion, unpack_ack, unpack_action_request,
                                         unpack_json)

from .telemetry import TelemetryCache
from .transport_base import Transport
from .watchdog import LinkWatchdog


LOG = logging.getLogger(__name__)
class VehicleGateway:
    def __init__(self, transport: Transport, vehicle_config: Dict[str, Any]) -> None:
        self.transport = transport
        self.config = vehicle_config
        self.sequence = 0
        self.decoder = FrameDecoder()
        self.telemetry = TelemetryCache()
        self.watchdog = LinkWatchdog(int(vehicle_config.get("link_timeout_ms", 500)))
        self.stats = {"sent": 0, "received": 0, "decode_errors": 0, "acks": 0,
                      "unknown_acks": 0, "old_sequence": 0}
        self.pending: Dict[int, float] = {}
        self.last_remote_sequence: Optional[int] = None
        self.last_action_request: Optional[Dict[str, Any]] = None
        self._action_requests: "Queue[Dict[str, Any]]" = Queue()
        self._seen_action_sequences = deque(maxlen=128)
        self._lock = threading.Lock()

    def open(self) -> None:
        self.transport.open()
        self.send_motion(MotionTarget(enabled=False,
                         valid_for_ms=int(self.config.get("default_valid_for_ms", 200))))

    def _send(self, message_type: MessageType, payload: bytes, expect_ack: bool = False) -> int:
        with self._lock:
            sequence = self.sequence
            self.sequence = (self.sequence + 1) & 0xFFFF
            self.transport.send(encode_frame(ProtocolMessage(message_type, sequence, payload)))
            self.stats["sent"] += 1
            if expect_ack:
                self.pending[sequence] = time.monotonic()
            return sequence

    def send_motion(self, target: MotionTarget) -> int:
        safe_target = target
        capture_config = self.config.get("capture", {})
        balance_config = self.config.get("balance", {})
        # Output-only is deliberately allowed through the master actuation gate: the
        # semantic measurement reaches an analyzer or MSPM0, while its enabled bit
        # remains zero below.  This permits UART bring-up with vehicle control off.
        output_only_target = (
            (target.mode == MotionMode.CAPTURE_TARGET_POLAR
             and bool(capture_config.get("enabled", False))
             and bool(capture_config.get("output_only", False)))
            or (target.mode == MotionMode.BALANCE_ROLLER
                and bool(balance_config.get("enabled", False))
                and bool(balance_config.get("output_only", False)))
        )
        if not bool(self.config.get("control_enabled", False)) and not output_only_target:
            safe_target = target.safe()
        capture = safe_target.capture_target
        balance = safe_target.balance_state
        capture_permitted = (safe_target.mode == MotionMode.CAPTURE_TARGET_POLAR
                             and bool(capture_config.get("enabled", False))
                             and capture is not None
                             and not capture.is_expired(time.monotonic_ns() // 1_000_000))
        if safe_target.mode == MotionMode.CAPTURE_TARGET_POLAR:
            if capture_permitted:
                output_only = bool(capture_config.get("output_only", False))
                payload = pack_motion(safe_target.mode, safe_target.enabled and not output_only,
                                      safe_target.valid_for_ms,
                                      track_id=capture.track_id, bearing_mdeg=capture.bearing_mdeg,
                                      range_mm=capture.range_mm,
                                      confidence_permille=capture.confidence_permille,
                                      measurement_age_ms=capture.measurement_age_ms,
                                      target_valid=capture.target_valid,
                                      capture_armed=capture.capture_armed)
            else:
                payload = pack_motion(safe_target.mode, False, safe_target.valid_for_ms)
        elif safe_target.mode == MotionMode.BALANCE_ROLLER:
            balance_permitted = (bool(balance_config.get("enabled", False))
                                 and balance is not None
                                 and not balance.is_expired(time.monotonic_ns() // 1_000_000))
            if balance_permitted:
                output_only = bool(balance_config.get("output_only", False))
                payload = pack_motion(
                    safe_target.mode, safe_target.enabled and not output_only,
                    safe_target.valid_for_ms, balance_error_mm=balance.error_mm,
                    balance_valid=balance.valid)
            else:
                payload = pack_motion(safe_target.mode, False, safe_target.valid_for_ms)
        else:
            payload = pack_motion(safe_target.mode, safe_target.enabled, safe_target.valid_for_ms)
        return self._send(MessageType.CMD_MOTION, payload, expect_ack=True)

    def send_event(self, event_type: str, payload: Dict[str, Any], valid_for_ms: int,
                   *, expect_ack: bool = True) -> int:
        return self._send(MessageType.CMD_EVENT, pack_json({"event_type": event_type,
                          "payload": payload, "valid_for_ms": valid_for_ms}),
                          expect_ack=expect_ack)

    def receive_action_request(self, timeout: float = 0.0) -> Optional[Dict[str, Any]]:
        """Return the next validated, de-duplicated remote action request."""
        try:
            return self._action_requests.get(timeout=timeout)
        except Empty:
            return None

    def send_heartbeat(self) -> int:
        now_ms = (time.monotonic_ns() // 1_000_000) & 0xFFFFFFFF
        validity = int(self.config.get("default_valid_for_ms", 200))
        return self._send(MessageType.HEARTBEAT, HEARTBEAT_STRUCT.pack(now_ms, validity))

    def poll(self, timeout: float = 0.0) -> None:
        data = self.transport.receive(timeout)
        if not data:
            return
        previous_errors = self.decoder.errors
        for message in self.decoder.feed(data):
            self.stats["received"] += 1
            self.watchdog.feed()
            if self.last_remote_sequence is not None:
                delta = (message.sequence - self.last_remote_sequence) & 0xFFFF
                if delta == 0 or delta > 0x8000:
                    self.stats["old_sequence"] += 1
            self.last_remote_sequence = message.sequence
            self._handle(message)
        self.stats["decode_errors"] += self.decoder.errors - previous_errors

    def _handle(self, message: ProtocolMessage) -> None:
        if message.message_type == MessageType.CMD_EVENT:
            try:
                event = unpack_json(message.payload)
                if event.get("event_type") != "ACTION_REQUEST":
                    return
                request = unpack_action_request(message.payload)
            except (TypeError, ValueError) as error:
                LOG.warning("invalid remote action request: %s", error)
                self._send(MessageType.ACK, pack_ack(message.sequence, ACK_STATUS_INVALID))
                return
            if message.sequence in self._seen_action_sequences:
                self.stats["old_sequence"] += 1
                self._send(MessageType.ACK, pack_ack(message.sequence, ACK_STATUS_DUPLICATE))
                return
            self._seen_action_sequences.append(message.sequence)
            request["frame_sequence"] = message.sequence
            request["received_monotonic_ms"] = time.monotonic_ns() // 1_000_000
            self.last_action_request = dict(request)
            self._action_requests.put(request)
            self._send(MessageType.ACK, pack_ack(message.sequence, ACK_STATUS_ACCEPTED))
        elif message.message_type == MessageType.TELEMETRY:
            self.telemetry.update(unpack_json(message.payload))
        elif message.message_type == MessageType.ACK:
            ack = unpack_ack(message.payload)
            if self.pending.pop(ack["acknowledged_sequence"], None) is None:
                self.stats["unknown_acks"] += 1
            else:
                self.stats["acks"] += 1
        elif message.message_type == MessageType.FAULT:
            value = unpack_json(message.payload)
            value["fault_message"] = True
            self.telemetry.update(value)

    def close(self) -> None:
        try:
            self.send_motion(MotionTarget(enabled=False,
                             valid_for_ms=int(self.config.get("default_valid_for_ms", 200))))
        except Exception:
            LOG.exception("failed to send final disable command")
        finally:
            self.transport.close()
