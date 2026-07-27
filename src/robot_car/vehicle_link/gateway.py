"""Protocol-aware exclusive vehicle link gateway."""

from __future__ import annotations

import logging
import struct
import threading
import time
from typing import Any, Dict, Optional

from robot_car.decision.motion_target import MotionTarget
from robot_car.protocol.framing import FrameDecoder, encode_frame
from robot_car.protocol.messages import (HEARTBEAT_STRUCT, MessageType, ProtocolMessage,
                                         pack_json, pack_motion, unpack_ack, unpack_json)

from .telemetry import TelemetryCache
from .transport_base import Transport
from .watchdog import LinkWatchdog


LOG = logging.getLogger(__name__)
MODE_VALUES = {"DISABLED": 0, "BOOT": 0, "IDLE": 1, "LINE_FOLLOW": 2, "VISION_ASSIST": 3,
               "FAILSAFE": 0, "E_STOP": 0, "FAULT": 0}


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
        self._lock = threading.Lock()

    def open(self) -> None:
        self.transport.open()
        self.send_motion(MotionTarget(enable=False, valid_for_ms=int(self.config.get("default_valid_for_ms", 200))))

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
        if not bool(self.config.get("control_enabled", False)):
            safe_target = target.safe()
        payload = pack_motion(MODE_VALUES.get(safe_target.mode, 0), safe_target.enable,
                              safe_target.target_speed_mm_s, safe_target.target_steering_mdeg,
                              safe_target.speed_limit_mm_s, safe_target.valid_for_ms)
        return self._send(MessageType.CMD_MOTION, payload, expect_ack=True)

    def send_event(self, event_type: str, payload: Dict[str, Any], valid_for_ms: int) -> int:
        return self._send(MessageType.CMD_EVENT, pack_json({"event_type": event_type,
                          "payload": payload, "valid_for_ms": valid_for_ms}), expect_ack=True)

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
        if message.message_type == MessageType.TELEMETRY:
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
            self.send_motion(MotionTarget(enable=False,
                             valid_for_ms=int(self.config.get("default_valid_for_ms", 200))))
        except Exception:
            LOG.exception("failed to send final disable command")
        finally:
            self.transport.close()
