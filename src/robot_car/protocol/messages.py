"""Protocol message types and semantic payload helpers."""

from __future__ import annotations

import json
import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Any, Dict


PROTOCOL_VERSION = 1


class MessageType(IntEnum):
    CMD_MOTION = 0x01
    CMD_EVENT = 0x02
    HEARTBEAT = 0x03
    TELEMETRY = 0x10
    ACK = 0x11
    FAULT = 0x12


@dataclass(frozen=True)
class ProtocolMessage:
    message_type: MessageType
    sequence: int
    payload: bytes = b""
    version: int = PROTOCOL_VERSION

    def __post_init__(self) -> None:
        if not 0 <= self.sequence <= 0xFFFF:
            raise ValueError("sequence must fit uint16")


MOTION_STRUCT = struct.Struct(">BBiiIH")
ACK_STRUCT = struct.Struct(">HB")
HEARTBEAT_STRUCT = struct.Struct(">IH")


def pack_motion(mode: int, enable: bool, speed: int, steering: int, speed_limit: int, valid_for_ms: int) -> bytes:
    return MOTION_STRUCT.pack(mode, int(enable), speed, steering, speed_limit, valid_for_ms)


def unpack_motion(payload: bytes) -> Dict[str, Any]:
    if len(payload) != MOTION_STRUCT.size:
        raise ValueError("invalid CMD_MOTION payload length")
    mode, enable, speed, steering, speed_limit, valid_for_ms = MOTION_STRUCT.unpack(payload)
    return {
        "mode": mode,
        "enable": bool(enable),
        "target_speed_mm_s": speed,
        "target_steering_mdeg": steering,
        "speed_limit_mm_s": speed_limit,
        "valid_for_ms": valid_for_ms,
    }


def pack_json(payload: Dict[str, Any]) -> bytes:
    return json.dumps(payload, separators=(",", ":"), ensure_ascii=False).encode("utf-8")


def unpack_json(payload: bytes) -> Dict[str, Any]:
    value = json.loads(payload.decode("utf-8"))
    if not isinstance(value, dict):
        raise ValueError("JSON payload must be an object")
    return value


def pack_ack(acknowledged_sequence: int, status: int = 0) -> bytes:
    return ACK_STRUCT.pack(acknowledged_sequence, status)


def unpack_ack(payload: bytes) -> Dict[str, int]:
    if len(payload) != ACK_STRUCT.size:
        raise ValueError("invalid ACK payload length")
    sequence, status = ACK_STRUCT.unpack(payload)
    return {"acknowledged_sequence": sequence, "status": status}
