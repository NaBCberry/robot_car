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
    CMD_CAPTURE_TARGET = 0x04
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
# flags:u8, reserved:u8, track_id:u16, bearing_mdeg:i32, range_mm:i32,
# confidence_permille:u16, measurement_age_ms:u16, valid_for_ms:u16
CAPTURE_TARGET_STRUCT = struct.Struct(">BBHiiHHH")
CAPTURE_TARGET_FLAG_VALID = 0x01
CAPTURE_TARGET_FLAG_ARMED = 0x02


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


def pack_capture_target(flags: int, track_id: int, bearing_mdeg: int, range_mm: int,
                        confidence_permille: int, measurement_age_ms: int,
                        valid_for_ms: int) -> bytes:
    """Pack a high-rate target relative to the MSPM0 capture point."""
    if not 0 <= flags <= 0xFF:
        raise ValueError("capture target flags must fit uint8")
    if not 0 <= track_id <= 0xFFFF:
        raise ValueError("capture target track_id must fit uint16")
    if not 0 <= confidence_permille <= 1000:
        raise ValueError("capture target confidence must be 0..1000")
    if not 0 <= measurement_age_ms <= 0xFFFF:
        raise ValueError("capture target measurement_age_ms must fit uint16")
    if not 0 < valid_for_ms <= 0xFFFF:
        raise ValueError("capture target valid_for_ms must fit uint16 and be positive")
    return CAPTURE_TARGET_STRUCT.pack(flags, 0, track_id, bearing_mdeg, range_mm,
                                      confidence_permille, measurement_age_ms, valid_for_ms)


def unpack_capture_target(payload: bytes) -> Dict[str, Any]:
    """Decode a target measurement without interpreting control policy."""
    if len(payload) != CAPTURE_TARGET_STRUCT.size:
        raise ValueError("invalid CMD_CAPTURE_TARGET payload length")
    flags, reserved, track_id, bearing, range_mm, confidence, age_ms, valid_for_ms = \
        CAPTURE_TARGET_STRUCT.unpack(payload)
    if reserved != 0:
        raise ValueError("CMD_CAPTURE_TARGET reserved byte must be zero")
    if confidence > 1000:
        raise ValueError("CMD_CAPTURE_TARGET confidence exceeds 1000")
    return {
        "target_valid": bool(flags & CAPTURE_TARGET_FLAG_VALID),
        "capture_armed": bool(flags & CAPTURE_TARGET_FLAG_ARMED),
        "flags": flags,
        "track_id": track_id,
        "bearing_mdeg": bearing,
        "range_mm": range_mm,
        "confidence_permille": confidence,
        "measurement_age_ms": age_ms,
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
