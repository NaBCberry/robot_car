"""Protocol message types and semantic payload helpers."""

from __future__ import annotations

import json
import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Any, Dict


PROTOCOL_VERSION = 2


class MessageType(IntEnum):
    CMD_MOTION = 0x01
    CMD_EVENT = 0x02
    HEARTBEAT = 0x03
    TELEMETRY = 0x10
    ACK = 0x11
    FAULT = 0x12


class MotionMode(IntEnum):
    """High-level movement behaviors implemented by the MSPM0."""

    DISABLED = 0
    IDLE = 1
    LINE_FOLLOW = 2
    VISION_ASSIST = 3
    CAPTURE_TARGET_POLAR = 4
    BALANCE_ROLLER = 5


@dataclass(frozen=True)
class ProtocolMessage:
    message_type: MessageType
    sequence: int
    payload: bytes = b""
    version: int = PROTOCOL_VERSION

    def __post_init__(self) -> None:
        if not 0 <= self.sequence <= 0xFFFF:
            raise ValueError("sequence must fit uint16")


# mode:u8, flags:u8, valid_for_ms:u16
MOTION_COMMON_STRUCT = struct.Struct(">BBH")
# track_id:u16, bearing_mdeg:i32, range_mm:i32, confidence_permille:u16,
# measurement_age_ms:u16
MOTION_POLAR_TARGET_STRUCT = struct.Struct(">HiiHH")
# error_mm:i16, signed distance from the tube center
MOTION_BALANCE_ERROR_STRUCT = struct.Struct(">h")
ACK_STRUCT = struct.Struct(">HB")
HEARTBEAT_STRUCT = struct.Struct(">IH")
MOTION_FLAG_ENABLED = 0x01
MOTION_FLAG_TARGET_VALID = 0x02
MOTION_FLAG_CAPTURE_ARMED = 0x04
MOTION_FLAG_BALANCE_VALID = 0x08
MOTION_FLAG_MASK = (MOTION_FLAG_ENABLED | MOTION_FLAG_TARGET_VALID | MOTION_FLAG_CAPTURE_ARMED
                    | MOTION_FLAG_BALANCE_VALID)


def _motion_mode(value: int | MotionMode) -> MotionMode:
    try:
        return MotionMode(value)
    except ValueError as error:
        raise ValueError(f"unknown CMD_MOTION mode: {value}") from error


def _validate_uint(value: int, label: str, maximum: int, *, minimum: int = 0) -> None:
    if not minimum <= value <= maximum:
        raise ValueError(f"{label} must fit {minimum}..{maximum}")


def _validate_int16(value: int, label: str) -> None:
    if not -0x8000 <= value <= 0x7FFF:
        raise ValueError(f"{label} must fit int16")


def pack_motion(mode: int | MotionMode, enabled: bool, valid_for_ms: int, *,
                track_id: int = 0, bearing_mdeg: int = 0, range_mm: int = 0,
                confidence_permille: int = 0, measurement_age_ms: int = 0,
                target_valid: bool = False, capture_armed: bool = False,
                balance_error_mm: int = 0,
                balance_valid: bool = False) -> bytes:
    """Pack one v2 semantic motion intent without exposing wheel control."""
    parsed_mode = _motion_mode(mode)
    _validate_uint(valid_for_ms, "valid_for_ms", 0xFFFF, minimum=1)
    flags = MOTION_FLAG_ENABLED if enabled else 0

    polar_fields = (track_id, bearing_mdeg, range_mm, confidence_permille, measurement_age_ms,
                    target_valid, capture_armed)
    balance_fields = (balance_error_mm, balance_valid)
    if parsed_mode not in {MotionMode.CAPTURE_TARGET_POLAR, MotionMode.BALANCE_ROLLER}:
        if any(polar_fields) or any(balance_fields):
            raise ValueError("only target motion modes may contain a mode payload")
        return MOTION_COMMON_STRUCT.pack(parsed_mode, flags, valid_for_ms)

    if parsed_mode == MotionMode.CAPTURE_TARGET_POLAR:
        if any(balance_fields):
            raise ValueError("BALANCE_ROLLER fields cannot accompany a polar target")
        if target_valid:
            _validate_uint(track_id, "track_id", 0xFFFF)
            _validate_uint(range_mm, "range_mm", 0x7FFFFFFF)
            _validate_uint(confidence_permille, "confidence_permille", 1000)
            _validate_uint(measurement_age_ms, "measurement_age_ms", 0xFFFF)
            if not -0x80000000 <= bearing_mdeg <= 0x7FFFFFFF:
                raise ValueError("bearing_mdeg must fit int32")
            flags |= MOTION_FLAG_TARGET_VALID
            if capture_armed:
                flags |= MOTION_FLAG_CAPTURE_ARMED
        elif any((track_id, bearing_mdeg, range_mm, confidence_permille, measurement_age_ms,
                  capture_armed)):
            raise ValueError("an invalid polar target must use zero fields and be disarmed")
        return (MOTION_COMMON_STRUCT.pack(parsed_mode, flags, valid_for_ms)
                + MOTION_POLAR_TARGET_STRUCT.pack(track_id, bearing_mdeg, range_mm,
                                                  confidence_permille, measurement_age_ms))

    if any(polar_fields):
        raise ValueError("polar target fields cannot accompany BALANCE_ROLLER")
    if balance_valid:
        _validate_int16(balance_error_mm, "balance_error_mm")
        flags |= MOTION_FLAG_BALANCE_VALID
    elif balance_error_mm:
        raise ValueError("an invalid balance state must use zero fields")
    return (MOTION_COMMON_STRUCT.pack(parsed_mode, flags, valid_for_ms)
            + MOTION_BALANCE_ERROR_STRUCT.pack(balance_error_mm))


def unpack_motion(payload: bytes) -> Dict[str, Any]:
    """Decode and strictly validate a v2 semantic motion intent."""
    if len(payload) < MOTION_COMMON_STRUCT.size:
        raise ValueError("CMD_MOTION payload is shorter than its common header")
    raw_mode, flags, valid_for_ms = MOTION_COMMON_STRUCT.unpack(payload[:MOTION_COMMON_STRUCT.size])
    mode = _motion_mode(raw_mode)
    if not 0 < valid_for_ms <= 0xFFFF:
        raise ValueError("CMD_MOTION valid_for_ms must be positive")
    if flags & ~MOTION_FLAG_MASK:
        raise ValueError("CMD_MOTION flags contain reserved bits")
    base = {
        "mode": mode,
        "enabled": bool(flags & MOTION_FLAG_ENABLED),
        "flags": flags,
        "valid_for_ms": valid_for_ms,
    }
    if mode not in {MotionMode.CAPTURE_TARGET_POLAR, MotionMode.BALANCE_ROLLER}:
        if flags & (MOTION_FLAG_TARGET_VALID | MOTION_FLAG_CAPTURE_ARMED | MOTION_FLAG_BALANCE_VALID):
            raise ValueError("only target motion modes may set target flags")
        if len(payload) != MOTION_COMMON_STRUCT.size:
            raise ValueError("CMD_MOTION mode has an unexpected mode payload")
        return base

    if mode == MotionMode.CAPTURE_TARGET_POLAR:
        if flags & MOTION_FLAG_BALANCE_VALID:
            raise ValueError("BALANCE_ROLLER flag is invalid for a polar target")
        expected_size = MOTION_COMMON_STRUCT.size + MOTION_POLAR_TARGET_STRUCT.size
        if len(payload) != expected_size:
            raise ValueError("CAPTURE_TARGET_POLAR has an invalid payload length")
        track_id, bearing, range_mm, confidence, age_ms = MOTION_POLAR_TARGET_STRUCT.unpack(
            payload[MOTION_COMMON_STRUCT.size:])
        target_valid = bool(flags & MOTION_FLAG_TARGET_VALID)
        capture_armed = bool(flags & MOTION_FLAG_CAPTURE_ARMED)
        if confidence > 1000:
            raise ValueError("CAPTURE_TARGET_POLAR confidence exceeds 1000")
        if not target_valid and (track_id or bearing or range_mm or confidence or age_ms or capture_armed):
            raise ValueError("invalid CAPTURE_TARGET_POLAR must use zero fields and be disarmed")
        return {
            **base,
            "target_valid": target_valid,
            "capture_armed": capture_armed,
            "track_id": track_id,
            "bearing_mdeg": bearing,
            "range_mm": range_mm,
            "confidence_permille": confidence,
            "measurement_age_ms": age_ms,
        }

    if flags & (MOTION_FLAG_TARGET_VALID | MOTION_FLAG_CAPTURE_ARMED):
        raise ValueError("polar target flags are invalid for BALANCE_ROLLER")
    expected_size = MOTION_COMMON_STRUCT.size + MOTION_BALANCE_ERROR_STRUCT.size
    if len(payload) != expected_size:
        raise ValueError("BALANCE_ROLLER has an invalid payload length")
    (error_mm,) = MOTION_BALANCE_ERROR_STRUCT.unpack(payload[MOTION_COMMON_STRUCT.size:])
    balance_valid = bool(flags & MOTION_FLAG_BALANCE_VALID)
    if not balance_valid and error_mm:
        raise ValueError("invalid BALANCE_ROLLER must use zero fields")
    return {
        **base,
        "balance_valid": balance_valid,
        "error_mm": error_mm,
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
