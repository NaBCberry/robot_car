#!/usr/bin/env python3
"""Read and decode robot-car v2 protocol frames from a local serial port.
Self-contained — no project-side imports required."""

from __future__ import annotations

import argparse
import json
import signal
import struct
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from enum import IntEnum
from typing import Any, Dict, List

# ── protocol constants ────────────────────────────────────────────────

PROTOCOL_VERSION = 2
MAGIC = b"\xA5\x5A"
HEADER_STRUCT = struct.Struct(">BBHB")   # version:u8, type:u8, seq:u16, payload_len:u8
CRC_STRUCT = struct.Struct(">H")
MAX_PAYLOAD = 0xFF

MOTION_COMMON_STRUCT = struct.Struct(">BBH")       # mode:u8, flags:u8, valid_for_ms:u16
MOTION_POLAR_TARGET_STRUCT = struct.Struct(">HiiHH")  # track_id, bearing_mdeg, range_mm, confidence, age_ms
MOTION_BALANCE_ERROR_STRUCT = struct.Struct(">h")  # error_mm:i16
ACK_STRUCT = struct.Struct(">HB")                  # acknowledged_sequence:u16, status:u8
HEARTBEAT_STRUCT = struct.Struct(">IH")             # monotonic_ms:u32, valid_for_ms:u16

MOTION_FLAG_ENABLED = 0x01
MOTION_FLAG_TARGET_VALID = 0x02
MOTION_FLAG_CAPTURE_ARMED = 0x04
MOTION_FLAG_BALANCE_VALID = 0x08
MOTION_FLAG_MASK = (MOTION_FLAG_ENABLED | MOTION_FLAG_TARGET_VALID | MOTION_FLAG_CAPTURE_ARMED
                    | MOTION_FLAG_BALANCE_VALID)


class MessageType(IntEnum):
    CMD_MOTION = 0x01
    CMD_EVENT = 0x02
    HEARTBEAT = 0x03
    TELEMETRY = 0x10
    ACK = 0x11
    FAULT = 0x12


class MotionMode(IntEnum):
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


class ProtocolError(ValueError):
    pass


# ── CRC & frame encode/decode ─────────────────────────────────────────

def crc16_ccitt(data: bytes, initial: int = 0xFFFF) -> int:
    crc = initial
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_frame(message: ProtocolMessage) -> bytes:
    if len(message.payload) > MAX_PAYLOAD:
        raise ProtocolError("payload too large")
    raw = HEADER_STRUCT.pack(message.version, int(message.message_type),
                             message.sequence, len(message.payload)) + message.payload
    raw += CRC_STRUCT.pack(crc16_ccitt(raw))
    return MAGIC + raw


def _decode_packet(packet: bytes) -> ProtocolMessage:
    if len(packet) < HEADER_STRUCT.size + CRC_STRUCT.size:
        raise ProtocolError("frame too short")
    body, received_crc = packet[:-CRC_STRUCT.size], CRC_STRUCT.unpack(packet[-CRC_STRUCT.size:])[0]
    if crc16_ccitt(body) != received_crc:
        raise ProtocolError("CRC mismatch")
    version, raw_type, sequence, payload_length = HEADER_STRUCT.unpack(body[:HEADER_STRUCT.size])
    if version != PROTOCOL_VERSION:
        raise ProtocolError(f"unsupported protocol version: {version}")
    if payload_length > MAX_PAYLOAD or len(body) != HEADER_STRUCT.size + payload_length:
        raise ProtocolError("invalid payload length")
    try:
        message_type = MessageType(raw_type)
    except ValueError as error:
        raise ProtocolError(f"unknown message type: {raw_type}") from error
    return ProtocolMessage(message_type, sequence, body[HEADER_STRUCT.size:], version)


class FrameDecoder:
    """Incremental length-based decoder; malformed candidates are resynchronized."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.errors = 0

    def feed(self, data: bytes) -> List[ProtocolMessage]:
        self._buffer.extend(data)
        messages: List[ProtocolMessage] = []
        minimum_size = len(MAGIC) + HEADER_STRUCT.size + CRC_STRUCT.size
        while True:
            start = self._buffer.find(MAGIC)
            if start < 0:
                if self._buffer[-1:] == MAGIC[:1]:
                    del self._buffer[:-1]
                else:
                    self._buffer.clear()
                return messages
            if start:
                del self._buffer[:start]
            if len(self._buffer) < minimum_size:
                return messages
            header_start = len(MAGIC)
            _, _, _, payload_length = HEADER_STRUCT.unpack(
                self._buffer[header_start:header_start + HEADER_STRUCT.size])
            if payload_length > MAX_PAYLOAD:
                self.errors += 1
                del self._buffer[0]
                continue
            frame_size = len(MAGIC) + HEADER_STRUCT.size + payload_length + CRC_STRUCT.size
            if len(self._buffer) < frame_size:
                return messages
            try:
                messages.append(_decode_packet(bytes(self._buffer[len(MAGIC):frame_size])))
            except ProtocolError:
                self.errors += 1
                del self._buffer[0]
            else:
                del self._buffer[:frame_size]
        return messages


# ── semantic payload helpers ───────────────────────────────────────────

def _motion_mode(value: int) -> MotionMode:
    try:
        return MotionMode(value)
    except ValueError as error:
        raise ValueError(f"unknown CMD_MOTION mode: {value}") from error


def unpack_motion(payload: bytes) -> Dict[str, Any]:
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
    return {**base, "balance_valid": balance_valid, "error_mm": error_mm}


def unpack_ack(payload: bytes) -> Dict[str, int]:
    if len(payload) != ACK_STRUCT.size:
        raise ValueError("invalid ACK payload length")
    sequence, status = ACK_STRUCT.unpack(payload)
    return {"acknowledged_sequence": sequence, "status": status}


def unpack_json(payload: bytes) -> Dict[str, Any]:
    value = json.loads(payload.decode("utf-8"))
    if not isinstance(value, dict):
        raise ValueError("JSON payload must be an object")
    return value


# ── serial helpers ─────────────────────────────────────────────────────

try:
    import serial
    import serial.tools.list_ports
except ImportError as error:
    raise SystemExit("缺少 pyserial；请运行: python -m pip install pyserial") from error


def list_serial_ports() -> list[serial.tools.list_ports.ListPortInfo]:
    return sorted(serial.tools.list_ports.comports(), key=lambda p: p.device)


def _choose_port_interactively() -> str:
    ports = list_serial_ports()
    if not ports:
        raise SystemExit("未检测到任何可用串口，请检查硬件连接后重试。")
    print("检测到以下可用串口：")
    for idx, port in enumerate(ports, 1):
        desc = f" — {port.description}" if port.description else ""
        hwid = f" (hwid: {port.hwid})" if port.hwid else ""
        print(f"  [{idx}] {port.device}{desc}{hwid}")
    if len(ports) == 1:
        print(f"仅发现一个串口，自动选择 {ports[0].device}")
        return ports[0].device
    print(f"  [0] 退出")
    while True:
        try:
            choice = input("请选择串口序号: ").strip()
            idx = int(choice)
            if idx == 0:
                raise SystemExit("用户取消选择。")
            if 1 <= idx <= len(ports):
                return ports[idx - 1].device
        except ValueError:
            pass
        print(f"请输入 0-{len(ports)} 之间的数字。")


# ── CLI ────────────────────────────────────────────────────────────────

def timestamp() -> str:
    return datetime.now().isoformat(timespec="milliseconds")


def describe(message: ProtocolMessage) -> str:
    try:
        if message.message_type == MessageType.CMD_MOTION:
            value: dict[str, Any] = unpack_motion(message.payload)
            value["mode"] = value["mode"].name
        elif message.message_type == MessageType.ACK:
            value = unpack_ack(message.payload)
        elif message.message_type == MessageType.HEARTBEAT:
            if len(message.payload) != HEARTBEAT_STRUCT.size:
                raise ValueError("invalid HEARTBEAT payload length")
            monotonic_ms, valid_for_ms = HEARTBEAT_STRUCT.unpack(message.payload)
            value = {"monotonic_ms": monotonic_ms, "valid_for_ms": valid_for_ms}
        elif message.message_type in {MessageType.CMD_EVENT, MessageType.TELEMETRY,
                                      MessageType.FAULT}:
            value = unpack_json(message.payload)
        else:
            value = {"payload_hex": message.payload.hex(" ").upper()}
        return json.dumps(value, ensure_ascii=False, separators=(",", ":"))
    except (UnicodeDecodeError, ValueError, json.JSONDecodeError) as error:
        return f"payload_decode_error={error}; payload_hex={message.payload.hex(' ').upper()}"


def main() -> int:
    parser = argparse.ArgumentParser(
        description="只读抓取并解码 RDK 小车 v2 UART 协议帧，不发送任何串口字节。",
    )
    parser.add_argument("--port", "--device", dest="port", default=None,
                        help="本地串口，例如 Linux 的 /dev/ttyUSB0 或 Windows 的 COM3；不指定则自动探测并交互选择")
    parser.add_argument("--baudrate", type=int, default=115200, help="波特率，默认 115200")
    parser.add_argument("--count", type=int, default=0,
                        help="解出 N 帧后退出；0 表示持续抓包，默认 0")
    parser.add_argument("--timeout", type=float, default=0,
                        help="总抓包时间（秒）；0 表示持续抓包，默认 0")
    parser.add_argument("--show-chunks", action="store_true",
                        help="同时显示每个原始读取块")
    args = parser.parse_args()

    if args.baudrate <= 0:
        raise SystemExit("--baudrate 必须为正整数")
    if args.count < 0:
        raise SystemExit("--count 必须是非负整数")
    if args.timeout < 0:
        raise SystemExit("--timeout 必须是非负秒数")

    if args.port is None:
        args.port = _choose_port_interactively()

    running = True

    def stop(_signum: int, _frame: object) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    decoder = FrameDecoder()
    frames = 0
    previous_errors = 0
    started = time.monotonic()

    try:
        stream = serial.Serial(
            port=args.port,
            baudrate=args.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.2,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
    except serial.SerialException as error:
        raise SystemExit(f"无法以只读抓包方式打开 {args.port}: {error}") from error

    try:
        try:
            stream.rts = False
            stream.dtr = False
        except (OSError, serial.SerialException):
            pass
        print(f"监听 {args.port} @ {args.baudrate} 8N1；只读，不会发送字节。", flush=True)
        while running and (args.count == 0 or frames < args.count):
            if args.timeout and time.monotonic() - started >= args.timeout:
                break
            chunk = stream.read(stream.in_waiting or 1)
            if not chunk:
                continue
            if args.show_chunks:
                print(f"{timestamp()} RX {len(chunk)}B {chunk.hex(' ').upper()}", flush=True)
            for message in decoder.feed(chunk):
                frame = encode_frame(message)
                print(f"{timestamp()} FRAME {len(frame)}B type={message.message_type.name} "
                      f"(0x{int(message.message_type):02X}) sequence={message.sequence}", flush=True)
                print(f"  HEX  {frame.hex(' ').upper()}", flush=True)
                print(f"  DATA {describe(message)}", flush=True)
                frames += 1
                if args.count and frames >= args.count:
                    break
            if decoder.errors != previous_errors:
                print(f"{timestamp()} DECODE_ERROR total={decoder.errors}", flush=True)
                previous_errors = decoder.errors
    finally:
        stream.close()

    print(f"抓包结束：已解码 {frames} 帧，解码错误 {decoder.errors} 次。", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
