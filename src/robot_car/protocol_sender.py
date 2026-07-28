"""Explicit UART sender for protocol-frame bring-up and hardware diagnostics."""

from __future__ import annotations

import argparse
import json
import os
import stat
import sys
import time
from typing import Any, Optional, Sequence, Tuple

from robot_car.protocol.framing import FrameDecoder, encode_frame
from robot_car.protocol.messages import MessageType, ProtocolMessage, pack_json


def parse_integer(value: str) -> int:
    try:
        return int(value, 0)
    except ValueError as error:
        raise argparse.ArgumentTypeError(f"invalid integer: {value}") from error


def build_message(message_type_value: Optional[int], payload_hex: Optional[str],
                  event_type: Optional[str], event_payload_json: str, valid_for_ms: int,
                  sequence: int) -> ProtocolMessage:
    """Build either an arbitrary protocol payload or a structured CMD_EVENT."""
    if not 0 < valid_for_ms <= 0xFFFF:
        raise ValueError("valid_for_ms must be 1..65535")
    if event_type is not None:
        if message_type_value is not None or payload_hex is not None:
            raise ValueError("--event-type cannot be combined with raw payload options")
        try:
            event_payload = json.loads(event_payload_json)
        except json.JSONDecodeError as error:
            raise ValueError("--event-payload-json must be valid JSON") from error
        if not isinstance(event_payload, dict):
            raise ValueError("--event-payload-json must be a JSON object")
        payload = pack_json({"event_type": event_type, "payload": event_payload,
                             "valid_for_ms": valid_for_ms})
        return ProtocolMessage(MessageType.CMD_EVENT, sequence, payload)
    if message_type_value is None or payload_hex is None:
        raise ValueError("provide --event-type or both --message-type and --payload-hex")
    try:
        message_type = MessageType(message_type_value)
    except ValueError as error:
        raise ValueError(f"unsupported message type: 0x{message_type_value:02X}") from error
    try:
        payload = bytes.fromhex(payload_hex)
    except ValueError as error:
        raise ValueError("--payload-hex must contain complete hexadecimal bytes") from error
    return ProtocolMessage(message_type, sequence, payload)


def send_uart(device: str, baudrate: int, frame: bytes, reply_timeout_ms: int) -> Tuple[int, list[ProtocolMessage]]:
    """Write one complete encoded frame and optionally decode replies until timeout."""
    try:
        import serial
    except ImportError as error:
        raise RuntimeError("pyserial is required; install it before using --send") from error
    try:
        device_mode = os.stat(device).st_mode
    except OSError as error:
        raise RuntimeError(f"cannot access UART device {device}: {error}") from error
    if not stat.S_ISCHR(device_mode):
        raise RuntimeError(f"UART device is not a character device: {device}")

    decoder = FrameDecoder()
    replies: list[ProtocolMessage] = []
    timeout_s = max(0, reply_timeout_ms) / 1000.0
    read_timeout_s = min(0.05, timeout_s) if timeout_s else 0
    try:
        with serial.Serial(device, baudrate=baudrate, timeout=read_timeout_s,
                           write_timeout=1.0) as port:
            written = port.write(frame)
            port.flush()
            deadline = time.monotonic() + timeout_s
            while time.monotonic() < deadline:
                data = port.read(4096)
                if data:
                    replies.extend(decoder.feed(data))
    except (serial.SerialException, OSError) as error:
        raise RuntimeError(f"UART send failed on {device}: {error}") from error
    return written, replies


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Build or explicitly send one RDK-MSPM0 protocol frame over UART.")
    parser.add_argument("--device", help="UART character device, for example /dev/ttyS1")
    parser.add_argument("--baudrate", type=parse_integer, default=115200)
    parser.add_argument("--sequence", type=parse_integer, default=0)
    parser.add_argument("--reply-timeout-ms", type=parse_integer, default=200)
    parser.add_argument("--message-type", type=parse_integer,
                        help="Raw message type, for example 0x02")
    parser.add_argument("--payload-hex",
                        help="Raw payload as even-length hexadecimal bytes, without 0x")
    parser.add_argument("--event-type", help="Build a CMD_EVENT with this event type")
    parser.add_argument("--event-payload-json", default="{}",
                        help="JSON object used with --event-type")
    parser.add_argument("--valid-for-ms", type=parse_integer, default=200,
                        help="CMD_EVENT validity period")
    parser.add_argument("--send", action="store_true",
                        help="Actually open --device and transmit the encoded frame")
    parser.add_argument("--i-understand-real-hardware", action="store_true",
                        help="Required together with --send")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        message = build_message(args.message_type, args.payload_hex, args.event_type,
                                args.event_payload_json, args.valid_for_ms, args.sequence)
    except ValueError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    frame = encode_frame(message)
    print(f"message_type=0x{int(message.message_type):02X} sequence={message.sequence}")
    print(f"payload_hex={message.payload.hex()}")
    print(f"frame_hex={frame.hex()}")
    if not args.send:
        print("dry run: no UART device was opened; add --send and --i-understand-real-hardware to transmit")
        return 0
    if not args.i_understand_real_hardware:
        print("error: --send requires --i-understand-real-hardware", file=sys.stderr)
        return 2
    if not args.device:
        print("error: --send requires --device", file=sys.stderr)
        return 2
    if args.baudrate <= 0:
        print("error: --baudrate must be positive", file=sys.stderr)
        return 2
    if args.reply_timeout_ms < 0:
        print("error: --reply-timeout-ms cannot be negative", file=sys.stderr)
        return 2
    try:
        written, replies = send_uart(args.device, args.baudrate, frame, args.reply_timeout_ms)
    except RuntimeError as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    print(f"sent_bytes={written} device={args.device} baudrate={args.baudrate}")
    for reply in replies:
        print(f"reply message_type=0x{int(reply.message_type):02X} sequence={reply.sequence} "
              f"payload_hex={reply.payload.hex()}")
    if not replies and args.reply_timeout_ms > 0:
        print("no complete reply frame received before timeout")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
