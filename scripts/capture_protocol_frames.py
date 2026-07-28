#!/usr/bin/env python3
"""Read and decode robot-car v2 protocol frames from a local serial port."""

from __future__ import annotations

import argparse
import json
import signal
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Optional

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from robot_car.protocol.framing import FrameDecoder, encode_frame
from robot_car.protocol.messages import (HEARTBEAT_STRUCT, MessageType, ProtocolMessage,
                                         unpack_ack, unpack_json, unpack_motion)

try:
    import serial
    import serial.tools.list_ports
except ImportError as error:
    raise SystemExit("缺少 pyserial；请运行: py -m pip install pyserial") from error


def list_serial_ports() -> list[serial.tools.list_ports.ListPortInfo]:
    """枚举当前系统上可用的串口列表。"""
    ports = sorted(serial.tools.list_ports.comports(), key=lambda p: p.device)
    return ports


def _choose_port_interactively() -> str:
    """交互式让用户选择一个串口，返回设备路径。"""
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


def parse_args() -> argparse.Namespace:
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
    return parser.parse_args()


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
    args = parse_args()
    if args.baudrate <= 0:
        raise SystemExit("--baudrate 必须为正整数")
    if args.count < 0:
        raise SystemExit("--count 必须是非负整数")
    if args.timeout < 0:
        raise SystemExit("--timeout 必须是非负秒数")

    # 未指定串口时自动探测并交互选择
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
        # pyserial opens an input/output handle internally, but this program never calls write().
        # Pseudo terminals and some USB adapters do not expose modem-control ioctls.
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
