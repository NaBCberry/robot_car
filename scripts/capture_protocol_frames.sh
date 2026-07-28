#!/usr/bin/env bash
# Receive and decode robot-car protocol-v2 frames from a local USB-UART adapter.
set -euo pipefail

project_root=$(cd "$(dirname "$0")/.." && pwd)

usage() {
    cat <<'EOF'
用法：
  capture_protocol_frames.sh --device DEVICE [选项]

本脚本只从本地电脑的串口读取数据，绝不发送任何字节。它解码 RDK 小车 v2 协议：
A5 5A 帧头、版本、消息类型、序列号、payload 长度、payload、CRC-16/CCITT。

选项：
  --device DEVICE       本地 USB-UART 接收设备，例如 /dev/ttyUSB0（必填）
  --baudrate RATE       波特率，默认 115200
  --count N             解出 N 帧后退出，默认持续抓包
  --timeout SEC         总抓包时间，秒；0 表示持续抓包，默认 0
  --show-chunks         同时显示每个原始读取块，便于定位不完整或损坏数据
  -h, --help            显示本帮助

接线：USB-UART 的 RX 接 RDK UART 的 TX，GND 接 GND。只观察 RDK 到 M0 的命令时，
不需要连接 USB-UART 的 TX。需要同时抓取 M0 回复时，应使用第二个接收通道或逻辑分析仪。

示例：
  ./scripts/capture_protocol_frames.sh --device /dev/ttyUSB0
  ./scripts/capture_protocol_frames.sh --device /dev/ttyUSB0 --count 20 --show-chunks
EOF
}

device=
baudrate=115200
count=0
timeout=0
show_chunks=0

while (($#)); do
    case "$1" in
        --device) device=$2; shift 2 ;;
        --baudrate) baudrate=$2; shift 2 ;;
        --count) count=$2; shift 2 ;;
        --timeout) timeout=$2; shift 2 ;;
        --show-chunks) show_chunks=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "未知参数：$1" >&2; usage >&2; exit 2 ;;
    esac
done

if [[ -z "$device" ]]; then
    echo "必须使用 --device 指定本地 USB-UART 接收设备" >&2
    exit 2
fi
if [[ ! -c "$device" ]]; then
    echo "串口不是字符设备：$device" >&2
    exit 2
fi
if [[ ! "$baudrate" =~ ^[0-9]+$ ]] || ((baudrate <= 0)); then
    echo "--baudrate 必须为正整数" >&2
    exit 2
fi
if [[ ! "$count" =~ ^[0-9]+$ ]]; then
    echo "--count 必须是非负整数" >&2
    exit 2
fi
if [[ ! "$timeout" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
    echo "--timeout 必须是非负秒数" >&2
    exit 2
fi

PYTHONPATH="$project_root/src${PYTHONPATH:+:$PYTHONPATH}" python3 - \
    "$device" "$baudrate" "$count" "$timeout" "$show_chunks" <<'PY'
import json
import os
import select
import signal
import struct
import sys
import termios
import time

from robot_car.protocol.framing import FrameDecoder, encode_frame
from robot_car.protocol.messages import (HEARTBEAT_STRUCT, MessageType, ProtocolMessage,
                                         unpack_ack, unpack_json, unpack_motion)


device, raw_baudrate, raw_count, raw_timeout, raw_show_chunks = sys.argv[1:]
baudrate = int(raw_baudrate)
limit = int(raw_count)
timeout = float(raw_timeout)
show_chunks = raw_show_chunks == "1"


def timestamp() -> str:
    return time.strftime("%Y-%m-%d %H:%M:%S") + f".{time.time_ns() % 1_000_000_000 // 1_000_000:03d}"


def describe(message: ProtocolMessage) -> str:
    try:
        if message.message_type == MessageType.CMD_MOTION:
            value = unpack_motion(message.payload)
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


baud_constant = getattr(termios, f"B{baudrate}", None)
if baud_constant is None:
    raise SystemExit(f"当前 Python/内核不支持该标准波特率：{baudrate}")

try:
    fd = os.open(device, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
except OSError as error:
    raise SystemExit(f"无法以只读方式打开 {device}: {error}") from error

running = True


def stop(_signum: int, _frame: object) -> None:
    global running
    running = False


signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)
decoder = FrameDecoder()
frames = 0
started = time.monotonic()
previous_errors = 0
print(f"监听 {device} @ {baudrate} 8N1；只读，不会发送字节。", flush=True)

try:
    attrs = termios.tcgetattr(fd)
    attrs[0] &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK | termios.ISTRIP |
                  termios.INLCR | termios.IGNCR | termios.ICRNL | termios.IXON |
                  termios.IXOFF | termios.IXANY)
    attrs[1] = 0
    attrs[2] &= ~(termios.PARENB | termios.CSTOPB | termios.CSIZE |
                  getattr(termios, "CRTSCTS", 0))
    attrs[2] |= termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[3] = 0
    attrs[4] = baud_constant
    attrs[5] = baud_constant
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)

    while running and (limit == 0 or frames < limit):
        if timeout and time.monotonic() - started >= timeout:
            break
        ready, _, _ = select.select([fd], [], [], 0.2)
        if not ready:
            continue
        chunk = os.read(fd, 4096)
        if not chunk:
            continue
        if show_chunks:
            print(f"{timestamp()} RX {len(chunk)}B {chunk.hex(' ').upper()}", flush=True)
        for message in decoder.feed(chunk):
            frame = encode_frame(message)
            print(f"{timestamp()} FRAME {len(frame)}B type={message.message_type.name} "
                  f"(0x{int(message.message_type):02X}) sequence={message.sequence}", flush=True)
            print(f"  HEX  {frame.hex(' ').upper()}", flush=True)
            print(f"  DATA {describe(message)}", flush=True)
            frames += 1
            if limit and frames >= limit:
                break
        if decoder.errors != previous_errors:
            print(f"{timestamp()} DECODE_ERROR total={decoder.errors}", flush=True)
            previous_errors = decoder.errors
finally:
    os.close(fd)

print(f"抓包结束：已解码 {frames} 帧，解码错误 {decoder.errors} 次。", flush=True)
PY
