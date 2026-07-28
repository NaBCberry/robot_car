#!/usr/bin/env bash
# Encode one protocol-v2 frame and optionally transmit it over a real UART.
set -euo pipefail

usage() {
    cat <<'EOF'
用法：
  send_protocol_frame.sh --message-type TYPE --payload-hex HEX [选项]
  send_protocol_frame.sh --raw-bytes-hex HEX [选项]

选项：
  --config PATH              transport.yaml 路径，默认项目 config/transport.yaml
  --device PATH              临时覆盖配置中的 UART 设备
  --baudrate RATE            临时覆盖配置中的波特率
  --message-type TYPE        协议消息类型（十进制或 0x 前缀十六进制）
  --payload-hex HEX          payload 的十六进制字节；可使用空格、冒号或连字符分隔
  --sequence N               协议序列号，默认 0
  --repeat N                 重复发送次数，默认 1
  --interval-ms N            重复发送间隔，默认 100 ms
  --raw-bytes-hex HEX        不封装协议、直接发送指定字节；必须同时传 --unsafe-allow-control
  --unsafe-allow-control     允许发送 CMD_MOTION(0x01)、CMD_EVENT(0x02) 或原始字节
  --send                     实际写入 UART；省略时只打印逻辑分析仪应看到的字节
  -h, --help                 显示本帮助

示例：
  # 仅显示 HEARTBEAT 帧，不写串口
  ./scripts/send_protocol_frame.sh --message-type 0x03 --payload-hex '0000000100c8'

  # 向 UART3 发送一次自定义 CMD_EVENT payload（实际发送需要两个显式开关）
  ./scripts/send_protocol_frame.sh --message-type 0x02 \
    --payload-hex '7b226576656e745f74797065223a2254455354227d' \
    --unsafe-allow-control --send

  # 发送 v2 极坐标 CMD_MOTION：有效、允许捕获、32 度、32 cm
  ./scripts/send_protocol_frame.sh --message-type 0x01 \
    --payload-hex '04 07 00 C8 00 01 00 00 7D 00 00 00 01 40 03 E8 00 00' \
    --unsafe-allow-control --send
EOF
}

project_root=$(cd "$(dirname "$0")/.." && pwd)
config_path="$project_root/config/transport.yaml"
device=
baudrate=
device_set=0
baudrate_set=0
message_type=
payload_hex=
raw_bytes_hex=
sequence=0
repeat=1
interval_ms=100
send=0
allow_control=0

while (($#)); do
    case "$1" in
        --config) config_path=$2; shift 2 ;;
        --device) device=$2; device_set=1; shift 2 ;;
        --baudrate) baudrate=$2; baudrate_set=1; shift 2 ;;
        --message-type) message_type=$2; shift 2 ;;
        --payload-hex) payload_hex=$2; shift 2 ;;
        --raw-bytes-hex) raw_bytes_hex=$2; shift 2 ;;
        --sequence) sequence=$2; shift 2 ;;
        --repeat) repeat=$2; shift 2 ;;
        --interval-ms) interval_ms=$2; shift 2 ;;
        --unsafe-allow-control) allow_control=1; shift ;;
        --send) send=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "未知参数：$1" >&2; usage >&2; exit 2 ;;
    esac
done

if [[ -n "$raw_bytes_hex" && ( -n "$message_type" || -n "$payload_hex" ) ]]; then
    echo "--raw-bytes-hex 不能与 --message-type 或 --payload-hex 同时使用" >&2
    exit 2
fi
if [[ -z "$raw_bytes_hex" && ( -z "$message_type" || -z "$payload_hex" ) ]]; then
    echo "需要指定 --message-type 和 --payload-hex，或指定 --raw-bytes-hex" >&2
    exit 2
fi
if [[ -n "$raw_bytes_hex" && $allow_control -ne 1 ]]; then
    echo "原始字节可能形成任意控制命令；必须显式传入 --unsafe-allow-control" >&2
    exit 2
fi

mapfile -t uart_config < <(python3 - "$config_path" <<'PY'
from pathlib import Path
import sys

import yaml

path = Path(sys.argv[1])
try:
    value = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    uart = value["transport"]["uart"]
    device = uart.get("device", "")
    baudrate = uart.get("baudrate", 115200)
except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as error:
    raise SystemExit(f"无法读取 UART 配置 {path}: {error}") from error
if not isinstance(device, str):
    raise SystemExit(f"UART 配置的 transport.uart.device 必须是字符串: {path}")
print(device)
print(baudrate)
PY
)

if ((device_set == 0)); then
    device=${uart_config[0]:-}
fi
if ((baudrate_set == 0)); then
    baudrate=${uart_config[1]:-}
fi

python3 - "$device" "$baudrate" "$message_type" "$payload_hex" "$raw_bytes_hex" \
    "$sequence" "$repeat" "$interval_ms" "$send" "$allow_control" <<'PY'
import os
import re
import struct
import sys
import termios
import time

device, baudrate, message_type, payload_hex, raw_bytes_hex, sequence, repeat, interval_ms, send, allow_control = sys.argv[1:]


def number(value: str, label: str, maximum: int) -> int:
    try:
        parsed = int(value, 0)
    except ValueError as error:
        raise SystemExit(f"{label} 不是有效整数：{value}") from error
    if not 0 <= parsed <= maximum:
        raise SystemExit(f"{label} 必须在 0..{maximum} 范围内")
    return parsed


def hex_bytes(value: str, label: str) -> bytes:
    cleaned = re.sub(r"[\s:_,-]", "", value).removeprefix("0x").removeprefix("0X")
    if not cleaned or len(cleaned) % 2 or not re.fullmatch(r"[0-9a-fA-F]+", cleaned):
        raise SystemExit(f"{label} 必须是偶数个十六进制字符")
    return bytes.fromhex(cleaned)


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


repeat_count = number(repeat, "repeat", 0xFFFFFFFF)
interval = number(interval_ms, "interval-ms", 0xFFFFFFFF)
if repeat_count == 0:
    raise SystemExit("repeat 必须大于零")

if raw_bytes_hex:
    frame = hex_bytes(raw_bytes_hex, "raw-bytes-hex")
    description = "原始字节"
else:
    kind = number(message_type, "message-type", 0xFF)
    seq = number(sequence, "sequence", 0xFFFF)
    payload = hex_bytes(payload_hex, "payload-hex")
    if len(payload) > 4096:
        raise SystemExit("payload 不能超过 4096 字节")
    if kind in {0x01, 0x02} and allow_control != "1":
        raise SystemExit("控制类消息需要 --unsafe-allow-control")
    body = struct.pack(">BBHH", 2, kind, seq, len(payload)) + payload
    frame = b"\xA5\x5A" + body + struct.pack(">H", crc16_ccitt(body))
    description = f"协议帧 type=0x{kind:02X} sequence={seq} payload={len(payload)}B"

print(description)
print(f"UART: {device or '(未选择设备)'} @ {baudrate} 8N1")
print(f"TX hex: {frame.hex(' ').upper()}")
if send != "1":
    print("未发送：加入 --send 后才会写入 UART。")
    raise SystemExit(0)
if not device:
    raise SystemExit("--send 需要由配置或 --device 指定 UART 设备")

baud_constant = getattr(termios, f"B{baudrate}", None)
if baud_constant is None:
    raise SystemExit(f"当前 Python/内核不支持该标准波特率：{baudrate}")
try:
    fd = os.open(device, os.O_RDWR | os.O_NOCTTY)
except OSError as error:
    raise SystemExit(f"无法打开 {device}: {error}") from error
try:
    attrs = termios.tcgetattr(fd)
    attrs[0] &= ~(termios.IXON | termios.IXOFF | termios.IXANY)
    attrs[1] = 0
    attrs[2] &= ~(termios.PARENB | termios.CSTOPB | termios.CSIZE)
    attrs[2] |= termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[3] = 0
    attrs[4] = baud_constant
    attrs[5] = baud_constant
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    for index in range(repeat_count):
        offset = 0
        while offset < len(frame):
            offset += os.write(fd, frame[offset:])
        if index + 1 < repeat_count:
            time.sleep(interval / 1000.0)
finally:
    os.close(fd)
print(f"已发送 {repeat_count} 帧。")
PY
