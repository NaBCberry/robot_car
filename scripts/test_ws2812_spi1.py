#!/usr/bin/env python3
"""Drive a WS2812 strip from RDK X5 SPI1 MOSI (40-pin header pin 19)."""

from __future__ import annotations

import argparse
import fcntl
import os
from pathlib import Path


SPI_IOC_WR_MODE = 0x40016B01
SPI_IOC_WR_BITS_PER_WORD = 0x40016B03
SPI_IOC_WR_MAX_SPEED_HZ = 0x40046B04
SPI_MODE_0 = 0
SPI_HZ = 2_400_000


def parse_rgb(value: str) -> tuple[int, int, int]:
    try:
        parts = [int(part, 0) for part in value.split(",")]
    except ValueError as error:
        raise argparse.ArgumentTypeError("颜色格式应为 R,G,B，例如 32,0,0") from error
    if len(parts) != 3 or any(not 0 <= part <= 255 for part in parts):
        raise argparse.ArgumentTypeError("每个 R,G,B 分量必须为 0 到 255")
    return tuple(parts)  # type: ignore[return-value]


def encode_ws2812(colors: list[tuple[int, int, int]]) -> bytes:
    """Encode RGB pixels as SPI mode-0 bits at 2.4 MHz.

    Each WS2812 data bit occupies three SPI bits: 1 -> 110 and 0 -> 100.
    The resulting high/low durations meet the WS2812 800 kHz timing window.
    """
    stream = bytearray()
    accumulator = 0
    pending = 0
    for red, green, blue in colors:
        for component in (green, red, blue):  # WS2812 wire order is GRB.
            for bit in range(7, -1, -1):
                accumulator = (accumulator << 3) | (0b110 if component & (1 << bit) else 0b100)
                pending += 3
                while pending >= 8:
                    shift = pending - 8
                    stream.append(accumulator >> shift)
                    accumulator &= (1 << shift) - 1
                    pending = shift
    if pending:
        stream.append(accumulator << (8 - pending))
    # Keep MOSI low for at least 80 us to latch the frame.
    stream.extend(b"\x00" * 24)
    return bytes(stream)


def write_frame(device: Path, payload: bytes, speed_hz: int) -> None:
    if not device.is_char_device():
        raise RuntimeError(
            f"未找到 SPI 设备节点：{device}\n"
            "本机 spi1.0 已被 spidev 绑定时，可由 root 执行：\n"
            f"  mknod {device} c 153 0 && chmod 666 {device}"
        )
    descriptor = os.open(device, os.O_WRONLY)
    try:
        fcntl.ioctl(descriptor, SPI_IOC_WR_MODE, bytes((SPI_MODE_0,)))
        fcntl.ioctl(descriptor, SPI_IOC_WR_BITS_PER_WORD, bytes((8,)))
        fcntl.ioctl(descriptor, SPI_IOC_WR_MAX_SPEED_HZ, speed_hz.to_bytes(4, "little"))
        written = os.write(descriptor, payload)
        if written != len(payload):
            raise RuntimeError(f"SPI 仅发送了 {written}/{len(payload)} 字节")
    finally:
        os.close(descriptor)


def main() -> int:
    parser = argparse.ArgumentParser(description="通过 SPI1 MOSI 点亮 WS2812 灯带")
    parser.add_argument("--device", default="/dev/spidev1.0", type=Path)
    parser.add_argument("--count", type=int, default=8, help="灯珠数量，默认 8")
    parser.add_argument("--color", type=parse_rgb, default=(16, 16, 16),
                        help="RGB，例如 32,0,0；默认低亮度白色")
    parser.add_argument("--brightness", type=float, default=0.10,
                        help="亮度比例 0 到 1，默认 0.10")
    parser.add_argument("--off", action="store_true", help="关闭全部灯珠")
    parser.add_argument("--dry-run", action="store_true", help="只打印编码结果，不发送 SPI")
    args = parser.parse_args()
    if not 1 <= args.count <= 1024:
        parser.error("--count 必须在 1 到 1024")
    if not 0.0 <= args.brightness <= 1.0:
        parser.error("--brightness 必须在 0 到 1")
    color = (0, 0, 0) if args.off else tuple(
        round(component * args.brightness) for component in args.color)
    payload = encode_ws2812([color] * args.count)
    if args.dry_run:
        print(f"SPI={SPI_HZ}Hz pixels={args.count} RGB={color} bytes={len(payload)}")
        print(payload.hex(" "))
        return 0
    write_frame(args.device, payload, SPI_HZ)
    state = "已关闭" if args.off else f"已点亮 RGB={color}"
    print(f"{state}：{args.count} 颗 WS2812，经 {args.device} / SPI1 MOSI")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
