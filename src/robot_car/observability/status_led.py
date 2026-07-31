"""Low-brightness WS2812 status display for the vehicle daemon."""

from __future__ import annotations

import fcntl
import logging
import os
import time
from pathlib import Path
from typing import Any, Callable, Iterable


LOG = logging.getLogger(__name__)

Color = tuple[int, int, int]
OFF: Color = (0, 0, 0)
RED: Color = (255, 0, 0)
GREEN: Color = (0, 255, 0)
BLUE: Color = (0, 0, 255)
# Use equal red/green channels. At the configured 1% brightness, a dim amber
# green component rounds to zero and becomes indistinguishable from red.
YELLOW: Color = (255, 255, 0)
PURPLE: Color = (180, 0, 255)
WHITE: Color = (255, 255, 255)

SPI_IOC_WR_MODE = 0x40016B01
SPI_IOC_WR_BITS_PER_WORD = 0x40016B03
SPI_IOC_WR_MAX_SPEED_HZ = 0x40046B04
SPI_HZ = 2_400_000


def encode_ws2812(colors: Iterable[Color]) -> bytes:
    """Encode RGB pixels for WS2812 as 2.4 MHz SPI mode-0 data."""
    stream = bytearray()
    accumulator = 0
    pending = 0
    for red, green, blue in colors:
        for component in (green, red, blue):
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
    # A zero-filled tail holds MOSI low for at least 80 us and latches the frame.
    stream.extend(b"\x00" * 24)
    return bytes(stream)


class SpiWs2812:
    """A small stdlib-only SPI writer; no RPi-specific package is required."""

    def __init__(self, device: str, count: int, brightness: float) -> None:
        self.device = Path(device)
        self.count = count
        self.brightness = brightness

    def write(self, colors: list[Color]) -> None:
        if len(colors) != self.count:
            raise ValueError("WS2812 pixel count does not match configuration")
        if not self.device.is_char_device():
            raise RuntimeError(f"WS2812 SPI device is unavailable: {self.device}")
        scaled = [tuple(round(channel * self.brightness) for channel in color) for color in colors]
        descriptor = os.open(self.device, os.O_WRONLY)
        try:
            fcntl.ioctl(descriptor, SPI_IOC_WR_MODE, bytes((0,)))
            fcntl.ioctl(descriptor, SPI_IOC_WR_BITS_PER_WORD, bytes((8,)))
            fcntl.ioctl(descriptor, SPI_IOC_WR_MAX_SPEED_HZ, SPI_HZ.to_bytes(4, "little"))
            payload = encode_ws2812(scaled)
            if os.write(descriptor, payload) != len(payload):
                raise RuntimeError("incomplete WS2812 SPI transfer")
        finally:
            os.close(descriptor)


class StatusLedController:
    """Render the eight operational states without participating in control decisions."""

    def __init__(self, config: dict[str, Any], *, writer: Callable[[list[Color]], None] | None = None) -> None:
        self.enabled = bool(config.get("enabled", False))
        self.count = int(config.get("count", 8))
        self.refresh_hz = float(config.get("refresh_hz", 8))
        self._next_update_s = 0.0
        self._failed = False
        brightness = float(config.get("brightness", 0.01))
        self._writer = writer or SpiWs2812(str(config.get("device", "/dev/spidev1.0")),
                                           self.count, brightness).write

    def update(self, state: dict[str, Any], now_s: float | None = None) -> None:
        if not self.enabled or self._failed:
            return
        now = time.monotonic() if now_s is None else now_s
        if now < self._next_update_s:
            return
        self._next_update_s = now + 1.0 / self.refresh_hz
        try:
            self._writer(self.render(state, now))
        except Exception as error:
            self._failed = True
            LOG.warning("status LED disabled after SPI error: %s", error)

    def close(self) -> None:
        if not self.enabled or self._failed:
            return
        try:
            self._writer([OFF] * self.count)
        except Exception:
            LOG.debug("could not clear status LEDs", exc_info=True)

    def render(self, state: dict[str, Any], now_s: float) -> list[Color]:
        """Return LEDs 0..7: system, UART, CAN, camera, ball, balance, action, safety."""
        action = state.get("action", {})
        error = str(state.get("ui_error", ""))
        recent_vision = bool(state.get("recent_vision", False))
        ball_error = action.get("ball_error_mm")
        active_action = action.get("action_id")
        action_status = str(action.get("status", "IDLE"))
        control_enabled = bool(state.get("control_enabled", False))
        balance_enabled = bool(state.get("balance_enabled", False))
        gateway = state.get("gateway", {})
        fault = bool(state.get("fault", False)) or bool(error)

        pixels: list[Color] = [GREEN if not fault else self._blink(RED, now_s, 4.0)]
        if int(gateway.get("decode_errors", 0)):
            pixels.append(self._blink(RED, now_s, 4.0))
        elif int(gateway.get("received", 0)):
            pixels.append(GREEN)
        else:
            pixels.append(self._breathe(YELLOW, now_s))

        if active_action == 1 and action_status == "RUNNING":
            pixels.append(self._breathe(YELLOW, now_s))
        elif "motor_home_failed" in error or action.get("reason") == "motor_home_failed":
            pixels.append(self._blink(RED, now_s, 4.0))
        else:
            pixels.append(OFF)

        if "camera" in error.lower() or "vision" in error.lower():
            pixels.append(self._blink(RED, now_s, 4.0))
        elif recent_vision:
            pixels.append(GREEN)
        else:
            pixels.append(self._breathe(YELLOW, now_s))

        if ball_error is None:
            pixels.append(OFF)
        elif abs(float(ball_error)) <= 10.0:
            pixels.append(BLUE)
        else:
            pixels.append(YELLOW)

        if not balance_enabled:
            pixels.append(OFF)
        elif ball_error is not None and abs(float(ball_error)) <= 10.0:
            pixels.append(GREEN)
        else:
            pixels.append(self._breathe(BLUE, now_s))

        if action_status == "RUNNING":
            pixels.append(self._breathe(PURPLE, now_s))
        elif action_status == "COMPLETE":
            pixels.append(self._counted_blink(GREEN, int(action.get("last_action_id") or 1), now_s))
        elif action_status == "FAILED":
            pixels.append(self._counted_blink(RED, int(action.get("last_action_id") or 1), now_s))
        else:
            pixels.append(OFF)

        if fault:
            pixels.append(RED)
        elif not control_enabled:
            pixels.append(YELLOW)
        else:
            pixels.append(GREEN)
        return pixels[:self.count]

    @staticmethod
    def _blink(color: Color, now_s: float, hz: float) -> Color:
        return color if int(now_s * hz * 2) % 2 == 0 else OFF

    @staticmethod
    def _breathe(color: Color, now_s: float) -> Color:
        return color if int(now_s * 2) % 2 == 0 else tuple(channel // 4 for channel in color)

    @staticmethod
    def _counted_blink(color: Color, count: int, now_s: float) -> Color:
        slot = int(now_s * 4) % 16
        return color if slot < min(12, count * 2) and slot % 2 == 0 else OFF
