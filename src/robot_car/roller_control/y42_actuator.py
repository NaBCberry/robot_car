"""Small adapter that reuses the existing Y42 SocketCAN implementation."""

from __future__ import annotations

from types import SimpleNamespace
import math
import time
from typing import Any, Callable


class Y42Actuator:
    def __init__(self, *, interface: str, address: int, firmware: str = "x",
                 packet_gap_ms: float = 3.0, dry_run: bool = False,
                 soft_limit_min_deg: float | None = None, soft_limit_max_deg: float | None = None,
                 driver_factory: Callable[[str, bool], Any] | None = None,
                 payload_builder: Callable[[Any], tuple[int, bytes]] | None = None,
                 payload_sender: Callable[..., None] | None = None) -> None:
        if not interface or not 0 <= address <= 255 or firmware != "x" or packet_gap_ms < 0:
            raise ValueError("Y42 actuator configuration is invalid")
        if (soft_limit_min_deg is None) != (soft_limit_max_deg is None):
            raise ValueError("both motor soft limits must be configured together")
        if soft_limit_min_deg is not None and (
                not math.isfinite(soft_limit_min_deg) or not math.isfinite(soft_limit_max_deg)
                or soft_limit_min_deg >= soft_limit_max_deg):
            raise ValueError("motor soft limits are invalid")
        if driver_factory is None or payload_builder is None or payload_sender is None:
            try:
                from canstep.can_driver import CANDriver, make_payload, send_payload
            except ImportError as error:
                raise RuntimeError("cannot import the local canstep Y42 driver") from error
            driver_factory = driver_factory or CANDriver
            payload_builder = payload_builder or make_payload
            payload_sender = payload_sender or send_payload
        self.address = address
        self.firmware = firmware
        self.packet_gap_ms = packet_gap_ms
        self.soft_limit_min_deg = soft_limit_min_deg
        self.soft_limit_max_deg = soft_limit_max_deg
        self.driver = driver_factory(interface, dry_run)
        self.payload_builder = payload_builder
        self.payload_sender = payload_sender
        self.enabled = False

    def open(self) -> None:
        self.driver.open()

    def enable(self) -> None:
        self._send("enable")
        self.enabled = True

    def disable(self) -> None:
        self._send("disable")
        self.enabled = False

    def stop(self) -> None:
        self._send("stop")

    def move_absolute(self, motor_angle_deg: float, *, speed_rpm: float,
                      acceleration_rpm_s: int, deceleration_rpm_s: int) -> float:
        if speed_rpm <= 0 or acceleration_rpm_s < 0 or deceleration_rpm_s < 0:
            raise ValueError("Y42 trapezoid parameters are invalid")
        bounded_angle = self._bound_angle(motor_angle_deg)
        self._send("trapezoid", direction="cw" if bounded_angle >= 0 else "ccw",
                   acceleration=acceleration_rpm_s, deceleration=deceleration_rpm_s,
                   speed=speed_rpm, position=abs(bounded_angle), mode="absolute-zero")
        return bounded_angle

    def read_position_deg(self, timeout_s: float = 2.0) -> float:
        data = self._read("position", 0x36, 7, timeout_s)
        position = int.from_bytes(data[2:6], "big") / 10.0
        return -position if data[1] else position

    def read_encoder_deg(self, timeout_s: float = 2.0) -> float:
        data = self._read("encoder", 0x31, 4, timeout_s)
        return int.from_bytes(data[1:3], "big") * 360.0 / 65536.0

    def close(self) -> None:
        try:
            if self.enabled:
                self.stop()
                self.disable()
        finally:
            self.driver.close()

    def _send(self, command: str, **kwargs: Any) -> None:
        arguments = SimpleNamespace(command=command, address=self.address, firmware=self.firmware,
                                    sync=False, **kwargs)
        address, payload = self.payload_builder(arguments)
        self.payload_sender(self.driver, address, payload, self.packet_gap_ms)

    def _read(self, item: str, code: int, length: int, timeout_s: float) -> bytes:
        if timeout_s <= 0 or getattr(self.driver, "dry_run", False):
            raise RuntimeError("cannot read motor state without a real CAN connection")
        arguments = SimpleNamespace(command="read", address=self.address, firmware=self.firmware,
                                    sync=False, item=item)
        address, payload = self.payload_builder(arguments)
        self.payload_sender(self.driver, address, payload, self.packet_gap_ms)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            received = self.driver.receive(max(0.001, deadline - time.monotonic()))
            if received is None:
                continue
            can_id, data, extended = received
            if (extended and can_id >> 8 == self.address and len(data) == length
                    and data[0] == code and data[-1] == 0x6B):
                return data
        raise RuntimeError("timed out reading Y42 motor state")

    def _bound_angle(self, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("motor angle must be finite")
        if self.soft_limit_min_deg is None:
            return value
        return max(self.soft_limit_min_deg, min(self.soft_limit_max_deg, value))
