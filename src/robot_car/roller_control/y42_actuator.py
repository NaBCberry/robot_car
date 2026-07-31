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
                 pulses_per_revolution: int = 3200,
                 driver_factory: Callable[[str, bool], Any] | None = None,
                 payload_builder: Callable[[Any], tuple[int, bytes]] | None = None,
                 payload_sender: Callable[..., None] | None = None) -> None:
        if (not interface or not 0 <= address <= 255 or firmware not in ("x", "emm")
                or packet_gap_ms < 0 or pulses_per_revolution <= 0):
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
        self.pulses_per_revolution = pulses_per_revolution
        self.packet_gap_ms = packet_gap_ms
        self.soft_limit_min_deg = soft_limit_min_deg
        self.soft_limit_max_deg = soft_limit_max_deg
        self.driver = driver_factory(interface, dry_run)
        self.payload_builder = payload_builder
        self.payload_sender = payload_sender
        self.enabled = False

    def open(self) -> None:
        self.driver.open()

    def enable(self, *, confirm: bool = False) -> None:
        self._send("enable", confirm=confirm)
        self.enabled = True

    def disable(self) -> None:
        self._send("disable")
        self.enabled = False

    def stop(self) -> None:
        self._send("stop")

    def move_absolute(self, motor_angle_deg: float, *, speed_rpm: float,
                      acceleration_rpm_s: int, deceleration_rpm_s: int,
                      confirm: bool = False, confirm_timeout_s: float = 2.0) -> float:
        if speed_rpm <= 0 or acceleration_rpm_s < 0 or deceleration_rpm_s < 0:
            raise ValueError("Y42 trapezoid parameters are invalid")
        bounded_angle = self._bound_angle(motor_angle_deg)
        if self.firmware == "emm":
            self._send("position", direction="cw" if bounded_angle >= 0 else "ccw",
                       speed=speed_rpm, acceleration=acceleration_rpm_s,
                       position=self._degrees_to_pulses(abs(bounded_angle)), mode="absolute-zero",
                       confirm=confirm, confirm_timeout_s=confirm_timeout_s)
            return bounded_angle
        self._send("trapezoid", direction="cw" if bounded_angle >= 0 else "ccw",
                   acceleration=acceleration_rpm_s, deceleration=deceleration_rpm_s,
                   speed=speed_rpm, position=abs(bounded_angle), mode="absolute-zero",
                   confirm=confirm, confirm_timeout_s=confirm_timeout_s)
        return bounded_angle

    def move_to_coordinate(self, motor_angle_deg: float, *, current_angle_deg: float,
                           speed_rpm: float, acceleration_rpm_s: int,
                           deceleration_rpm_s: int, confirm: bool = False,
                           confirm_timeout_s: float = 2.0) -> float:
        if speed_rpm <= 0 or acceleration_rpm_s < 0 or deceleration_rpm_s < 0:
            raise ValueError("Y42 trapezoid parameters are invalid")
        if not math.isfinite(current_angle_deg):
            raise ValueError("current motor angle must be finite")
        bounded_angle = self._bound_angle(motor_angle_deg)
        delta = bounded_angle - current_angle_deg
        if self.firmware == "emm":
            self._send("position", direction="cw" if delta >= 0 else "ccw", speed=speed_rpm,
                       acceleration=acceleration_rpm_s, position=self._degrees_to_pulses(abs(delta)),
                       mode="relative-current", confirm=confirm, confirm_timeout_s=confirm_timeout_s)
        else:
            self._send("trapezoid", direction="cw" if delta >= 0 else "ccw",
                       acceleration=acceleration_rpm_s, deceleration=deceleration_rpm_s,
                       speed=speed_rpm, position=abs(delta), mode="relative-current", confirm=confirm,
                       confirm_timeout_s=confirm_timeout_s)
        return bounded_angle

    def move_relative_target(self, motor_angle_deg: float, *, speed_rpm: float,
                             acceleration_rpm_s: int, deceleration_rpm_s: int) -> None:
        if speed_rpm <= 0 or acceleration_rpm_s < 0 or deceleration_rpm_s < 0:
            raise ValueError("Y42 position parameters are invalid")
        if self.firmware != "emm":
            raise RuntimeError("relative-target calibration is only supported for EMM firmware")
        self._send("position", direction="cw" if motor_angle_deg >= 0 else "ccw", speed=speed_rpm,
                   acceleration=acceleration_rpm_s,
                   position=self._degrees_to_pulses(abs(motor_angle_deg)), mode="relative-target")

    def home_absolute_zero(self) -> None:
        self._send("home", mode=4)

    def read_position_deg(self, timeout_s: float = 2.0) -> float:
        return self._read_position_deg("position", 0x36, timeout_s)

    def read_target_position_deg(self, timeout_s: float = 2.0) -> float:
        return self._read_position_deg("target-position", 0x33, timeout_s)

    def _read_position_deg(self, item: str, code: int, timeout_s: float) -> float:
        data = self._read(item, code, 7, timeout_s)
        raw_position = int.from_bytes(data[2:6], "big")
        position = raw_position * 360.0 / 65536.0 if self.firmware == "emm" else raw_position / 10.0
        return -position if data[1] else position

    def read_encoder_deg(self, timeout_s: float = 2.0) -> float:
        data = self._read("encoder", 0x31, 4, timeout_s)
        return int.from_bytes(data[1:3], "big") * 360.0 / 65536.0

    def read_home_status(self, timeout_s: float = 2.0) -> int:
        return self._read("home-status", 0x3B, 3, timeout_s)[1]

    def read_motor_status(self, timeout_s: float = 2.0) -> int:
        return self._read("status", 0x3A, 3, timeout_s)[1]

    def close(self, *, disable: bool = True) -> None:
        try:
            if disable and self.enabled:
                self.stop()
                self.disable()
        finally:
            self.driver.close()

    def _send(self, command: str, **kwargs: Any) -> None:
        confirm = bool(kwargs.pop("confirm", False))
        confirm_timeout_s = float(kwargs.pop("confirm_timeout_s", 2.0))
        arguments = SimpleNamespace(command=command, address=self.address, firmware=self.firmware,
                                    sync=False, **kwargs)
        address, payload = self.payload_builder(arguments)
        # EMM position commands exceed one CAN frame. The motor requires the
        # function byte (FD) at the start of every continuation frame.
        self.payload_sender(self.driver, address, payload, self.packet_gap_ms, len(payload) > 8)
        if confirm:
            self._confirm_command(payload[0], timeout_s=confirm_timeout_s)

    def _confirm_command(self, code: int, timeout_s: float = 2.0) -> None:
        if getattr(self.driver, "dry_run", False):
            return
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            received = self.driver.receive(max(0.001, deadline - time.monotonic()))
            if received is None:
                continue
            can_id, data, extended = received
            if not (extended and can_id >> 8 == self.address and len(data) == 3
                    and data[0] == code and data[-1] == 0x6B):
                continue
            status = data[1]
            if status in (0x02, 0x9F):
                return
            if status in (0xE2, 0xEE):
                raise RuntimeError("Y42 command 0x%02X rejected: status 0x%02X" % (code, status))
        raise RuntimeError("Y42 command 0x%02X confirmation timed out" % code)

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

    def _degrees_to_pulses(self, degrees: float) -> int:
        return round(degrees * self.pulses_per_revolution / 360.0)
