"""Small adapter that reuses the existing Y42 SocketCAN implementation."""

from __future__ import annotations

from types import SimpleNamespace
from typing import Any, Callable


class Y42Actuator:
    def __init__(self, *, interface: str, address: int, firmware: str = "x",
                 packet_gap_ms: float = 3.0, dry_run: bool = False,
                 driver_factory: Callable[[str, bool], Any] | None = None,
                 payload_builder: Callable[[Any], tuple[int, bytes]] | None = None,
                 payload_sender: Callable[..., None] | None = None) -> None:
        if not interface or not 0 <= address <= 255 or firmware != "x" or packet_gap_ms < 0:
            raise ValueError("Y42 actuator configuration is invalid")
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
                      acceleration_rpm_s: int, deceleration_rpm_s: int) -> None:
        if speed_rpm <= 0 or acceleration_rpm_s < 0 or deceleration_rpm_s < 0:
            raise ValueError("Y42 trapezoid parameters are invalid")
        self._send("trapezoid", direction="cw" if motor_angle_deg >= 0 else "ccw",
                   acceleration=acceleration_rpm_s, deceleration=deceleration_rpm_s,
                   speed=speed_rpm, position=abs(motor_angle_deg), mode="absolute-zero")

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
