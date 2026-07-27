"""Optional pyserial transport; device must be explicitly configured."""

from typing import Any, Dict, Optional

from .transport_base import Transport


class UartTransport(Transport):
    def __init__(self, config: Dict[str, Any]) -> None:
        self.config = config
        self.serial = None

    def open(self) -> None:
        device = self.config.get("device", "")
        if not device:
            raise ValueError("UART device is empty")
        try:
            import serial
        except ImportError as error:
            raise RuntimeError("pyserial is required for UART transport") from error
        self.serial = serial.Serial(device, int(self.config.get("baudrate", 115200)),
                                    timeout=float(self.config.get("read_timeout_ms", 20)) / 1000.0)

    def send(self, data: bytes) -> None:
        if self.serial is None:
            raise RuntimeError("UART is closed")
        self.serial.write(data)

    def receive(self, timeout: float = 0.0) -> Optional[bytes]:
        if self.serial is None:
            return None
        old_timeout = self.serial.timeout
        self.serial.timeout = timeout
        try:
            data = self.serial.read(4096)
            return data or None
        finally:
            self.serial.timeout = old_timeout

    def close(self) -> None:
        if self.serial is not None:
            self.serial.close()
            self.serial = None
