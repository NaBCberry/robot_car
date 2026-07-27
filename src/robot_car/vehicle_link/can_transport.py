"""Optional python-can transport with deployment-supplied identifiers."""

from typing import Any, Dict, Optional

from .transport_base import Transport


class CanTransport(Transport):
    def __init__(self, config: Dict[str, Any]) -> None:
        self.config = config
        self.bus = None
        self.can_module = None
        self._receive_buffer = bytearray()
        self._receive_index = 0

    def open(self) -> None:
        if not self.config.get("channel"):
            raise ValueError("CAN channel is empty")
        if self.config.get("ids", {}).get("command") is None:
            raise ValueError("CAN command ID is not configured")
        try:
            import can
        except ImportError as error:
            raise RuntimeError("python-can is required for CAN transport") from error
        self.can_module = can
        self.bus = can.Bus(interface=self.config.get("interface") or None,
                           channel=self.config["channel"], bitrate=int(self.config.get("bitrate", 500000)))

    def send(self, data: bytes) -> None:
        if self.bus is None or self.can_module is None:
            raise RuntimeError("CAN is closed")
        chunks = [data[offset:offset + 7] for offset in range(0, len(data), 7)] or [b""]
        if len(chunks) > 64:
            raise RuntimeError("protocol frame exceeds CAN segmentation limit")
        for index, chunk in enumerate(chunks):
            marker = index & 0x3F
            if index == 0:
                marker |= 0x80
            if index == len(chunks) - 1:
                marker |= 0x40
            message = self.can_module.Message(arbitration_id=int(self.config["ids"]["command"]),
                                              data=bytes([marker]) + chunk)
            self.bus.send(message)

    def receive(self, timeout: float = 0.0) -> Optional[bytes]:
        if self.bus is None:
            return None
        message = self.bus.recv(timeout)
        if message is None:
            return None
        telemetry_id = self.config.get("ids", {}).get("telemetry")
        if telemetry_id is not None and message.arbitration_id != int(telemetry_id):
            return None
        data = bytes(message.data)
        if not data:
            return None
        marker, chunk = data[0], data[1:]
        index = marker & 0x3F
        if marker & 0x80:
            self._receive_buffer.clear()
            self._receive_index = 0
        if index != self._receive_index:
            self._receive_buffer.clear()
            self._receive_index = 0
            return None
        self._receive_buffer.extend(chunk)
        self._receive_index += 1
        if marker & 0x40:
            result = bytes(self._receive_buffer)
            self._receive_buffer.clear()
            self._receive_index = 0
            return result
        return None

    def close(self) -> None:
        if self.bus is not None:
            self.bus.shutdown()
            self.bus = None
