"""Length-delimited UART framing with an A5 5A sync word and CRC-16/CCITT."""

from __future__ import annotations

import struct
from typing import List

from .messages import MessageType, PROTOCOL_VERSION, ProtocolMessage


MAGIC = b"\xA5\x5A"
HEADER = struct.Struct(">BBHB")
CRC = struct.Struct(">H")
MAX_PAYLOAD = 0xFF


class ProtocolError(ValueError):
    pass


def crc16_ccitt(data: bytes, initial: int = 0xFFFF) -> int:
    crc = initial
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_frame(message: ProtocolMessage) -> bytes:
    if len(message.payload) > MAX_PAYLOAD:
        raise ProtocolError("payload too large")
    raw = HEADER.pack(message.version, int(message.message_type), message.sequence, len(message.payload)) + message.payload
    raw += CRC.pack(crc16_ccitt(raw))
    return MAGIC + raw


def decode_packet(packet: bytes) -> ProtocolMessage:
    if len(packet) < HEADER.size + CRC.size:
        raise ProtocolError("frame too short")
    body, received_crc = packet[:-CRC.size], CRC.unpack(packet[-CRC.size:])[0]
    if crc16_ccitt(body) != received_crc:
        raise ProtocolError("CRC mismatch")
    version, raw_type, sequence, payload_length = HEADER.unpack(body[:HEADER.size])
    if version != PROTOCOL_VERSION:
        raise ProtocolError(f"unsupported protocol version: {version}")
    if payload_length > MAX_PAYLOAD or len(body) != HEADER.size + payload_length:
        raise ProtocolError("invalid payload length")
    try:
        message_type = MessageType(raw_type)
    except ValueError as error:
        raise ProtocolError(f"unknown message type: {raw_type}") from error
    return ProtocolMessage(message_type, sequence, body[HEADER.size:], version)


class FrameDecoder:
    """Incremental length-based decoder; malformed candidates are resynchronized."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.errors = 0

    def feed(self, data: bytes) -> List[ProtocolMessage]:
        self._buffer.extend(data)
        messages: List[ProtocolMessage] = []
        minimum_size = len(MAGIC) + HEADER.size + CRC.size
        while True:
            start = self._buffer.find(MAGIC)
            if start < 0:
                # Retain a possible first sync byte across receive calls.
                if self._buffer[-1:] == MAGIC[:1]:
                    del self._buffer[:-1]
                else:
                    self._buffer.clear()
                return messages
            if start:
                del self._buffer[:start]
            if len(self._buffer) < minimum_size:
                return messages
            header_start = len(MAGIC)
            _, _, _, payload_length = HEADER.unpack(
                self._buffer[header_start:header_start + HEADER.size])
            if payload_length > MAX_PAYLOAD:
                self.errors += 1
                del self._buffer[0]
                continue
            frame_size = len(MAGIC) + HEADER.size + payload_length + CRC.size
            if len(self._buffer) < frame_size:
                return messages
            try:
                messages.append(decode_packet(bytes(self._buffer[len(MAGIC):frame_size])))
            except ProtocolError:
                self.errors += 1
                # A valid sync word may follow corrupted bytes.
                del self._buffer[0]
            else:
                del self._buffer[:frame_size]
        return messages
