"""SLIP-like UART framing and CRC-16/CCITT validation."""

from __future__ import annotations

import struct
from typing import List

from .messages import MessageType, PROTOCOL_VERSION, ProtocolMessage


SOF = 0x7E
ESC = 0x7D
ESC_XOR = 0x20
HEADER = struct.Struct(">BBHH")
CRC = struct.Struct(">H")
MAX_PAYLOAD = 4096


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
    escaped = bytearray([SOF])
    for byte in raw:
        if byte in (SOF, ESC):
            escaped.extend((ESC, byte ^ ESC_XOR))
        else:
            escaped.append(byte)
    escaped.append(SOF)
    return bytes(escaped)


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
    """Incremental decoder; malformed packets are counted and discarded."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self._in_frame = False
        self._escaped = False
        self.errors = 0

    def feed(self, data: bytes) -> List[ProtocolMessage]:
        messages: List[ProtocolMessage] = []
        for byte in data:
            if byte == SOF:
                if self._in_frame and self._buffer:
                    try:
                        messages.append(decode_packet(bytes(self._buffer)))
                    except ProtocolError:
                        self.errors += 1
                self._buffer.clear()
                self._in_frame = True
                self._escaped = False
                continue
            if not self._in_frame:
                continue
            if self._escaped:
                self._buffer.append(byte ^ ESC_XOR)
                self._escaped = False
            elif byte == ESC:
                self._escaped = True
            elif len(self._buffer) <= MAX_PAYLOAD + HEADER.size + CRC.size:
                self._buffer.append(byte)
            else:
                self.errors += 1
                self._buffer.clear()
                self._in_frame = False
        return messages
