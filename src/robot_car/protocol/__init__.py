"""MSPM0 protocol v1 codec."""

from .framing import FrameDecoder, ProtocolError, crc16_ccitt, encode_frame
from .messages import (CAPTURE_TARGET_FLAG_ARMED, CAPTURE_TARGET_FLAG_VALID, MessageType,
                       ProtocolMessage, pack_capture_target, unpack_capture_target)

__all__ = ["CAPTURE_TARGET_FLAG_ARMED", "CAPTURE_TARGET_FLAG_VALID", "FrameDecoder",
           "MessageType", "ProtocolError", "ProtocolMessage", "crc16_ccitt", "encode_frame",
           "pack_capture_target", "unpack_capture_target"]
