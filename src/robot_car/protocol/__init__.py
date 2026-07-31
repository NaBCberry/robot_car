"""MSPM0 protocol v2 codec."""

from .framing import FrameDecoder, ProtocolError, crc16_ccitt, encode_frame
from .messages import (MOTION_FLAG_CAPTURE_ARMED, MOTION_FLAG_ENABLED,
                       MOTION_FLAG_TARGET_VALID, MessageType, MotionMode, ProtocolMessage,
                       pack_action_request, pack_motion, unpack_action_request, unpack_motion)

__all__ = ["MOTION_FLAG_CAPTURE_ARMED", "MOTION_FLAG_ENABLED", "MOTION_FLAG_TARGET_VALID",
           "FrameDecoder", "MessageType", "MotionMode", "ProtocolError", "ProtocolMessage",
           "crc16_ccitt", "encode_frame", "pack_action_request", "pack_motion",
           "unpack_action_request", "unpack_motion"]
