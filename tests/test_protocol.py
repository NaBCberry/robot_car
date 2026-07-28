import time
import unittest

from robot_car.decision.capture_target import CaptureTarget
from robot_car.decision.motion_target import MotionTarget
from robot_car.protocol.framing import (CRC, HEADER, MAGIC, FrameDecoder, ProtocolError,
                                        crc16_ccitt, decode_packet, encode_frame)
from robot_car.protocol.messages import (MOTION_COMMON_STRUCT, MOTION_FLAG_ENABLED,
                                         MessageType, MotionMode, PROTOCOL_VERSION,
                                         ProtocolMessage, pack_ack, pack_motion, unpack_ack,
                                         unpack_motion)
from robot_car.vehicle_link.fake_transport import FakeTransport
from robot_car.vehicle_link.gateway import VehicleGateway


class ProtocolTests(unittest.TestCase):
    def test_round_trip_and_magic_bytes_in_payload(self):
        message = ProtocolMessage(MessageType.CMD_EVENT, 0x7E, b"a\xa5\x5ab")
        frame = encode_frame(message)
        self.assertTrue(frame.startswith(MAGIC))
        decoder = FrameDecoder()
        self.assertEqual(decoder.feed(frame[:5]), [])
        self.assertEqual(decoder.feed(frame[5:]), [message])

    def test_crc_failure_resynchronizes_to_next_magic_word(self):
        bad = bytearray(encode_frame(ProtocolMessage(MessageType.HEARTBEAT, 1, b"bad")))
        bad[-1] ^= 1
        good = ProtocolMessage(MessageType.HEARTBEAT, 2, b"good")
        decoder = FrameDecoder()
        self.assertEqual(decoder.feed(bytes(bad) + encode_frame(good)), [good])
        self.assertEqual(decoder.errors, 1)

    def test_crc_error_is_rejected(self):
        body = HEADER.pack(PROTOCOL_VERSION, int(MessageType.HEARTBEAT), 4, 1) + b"x"
        with self.assertRaisesRegex(ProtocolError, "CRC"):
            decode_packet(body + CRC.pack(crc16_ccitt(body) ^ 1))

    def test_v1_frame_is_rejected(self):
        body = HEADER.pack(1, int(MessageType.HEARTBEAT), 4, 0)
        with self.assertRaisesRegex(ProtocolError, "version"):
            decode_packet(body + CRC.pack(crc16_ccitt(body)))

    def test_invalid_length_is_rejected(self):
        body = HEADER.pack(PROTOCOL_VERSION, int(MessageType.HEARTBEAT), 4, 10) + b"x"
        with self.assertRaisesRegex(ProtocolError, "length"):
            decode_packet(body + CRC.pack(crc16_ccitt(body)))

    def test_unknown_message_type_is_rejected(self):
        body = HEADER.pack(PROTOCOL_VERSION, 0xFF, 4, 0)
        with self.assertRaisesRegex(ProtocolError, "unknown"):
            decode_packet(body + CRC.pack(crc16_ccitt(body)))

    def test_line_follow_has_only_the_common_semantic_header(self):
        payload = pack_motion(MotionMode.LINE_FOLLOW, True, 200)
        self.assertEqual(len(payload), MOTION_COMMON_STRUCT.size)
        self.assertEqual(unpack_motion(payload), {
            "mode": MotionMode.LINE_FOLLOW,
            "enabled": True,
            "flags": MOTION_FLAG_ENABLED,
            "valid_for_ms": 200,
        })

    def test_polar_capture_target_round_trip(self):
        payload = pack_motion(MotionMode.CAPTURE_TARGET_POLAR, True, 200, track_id=17,
                              bearing_mdeg=-12000, range_mm=680, confidence_permille=940,
                              measurement_age_ms=35, target_valid=True, capture_armed=True)
        self.assertEqual(unpack_motion(payload), {
            "mode": MotionMode.CAPTURE_TARGET_POLAR,
            "enabled": True,
            "flags": 7,
            "valid_for_ms": 200,
            "target_valid": True,
            "capture_armed": True,
            "track_id": 17,
            "bearing_mdeg": -12000,
            "range_mm": 680,
            "confidence_permille": 940,
            "measurement_age_ms": 35,
        })

    def test_motion_rejects_invalid_mode_fields(self):
        with self.assertRaises(ValueError):
            pack_motion(MotionMode.LINE_FOLLOW, True, 200, range_mm=1)
        with self.assertRaises(ValueError):
            pack_motion(MotionMode.CAPTURE_TARGET_POLAR, True, 200, track_id=1,
                        confidence_permille=1001, target_valid=True)
        with self.assertRaises(ValueError):
            unpack_motion(bytes((MotionMode.LINE_FOLLOW, 0x80, 0, 200)))
        with self.assertRaises(ValueError):
            unpack_motion(bytes((MotionMode.CAPTURE_TARGET_POLAR, 0, 0, 200)))

    def test_sequence_and_ack_matching(self):
        transport = FakeTransport()
        gateway = VehicleGateway(transport, {"control_enabled": False, "default_valid_for_ms": 200})
        gateway.open()
        sequence = gateway.send_motion(MotionTarget(mode=MotionMode.LINE_FOLLOW, enabled=True))
        ack = ProtocolMessage(MessageType.ACK, 20, pack_ack(sequence))
        transport.inject(encode_frame(ack))
        gateway.poll()
        self.assertEqual(gateway.stats["acks"], 1)
        self.assertNotIn(sequence, gateway.pending)
        first_motion = FrameDecoder().feed(transport.sent[0])[0]
        self.assertEqual(unpack_motion(first_motion.payload)["mode"], MotionMode.DISABLED)
        self.assertFalse(unpack_motion(first_motion.payload)["enabled"])
        gateway.close()

    def test_ack_payload(self):
        self.assertEqual(unpack_ack(pack_ack(65535, 2)), {"acknowledged_sequence": 65535, "status": 2})

    def test_gateway_polar_target_obeys_safety_gate(self):
        transport = FakeTransport()
        gateway = VehicleGateway(transport, {
            "control_enabled": True,
            "default_valid_for_ms": 200,
            "capture": {"enabled": True},
        })
        gateway.open()
        now_ms = time.monotonic_ns() // 1_000_000
        target = CaptureTarget(now_ms, 17, 12000, 680, 940, 35, 200, True, True)
        gateway.send_motion(MotionTarget(MotionMode.CAPTURE_TARGET_POLAR, True, 200, target))
        command = FrameDecoder().feed(transport.sent[-1])[0]
        self.assertEqual(command.message_type, MessageType.CMD_MOTION)
        self.assertTrue(unpack_motion(command.payload)["target_valid"])
        gateway.close()

    def test_gateway_expired_polar_target_becomes_explicit_safe_frame(self):
        transport = FakeTransport()
        gateway = VehicleGateway(transport, {
            "control_enabled": True,
            "default_valid_for_ms": 200,
            "capture": {"enabled": True},
        })
        gateway.open()
        target = CaptureTarget(1, 17, 12000, 680, 940, 35, 200, True, True)
        gateway.send_motion(MotionTarget(MotionMode.CAPTURE_TARGET_POLAR, True, 200, target))
        command = FrameDecoder().feed(transport.sent[-1])[0]
        decoded = unpack_motion(command.payload)
        self.assertEqual(decoded["mode"], MotionMode.CAPTURE_TARGET_POLAR)
        self.assertFalse(decoded["enabled"])
        self.assertFalse(decoded["target_valid"])
        gateway.close()


if __name__ == "__main__":
    unittest.main()
