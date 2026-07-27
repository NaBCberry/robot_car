import struct
import time
import unittest

from robot_car.decision.motion_target import MotionTarget
from robot_car.decision.capture_target import CaptureTarget
from robot_car.protocol.framing import (CRC, HEADER, FrameDecoder, ProtocolError, crc16_ccitt,
                                        decode_packet, encode_frame)
from robot_car.protocol.messages import (MessageType, ProtocolMessage, pack_ack, unpack_ack,
                                         pack_capture_target, unpack_capture_target, unpack_motion)
from robot_car.vehicle_link.fake_transport import FakeTransport
from robot_car.vehicle_link.gateway import VehicleGateway


class ProtocolTests(unittest.TestCase):
    def test_round_trip_and_escaped_bytes(self):
        message = ProtocolMessage(MessageType.CMD_EVENT, 0x7E, b"a\x7d\x7eb")
        self.assertEqual(FrameDecoder().feed(encode_frame(message)), [message])

    def test_crc_error_is_rejected(self):
        body = HEADER.pack(1, int(MessageType.HEARTBEAT), 4, 1) + b"x"
        with self.assertRaisesRegex(ProtocolError, "CRC"):
            decode_packet(body + CRC.pack(crc16_ccitt(body) ^ 1))

    def test_invalid_length_is_rejected(self):
        body = HEADER.pack(1, int(MessageType.HEARTBEAT), 4, 10) + b"x"
        with self.assertRaisesRegex(ProtocolError, "length"):
            decode_packet(body + CRC.pack(crc16_ccitt(body)))

    def test_unknown_message_type_is_rejected(self):
        body = HEADER.pack(1, 0xFF, 4, 0)
        with self.assertRaisesRegex(ProtocolError, "unknown"):
            decode_packet(body + CRC.pack(crc16_ccitt(body)))

    def test_sequence_and_ack_matching(self):
        transport = FakeTransport()
        gateway = VehicleGateway(transport, {"control_enabled": False, "default_valid_for_ms": 200})
        gateway.open()
        sequence = gateway.send_motion(MotionTarget(enable=True, target_speed_mm_s=500))
        ack = ProtocolMessage(MessageType.ACK, 20, pack_ack(sequence))
        transport.inject(encode_frame(ack))
        gateway.poll()
        self.assertEqual(gateway.stats["acks"], 1)
        self.assertNotIn(sequence, gateway.pending)
        first_motion = FrameDecoder().feed(transport.sent[0])[0]
        self.assertFalse(unpack_motion(first_motion.payload)["enable"])
        gateway.close()

    def test_ack_payload(self):
        self.assertEqual(unpack_ack(pack_ack(65535, 2)), {"acknowledged_sequence": 65535, "status": 2})

    def test_capture_target_round_trip(self):
        payload = pack_capture_target(0x03, 17, -12000, 680, 940, 35, 200)
        self.assertEqual(unpack_capture_target(payload), {
            "target_valid": True,
            "capture_armed": True,
            "flags": 3,
            "track_id": 17,
            "bearing_mdeg": -12000,
            "range_mm": 680,
            "confidence_permille": 940,
            "measurement_age_ms": 35,
            "valid_for_ms": 200,
        })

    def test_capture_target_rejects_invalid_values(self):
        with self.assertRaises(ValueError):
            pack_capture_target(1, 1, 0, 0, 1001, 0, 200)
        with self.assertRaises(ValueError):
            unpack_capture_target(b"\x00")

    def test_gateway_capture_target_obeys_safety_gate(self):
        transport = FakeTransport()
        gateway = VehicleGateway(transport, {
            "control_enabled": True,
            "default_valid_for_ms": 200,
            "capture": {"enabled": True, "control_mode": "MCU_TARGET_SERVO"},
        })
        gateway.open()
        now_ms = time.monotonic_ns() // 1_000_000
        gateway.send_capture_target(CaptureTarget(now_ms, 17, 12000, 680, 940, 35, 200, True, True))
        command = FrameDecoder().feed(transport.sent[-1])[0]
        self.assertEqual(command.message_type, MessageType.CMD_CAPTURE_TARGET)
        self.assertTrue(unpack_capture_target(command.payload)["target_valid"])
        gateway.close()

    def test_gateway_capture_target_expires_to_safe_frame(self):
        transport = FakeTransport()
        gateway = VehicleGateway(transport, {
            "control_enabled": True,
            "default_valid_for_ms": 200,
            "capture": {"enabled": True, "control_mode": "MCU_TARGET_SERVO"},
        })
        gateway.open()
        gateway.send_capture_target(CaptureTarget(1, 17, 12000, 680, 940, 35, 200, True, True))
        command = FrameDecoder().feed(transport.sent[-1])[0]
        self.assertFalse(unpack_capture_target(command.payload)["target_valid"])
        gateway.close()


if __name__ == "__main__":
    unittest.main()
