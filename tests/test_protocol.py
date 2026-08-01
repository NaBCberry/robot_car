import time
import unittest

from robot_car.decision.capture_target import CaptureTarget
from robot_car.decision.balance_state import BalanceState
from robot_car.decision.motion_target import MotionTarget
from robot_car.protocol.framing import (CRC, HEADER, MAGIC, MAX_PAYLOAD, FrameDecoder,
                                        ProtocolError, crc16_ccitt, decode_packet, encode_frame)
from robot_car.protocol.messages import (MOTION_COMMON_STRUCT, MOTION_FLAG_ENABLED,
                                         MessageType, MotionMode, PROTOCOL_VERSION,
                                         ProtocolMessage, pack_ack, pack_action_request,
                                         pack_motion, unpack_ack, unpack_action_request,
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

    def test_payload_length_is_one_byte_with_a_255_byte_limit(self):
        message = ProtocolMessage(MessageType.CMD_EVENT, 0x7E, b"x" * MAX_PAYLOAD)
        frame = encode_frame(message)
        self.assertEqual(HEADER.size, 5)
        self.assertEqual(frame[6], MAX_PAYLOAD)
        self.assertEqual(FrameDecoder().feed(frame), [message])
        with self.assertRaisesRegex(ProtocolError, "payload too large"):
            encode_frame(ProtocolMessage(MessageType.CMD_EVENT, 0x7E, b"x" * (MAX_PAYLOAD + 1)))

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

    def test_roller_balance_round_trip(self):
        payload = pack_motion(
            MotionMode.BALANCE_ROLLER, True, 120, balance_error_mm=-45, balance_valid=True)
        self.assertEqual(unpack_motion(payload), {
            "mode": MotionMode.BALANCE_ROLLER,
            "enabled": True,
            "flags": 9,
            "valid_for_ms": 120,
            "balance_valid": True,
            "error_mm": -45,
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
        with self.assertRaisesRegex(ValueError, "balance_error_mm"):
            pack_motion(MotionMode.BALANCE_ROLLER, True, 120, balance_error_mm=32768,
                        balance_valid=True)

    def test_sequence_and_ack_matching(self):
        transport = FakeTransport()
        gateway = VehicleGateway(transport, {"control_enabled": False, "default_valid_for_ms": 200})
        gateway.open()
        sequence = gateway.send_motion(MotionTarget(mode=MotionMode.LINE_FOLLOW, enabled=True))
        ack = ProtocolMessage(MessageType.ACK, 20, pack_ack(sequence))
        transport.inject(encode_frame(ack))
        gateway.poll()
        self.assertEqual(gateway.stats["acks"], 1)
        self.assertEqual(gateway.stats_snapshot()["received_ack"], 1)
        self.assertEqual(gateway.stats_snapshot()["received_rate_hz"], 1)
        self.assertNotIn(sequence, gateway.pending)
        first_motion = FrameDecoder().feed(transport.sent[0])[0]
        self.assertEqual(unpack_motion(first_motion.payload)["mode"], MotionMode.DISABLED)
        self.assertFalse(unpack_motion(first_motion.payload)["enabled"])
        gateway.close()

    def test_ack_payload(self):
        self.assertEqual(unpack_ack(pack_ack(65535, 2)), {"acknowledged_sequence": 65535, "status": 2})

    def test_action_request_round_trip_and_validation(self):
        payload = pack_action_request(3, 128, {"target_mm": 50}, 1000)
        self.assertEqual(unpack_action_request(payload), {
            "action_id": 3, "request_id": 128, "parameters": {"target_mm": 50},
            "valid_for_ms": 1000,
        })
        with self.assertRaises(ValueError):
            pack_action_request(256, 1, {}, 1000)

    def test_gateway_queues_remote_action_once_and_acknowledges_duplicate(self):
        transport = FakeTransport()
        gateway = VehicleGateway(transport, {"control_enabled": False, "default_valid_for_ms": 200})
        gateway.open()
        remote = ProtocolMessage(MessageType.CMD_EVENT, 77,
                                  pack_action_request(2, 9, {"target_mm": -50}, 1000))
        frame = encode_frame(remote)
        transport.inject(frame + frame)
        gateway.poll()
        request = gateway.receive_action_request()
        self.assertEqual(request["action_id"], 2)
        self.assertIsNone(gateway.receive_action_request())
        replies = [FrameDecoder().feed(item)[0] for item in transport.sent[1:]]
        self.assertEqual([unpack_ack(item.payload)["status"] for item in replies], [0, 1])
        gateway.close()

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

    def test_gateway_output_only_keeps_coordinates_and_disables_motion(self):
        transport = FakeTransport()
        gateway = VehicleGateway(transport, {
            "control_enabled": True,
            "default_valid_for_ms": 200,
            "capture": {"enabled": True, "output_only": True},
        })
        gateway.open()
        now_ms = time.monotonic_ns() // 1_000_000
        target = CaptureTarget(now_ms, 17, -12000, 680, 940, 35, 200, True, True)
        gateway.send_motion(MotionTarget(MotionMode.CAPTURE_TARGET_POLAR, True, 200, target))
        decoded = unpack_motion(FrameDecoder().feed(transport.sent[-1])[0].payload)
        self.assertFalse(decoded["enabled"])
        self.assertTrue(decoded["target_valid"])
        self.assertEqual(decoded["bearing_mdeg"], -12000)
        self.assertEqual(decoded["range_mm"], 680)
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

    def test_gateway_sends_valid_roller_balance_state(self):
        transport = FakeTransport()
        gateway = VehicleGateway(transport, {
            "control_enabled": True,
            "default_valid_for_ms": 120,
            "balance": {"enabled": True},
        })
        gateway.open()
        now_ms = time.monotonic_ns() // 1_000_000
        state = BalanceState(now_ms, 25, 120, True)
        gateway.send_motion(MotionTarget(MotionMode.BALANCE_ROLLER, True, 120,
                                         balance_state=state))
        decoded = unpack_motion(FrameDecoder().feed(transport.sent[-1])[0].payload)
        self.assertEqual(decoded["mode"], MotionMode.BALANCE_ROLLER)
        self.assertTrue(decoded["balance_valid"])
        self.assertEqual(decoded["error_mm"], 25)
        gateway.close()

    def test_balance_output_only_sends_state_with_global_control_disabled(self):
        transport = FakeTransport()
        gateway = VehicleGateway(transport, {
            "control_enabled": False,
            "default_valid_for_ms": 120,
            "balance": {"enabled": True, "output_only": True},
        })
        gateway.open()
        now_ms = time.monotonic_ns() // 1_000_000
        state = BalanceState(now_ms, 25, 120, True)
        gateway.send_motion(MotionTarget(MotionMode.BALANCE_ROLLER, True, 120,
                                         balance_state=state))
        decoded = unpack_motion(FrameDecoder().feed(transport.sent[-1])[0].payload)
        self.assertFalse(decoded["enabled"])
        self.assertTrue(decoded["balance_valid"])
        self.assertEqual(decoded["error_mm"], 25)
        gateway.close()


if __name__ == "__main__":
    unittest.main()
