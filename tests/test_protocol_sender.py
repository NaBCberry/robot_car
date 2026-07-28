import unittest

from robot_car.protocol.messages import MessageType, unpack_json
from robot_car.protocol_sender import build_message


class ProtocolSenderTests(unittest.TestCase):
    def test_builds_custom_event_payload(self):
        message = build_message(None, None, "DIAGNOSTIC", '{"request":"status"}', 200, 7)
        self.assertEqual(message.message_type, MessageType.CMD_EVENT)
        self.assertEqual(message.sequence, 7)
        self.assertEqual(unpack_json(message.payload), {
            "event_type": "DIAGNOSTIC",
            "payload": {"request": "status"},
            "valid_for_ms": 200,
        })

    def test_builds_raw_payload(self):
        message = build_message(0x03, "000100c8", None, "{}", 200, 8)
        self.assertEqual(message.message_type, MessageType.HEARTBEAT)
        self.assertEqual(message.payload, b"\x00\x01\x00\xc8")

    def test_rejects_ambiguous_payload_input(self):
        with self.assertRaisesRegex(ValueError, "cannot be combined"):
            build_message(0x02, "00", "DIAGNOSTIC", "{}", 200, 1)


if __name__ == "__main__":
    unittest.main()
