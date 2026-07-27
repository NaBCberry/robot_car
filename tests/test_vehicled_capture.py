import time
import unittest

from robot_car.perception.events import VisionEvent
from robot_car.protocol.framing import FrameDecoder
from robot_car.protocol.messages import MessageType, unpack_capture_target, unpack_motion
from robot_car.vehicle_link.fake_transport import FakeTransport
from robot_car.vehicled import VehicleDaemon


class OneEventSubscriber:
    def __init__(self, daemon: VehicleDaemon, event: VisionEvent) -> None:
        self.daemon = daemon
        self.event = event
        self.delivered = False

    def receive(self, timeout: float):
        if not self.delivered:
            self.delivered = True
            return self.event
        self.daemon.stop_event.set()
        return None

    def close(self) -> None:
        pass


class VehicleDaemonCaptureTests(unittest.TestCase):
    def test_capture_servo_cycle_does_not_send_enabled_motion_command(self):
        now_ms = time.monotonic_ns() // 1_000_000
        config = {
            "runtime": {"vision_socket": "/tmp/not-used.sock"},
            "vehicle": {
                "control_enabled": True,
                "initial_mode": "LINE_FOLLOW",
                "line_follow_speed_mm_s": 200,
                "default_valid_for_ms": 200,
                "heartbeat_hz": 1000,
                "vision_timeout_ms": 500,
                "link_timeout_ms": 500,
                "capture": {
                    "enabled": True,
                    "control_mode": "MCU_TARGET_SERVO",
                    "target_timeout_ms": 200,
                    "feedback": {"enabled": False},
                },
            },
        }
        transport = FakeTransport()
        daemon = VehicleDaemon(config, transport)
        event = VisionEvent(now_ms, "steelball", "BALL_TARGET", 0.94, {
            "track_id": 17, "bearing_mdeg": -12000, "range_mm": 680,
        }, 8, 150, 1280, 720)
        daemon.subscriber = OneEventSubscriber(daemon, event)

        daemon.run()

        messages = []
        decoder = FrameDecoder()
        for frame in transport.sent:
            messages.extend(decoder.feed(frame))
        capture_messages = [message for message in messages
                            if message.message_type == MessageType.CMD_CAPTURE_TARGET]
        self.assertEqual(len(capture_messages), 1)
        self.assertTrue(unpack_capture_target(capture_messages[0].payload)["target_valid"])
        motions = [unpack_motion(message.payload) for message in messages
                   if message.message_type == MessageType.CMD_MOTION]
        self.assertTrue(motions)
        self.assertTrue(all(not motion["enable"] for motion in motions))


if __name__ == "__main__":
    unittest.main()
