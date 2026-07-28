import time
import unittest

from robot_car.perception.events import VisionEvent
from robot_car.protocol.framing import FrameDecoder
from robot_car.protocol.messages import MessageType, MotionMode, unpack_motion
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
    def test_capture_servo_cycle_sends_one_polar_cmd_motion(self):
        now_ms = time.monotonic_ns() // 1_000_000
        config = {
            "runtime": {"vision_socket": "/tmp/not-used.sock"},
            "vehicle": {
                "control_enabled": True,
                "initial_mode": "LINE_FOLLOW",
                "default_valid_for_ms": 200,
                "heartbeat_hz": 1000,
                "vision_timeout_ms": 500,
                "link_timeout_ms": 500,
                "capture": {
                    "enabled": True,
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
        motions = [unpack_motion(message.payload) for message in messages
                   if message.message_type == MessageType.CMD_MOTION]
        polar = [motion for motion in motions if motion["mode"] == MotionMode.CAPTURE_TARGET_POLAR]
        self.assertEqual(len(polar), 1)
        self.assertTrue(polar[0]["target_valid"])
        self.assertEqual(polar[0]["range_mm"], 680)

    def test_transport_heartbeat_switch_suppresses_heartbeat_frames(self):
        now_ms = time.monotonic_ns() // 1_000_000
        config = {
            "runtime": {"vision_socket": "/tmp/not-used.sock"},
            "transport": {"heartbeat": {"enabled": False}},
            "vehicle": {
                "control_enabled": True,
                "initial_mode": "LINE_FOLLOW",
                "default_valid_for_ms": 200,
                "heartbeat_hz": 1000,
                "vision_timeout_ms": 500,
                "link_timeout_ms": 500,
                "capture": {"enabled": True, "target_timeout_ms": 200,
                            "feedback": {"enabled": False}},
            },
        }
        transport = FakeTransport()
        daemon = VehicleDaemon(config, transport)
        event = VisionEvent(now_ms, "steelball", "BALL_TARGET", 0.94, {
            "track_id": 17, "bearing_mdeg": -12000, "range_mm": 680,
        }, 8, 150, 1280, 720)
        daemon.subscriber = OneEventSubscriber(daemon, event)

        daemon.run()

        decoder = FrameDecoder()
        messages = [message for frame in transport.sent for message in decoder.feed(frame)]
        self.assertFalse(any(message.message_type == MessageType.HEARTBEAT for message in messages))

    def test_output_only_capture_sends_coordinates_without_mspm0_telemetry(self):
        now_ms = time.monotonic_ns() // 1_000_000
        config = {
            "runtime": {"vision_socket": "/tmp/not-used.sock"},
            "vehicle": {
                "control_enabled": True,
                "initial_mode": "IDLE",
                "default_valid_for_ms": 200,
                "heartbeat_hz": 1000,
                "vision_timeout_ms": 500,
                "link_timeout_ms": 500,
                "capture": {"enabled": True, "output_only": True,
                            "target_timeout_ms": 200, "feedback": {"enabled": False}},
            },
        }
        transport = FakeTransport()
        daemon = VehicleDaemon(config, transport)
        daemon.is_fake = False  # Model the no-reply UART/logic-analyzer test condition.
        event = VisionEvent(now_ms, "steelball", "BALL_TARGET", 0.94, {
            "track_id": 17, "bearing_mdeg": -12000, "range_mm": 680,
        }, 8, 150, 1280, 720)
        daemon.subscriber = OneEventSubscriber(daemon, event)

        daemon.run()

        decoder = FrameDecoder()
        messages = [message for frame in transport.sent for message in decoder.feed(frame)]
        polar = [unpack_motion(message.payload) for message in messages
                 if message.message_type == MessageType.CMD_MOTION
                 and unpack_motion(message.payload)["mode"] == MotionMode.CAPTURE_TARGET_POLAR]
        self.assertEqual(len(polar), 1)
        self.assertFalse(polar[0]["enabled"])
        self.assertTrue(polar[0]["target_valid"])
        self.assertEqual(polar[0]["bearing_mdeg"], -12000)
        self.assertEqual(polar[0]["range_mm"], 680)


if __name__ == "__main__":
    unittest.main()
