import tempfile
import time
import unittest
from pathlib import Path

from robot_car.ipc.vision_socket import VisionEventPublisher, VisionEventSubscriber
from robot_car.perception.events import VisionEvent


class VisionIpcTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.path = str(Path(self.temporary.name) / "vision.sock")

    def tearDown(self):
        self.temporary.cleanup()

    def make_event(self, frame_id=1):
        return VisionEvent(1000, "test", "STOP", 0.9, {"reason": "test"}, frame_id, 150, 640, 480)

    def wait_connected(self, publisher, subscriber):
        deadline = time.monotonic() + 1.0
        self.assertTrue(subscriber.connect())
        while time.monotonic() < deadline:
            if publisher.client_count:
                return
            time.sleep(0.01)
        self.fail("subscriber did not connect")

    def test_subscribe_and_transfer(self):
        publisher = VisionEventPublisher(self.path)
        subscriber = VisionEventSubscriber(self.path, 0.01)
        publisher.start()
        try:
            self.wait_connected(publisher, subscriber)
            publisher.publish(self.make_event())
            self.assertEqual(subscriber.receive(0.5), self.make_event())
        finally:
            subscriber.close()
            publisher.close()

    def test_disconnect_and_reconnect(self):
        subscriber = VisionEventSubscriber(self.path, 0.01)
        first = VisionEventPublisher(self.path)
        first.start()
        self.wait_connected(first, subscriber)
        first.close()
        self.assertIsNone(subscriber.receive(0.05))
        second = VisionEventPublisher(self.path)
        second.start()
        try:
            self.wait_connected(second, subscriber)
            event = self.make_event(2)
            second.publish(event)
            self.assertEqual(subscriber.receive(0.5), event)
        finally:
            subscriber.close()
            second.close()


if __name__ == "__main__":
    unittest.main()
