import unittest

from robot_car.camera.frame import CameraFrame
from robot_car.perception.events import VisionEvent
from robot_car.perception.plugin import VisionPlugin
from robot_car.perception.scheduler import PluginScheduler


class ImmediatePlugin(VisionPlugin):
    def initialize(self):
        pass

    def process(self, frame):
        return [VisionEvent(frame.timestamp_monotonic_ms, self.name, "DETECTED", 1.0, {},
                            frame.frame_id, 100, frame.width, frame.height, False)]

    def health(self):
        return {}

    def close(self):
        pass


class PluginSchedulerTests(unittest.TestCase):
    def test_completed_plugin_starts_the_next_due_frame_without_an_idle_cycle(self):
        plugin = ImmediatePlugin("test", {"enabled": True, "interval_ms": 10})
        scheduler = PluginScheduler([plugin])
        frame = CameraFrame(1, 0, None, 640, 480)
        try:
            scheduler.process_latest(frame, 0)
            scheduler.slots[0].future.result(timeout=1)
            events, sources = scheduler.process_latest(frame, 10)
            self.assertEqual(sources, {"test"})
            self.assertEqual(len(events), 1)
            self.assertIsNotNone(scheduler.slots[0].future)
        finally:
            scheduler.close()

    def test_zero_interval_starts_the_next_frame_immediately(self):
        plugin = ImmediatePlugin("test", {"enabled": True, "interval_ms": 0,
                                           "max_processing_ms": 0})
        scheduler = PluginScheduler([plugin])
        frame = CameraFrame(1, 0, None, 640, 480)
        try:
            scheduler.process_latest(frame, 0)
            scheduler.slots[0].future.result(timeout=1)
            events, sources = scheduler.process_latest(frame, 0)
            self.assertEqual(sources, {"test"})
            self.assertEqual(len(events), 1)
            self.assertIsNotNone(scheduler.slots[0].future)
            self.assertEqual(scheduler.slots[0].max_processing_ms, 0)
        finally:
            scheduler.close()


if __name__ == "__main__":
    unittest.main()
