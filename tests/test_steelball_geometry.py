import unittest

from robot_car.camera.frame import CameraFrame
from robot_car.perception.steelball_adapter import SteelballAdapter
from robot_car.perception.steelball_geometry import ImageToCaptureProjector


class SteelballGeometryTests(unittest.TestCase):
    def test_adapter_prefers_lowest_ball_over_higher_confidence(self):
        adapter = SteelballAdapter("steelball", {"config": {"target_class_id": 0}})
        selected = adapter._select_primary(
            [[100, 100, 140, 180], [300, 250, 360, 400]],
            [0.99, 0.60],
            [0, 0],
        )
        self.assertEqual(selected[0], [300, 250, 360, 400])

    def test_identity_homography_projects_capture_coordinates(self):
        projector = ImageToCaptureProjector([1, 0, 0, 0, 1, 0, 0, 0, 1])
        target = projector.project(300, 400)
        self.assertEqual(target.forward_mm, 300)
        self.assertEqual(target.lateral_mm, 400)
        self.assertEqual(target.range_mm, 500)
        self.assertAlmostEqual(target.bearing_mdeg, 53130, delta=1)

    def test_steelball_event_requires_calibration_for_target_coordinates(self):
        frame = CameraFrame(8, 1000, None, 1280, 720)
        adapter = SteelballAdapter("steelball", {"enabled": True, "config": {}})
        event = adapter._event_from_detection(frame, [100, 200, 200, 300], 0.9, 0)
        self.assertEqual(event.event_type, "STEELBALL_DETECTED")
        self.assertNotIn("range_mm", event.payload)

    def test_steelball_event_uses_capture_point_homography(self):
        frame = CameraFrame(8, 1000, None, 1280, 720)
        adapter = SteelballAdapter("steelball", {"enabled": True, "config": {}})
        adapter.projector = ImageToCaptureProjector([1, 0, 0, 0, 1, 0, 0, 0, 1])
        event = adapter._event_from_detection(frame, [100, 200, 200, 300], 0.9, 0)
        self.assertEqual(event.event_type, "BALL_TARGET")
        self.assertEqual(event.payload["forward_mm"], 150)
        self.assertEqual(event.payload["lateral_mm"], 300)
        self.assertEqual(event.payload["range_mm"], 335)


if __name__ == "__main__":
    unittest.main()
