import unittest

from robot_car.camera.frame import CameraFrame
from robot_car.perception.roller_balance_adapter import RollerBalanceAdapter


class RollerBalanceTests(unittest.TestCase):
    def test_adapter_filters_to_tube_roi_and_emits_signed_error(self):
        adapter = RollerBalanceAdapter("roller", {"ttl_ms": 120, "config": {
            "target_class_id": 0,
            "roi_xyxy": [100, 40, 500, 180],
            "center_x_px": 300,
            "mm_per_pixel": 0.5,
            "axis_direction": 1,
        }})
        adapter.roi = (100, 40, 500, 180)
        adapter.center_x_px = 300
        adapter.mm_per_pixel = 0.5
        adapter.axis_direction = 1
        selected = adapter._select_primary(
            [[10, 50, 30, 70], [380, 80, 420, 120]], [0.99, 0.80], [0, 0])
        self.assertEqual(selected[0], [380, 80, 420, 120])

        event = adapter._event_from_detection(
            CameraFrame(5, 1000, None, 1280, 720), selected[0], selected[1], selected[2])
        self.assertEqual(event.event_type, "BALL_BALANCE_STATE")
        self.assertEqual(event.payload["error_mm"], 50)
        self.assertNotIn("velocity_mm_s", event.payload)
        self.assertEqual(event.payload["roller_roi_xyxy"], [100, 40, 500, 180])

    def test_adapter_rejects_incomplete_calibration(self):
        adapter = RollerBalanceAdapter("roller", {"config": {"roi_xyxy": [1, 2, 3, 4]}})
        with self.assertRaisesRegex(ValueError, "calibration is incomplete"):
            adapter.initialize()

    def test_adapter_interpolates_between_scale_ticks_and_clamps_endpoints(self):
        adapter = RollerBalanceAdapter("roller", {"config": {}})
        adapter.axis_points = ((100.0, -120.0), (300.0, 0.0), (700.0, 120.0))
        self.assertEqual(adapter._position_mm(200.0), -60.0)
        self.assertEqual(adapter._position_mm(500.0), 60.0)
        self.assertEqual(adapter._position_mm(50.0), -120.0)
        self.assertEqual(adapter._position_mm(800.0), 120.0)


if __name__ == "__main__":
    unittest.main()
