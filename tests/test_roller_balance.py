import unittest

from robot_car.camera.frame import CameraFrame
from robot_car.perception.roller_balance_adapter import RollerBalanceAdapter
from robot_car.perception.roller_kinematics import RollerKinematicsEstimator


class RollerBalanceTests(unittest.TestCase):
    def test_kinematics_uses_timestamps_for_velocity_and_acceleration(self):
        estimator = RollerKinematicsEstimator(1.0, 1.0, 1.0)
        first = estimator.update(1000, 0.0)
        second = estimator.update(1100, 10.0)
        third = estimator.update(1200, 20.0)
        self.assertEqual(first.velocity_mm_s, 0.0)
        self.assertAlmostEqual(second.velocity_mm_s, 100.0)
        self.assertAlmostEqual(second.acceleration_mm_s2, 1000.0)
        self.assertAlmostEqual(third.position_mm, 20.0)
        self.assertAlmostEqual(third.velocity_mm_s, 100.0)
        self.assertAlmostEqual(third.acceleration_mm_s2, 0.0)

    def test_kinematics_resets_after_a_detection_gap(self):
        estimator = RollerKinematicsEstimator(max_gap_ms=100)
        estimator.update(1000, 10.0)
        state = estimator.update(1200, 30.0)
        self.assertEqual(state.position_mm, 30.0)
        self.assertEqual(state.velocity_mm_s, 0.0)
        self.assertEqual(state.acceleration_mm_s2, 0.0)

    def test_adapter_filters_to_tube_roi_and_emits_signed_state(self):
        adapter = RollerBalanceAdapter("roller", {"ttl_ms": 120, "config": {
            "target_class_id": 0,
            "roi_xyxy": [100, 40, 500, 180],
            "center_x_px": 300,
            "mm_per_pixel": 0.5,
            "axis_direction": 1,
            "target_mm": 0,
            "estimator": {"position_alpha": 1, "velocity_alpha": 1,
                          "acceleration_alpha": 1, "max_gap_ms": 250},
        }})
        adapter.roi = (100, 40, 500, 180)
        adapter.center_x_px = 300
        adapter.mm_per_pixel = 0.5
        adapter.axis_direction = 1
        adapter.target_mm = 0
        adapter.estimator = RollerKinematicsEstimator(1, 1, 1)
        selected = adapter._select_primary(
            [[10, 50, 30, 70], [380, 80, 420, 120]], [0.99, 0.80], [0, 0])
        self.assertEqual(selected[0], [380, 80, 420, 120])

        event = adapter._event_from_detection(
            CameraFrame(5, 1000, None, 1280, 720), selected[0], selected[1], selected[2])
        self.assertEqual(event.event_type, "BALL_BALANCE_STATE")
        self.assertEqual(event.payload["position_mm"], 50)
        self.assertEqual(event.payload["error_mm"], 50)
        self.assertEqual(event.payload["velocity_mm_s"], 0)
        self.assertEqual(event.payload["roller_roi_xyxy"], [100, 40, 500, 180])

    def test_adapter_rejects_incomplete_calibration(self):
        adapter = RollerBalanceAdapter("roller", {"config": {"roi_xyxy": [1, 2, 3, 4]}})
        with self.assertRaisesRegex(ValueError, "calibration is incomplete"):
            adapter.initialize()


if __name__ == "__main__":
    unittest.main()
