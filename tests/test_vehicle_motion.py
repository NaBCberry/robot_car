import unittest

from robot_car.roller_control.vehicle_motion import VehicleMotionFeedforward


class VehicleMotionFeedforwardTests(unittest.TestCase):
    def configuration(self, **overrides):
        configuration = {
            "enabled": True,
            "telemetry_timeout_ms": 150,
            "pipe_axis_in_chassis": [1.0, 0.0],
            "pipe_acceleration_sign": 1,
            "wheel_track_mm": 100.0,
            "encoder_accel_alpha": 1.0,
            "imu_accel_correction_gain": 0.0,
            "imu_yaw_correction_gain": 0.0,
            "max_acceleration_mm_s2": 3000.0,
            "mpu6500": {"enabled": False},
        }
        configuration.update(overrides)
        return configuration

    def test_fused_speed_drives_longitudinal_acceleration(self):
        fusion = VehicleMotionFeedforward(self.configuration())
        first = fusion.update({"speed_mm_s": 100, "speed_left_mm_s": 100,
                               "speed_right_mm_s": 100}, 1000, 1000)
        second = fusion.update({"speed_mm_s": 140, "speed_left_mm_s": 140,
                                "speed_right_mm_s": 140}, 1020, 1020)

        self.assertEqual(first.longitudinal_mm_s2, 0.0)
        self.assertAlmostEqual(second.longitudinal_mm_s2, 2000.0)
        self.assertAlmostEqual(second.tube_axis_mm_s2, 2000.0)

    def test_wheel_difference_produces_lateral_turn_acceleration(self):
        fusion = VehicleMotionFeedforward(self.configuration(pipe_axis_in_chassis=[0.0, 1.0]))
        estimate = fusion.update({"speed_mm_s": 200, "speed_left_mm_s": 150,
                                  "speed_right_mm_s": 250}, 1000, 1000)

        self.assertAlmostEqual(estimate.lateral_mm_s2, 200.0)
        self.assertAlmostEqual(estimate.tube_axis_mm_s2, 200.0)

    def test_stale_or_missing_speed_disables_feedforward(self):
        fusion = VehicleMotionFeedforward(self.configuration())
        self.assertIsNone(fusion.update({"speed_mm_s": 100}, 1000, 1151))
        self.assertIsNone(fusion.update({"speed_left_mm_s": 100}, 1000, 1000))


if __name__ == "__main__":
    unittest.main()
