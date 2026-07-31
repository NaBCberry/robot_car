import unittest

from robot_car.roller_control.config import build_controller


def configuration():
    return {
        "motor": {"soft_limit_min_deg": -50, "soft_limit_max_deg": 50},
        "control": {
            "tilt_sign": 1,
            "limits": {"target_min_mm": -10, "target_max_mm": 10,
                       "tube_angle_min_deg": -5, "tube_angle_max_deg": 5},
            "position_pid": {"kp": 1, "ki": 0, "kd": 0, "output_limit": 5, "integral_limit": 1},
            "velocity_pid": {"kp": 1, "ki": 0, "kd": 0, "output_limit": 5, "integral_limit": 1},
            "angle_pid": {"kp": 1, "ki": 0, "kd": 0, "output_limit": 5, "integral_limit": 1},
            "slope_bias_deg_by_position": [[-10, 0], [10, 0]],
            "motor_deg_by_tube_angle": [[-5, -50], [5, 50]],
        }
    }


class RollerControlConfigTests(unittest.TestCase):
    def test_builds_controller_from_isolated_configuration(self):
        controller = build_controller(configuration())
        self.assertEqual(controller.target_min_mm, -10)

    def test_rejects_incomplete_control_mapping(self):
        with self.assertRaisesRegex(ValueError, "position_pid"):
            build_controller({"motor": {"soft_limit_min_deg": -1, "soft_limit_max_deg": 1},
                              "control": {"limits": {}}})


if __name__ == "__main__":
    unittest.main()
