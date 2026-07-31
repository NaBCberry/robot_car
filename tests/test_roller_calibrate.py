import unittest
from unittest.mock import patch

from robot_car.roller_calibrate import finish_calibration, invert_crank_samples, soft_limits_from_points


class FinishingActuator:
    def __init__(self):
        self.calls = []

    def disable(self):
        self.calls.append("disable")

    def enable(self):
        self.calls.append("enable")


class RollerCalibrationTests(unittest.TestCase):
    def test_builds_inner_soft_limits_from_manual_endpoints(self):
        self.assertEqual(soft_limits_from_points(80, -90, 5), (-85, 75))

    def test_inverts_monotonic_motor_to_tube_measurements(self):
        points = invert_crank_samples([(-40, -4), (0, 0), (40, 5)])
        self.assertEqual(points, [[-4.0, -40.0], [0.0, 0.0], [5.0, 40.0]])

    def test_rejects_non_monotonic_crank_samples(self):
        with self.assertRaisesRegex(ValueError, "monotonic.*水管"):
            invert_crank_samples([(-40, -4), (0, 1), (40, -3)])

    def test_finish_calibration_can_leave_motor_loose(self):
        actuator = FinishingActuator()

        with patch("builtins.input", return_value="1"):
            self.assertFalse(finish_calibration(actuator, (-10, 10)))

        self.assertEqual(actuator.calls, ["disable"])

    def test_finish_calibration_can_keep_motor_enabled_without_homing(self):
        actuator = FinishingActuator()

        with patch("builtins.input", side_effect=("2", "1")):
            self.assertTrue(finish_calibration(actuator, (-10, 10)))

        self.assertEqual(actuator.calls, ["enable"])
