import unittest
from unittest.mock import patch

from robot_car.roller_calibrate import (finish_calibration, invert_crank_samples,
                                        soft_limits_from_points, wait_for_target_position)


class FinishingActuator:
    def __init__(self):
        self.calls = []

    def disable(self):
        self.calls.append("disable")

    def enable(self):
        self.calls.append("enable")


class TargetActuator:
    def __init__(self, targets, status=0x01):
        self.targets = iter(targets)
        self.status = status

    def read_target_position_deg(self, timeout_s):
        value = next(self.targets)
        if isinstance(value, Exception):
            raise value
        return value

    def read_motor_status(self, timeout_s):
        return self.status


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

    def test_waits_for_emm_target_register_to_update(self):
        actuator = TargetActuator((0.055, -33.525))

        self.assertEqual(wait_for_target_position(actuator, -33.541), -33.525)

    def test_target_register_timeout_reports_last_readback_and_status(self):
        actuator = TargetActuator((RuntimeError("timed out reading Y42 motor state"),))

        with self.assertRaisesRegex(RuntimeError, "last target unavailable, status 0x01"):
            wait_for_target_position(actuator, 5, timeout_s=0.001)
