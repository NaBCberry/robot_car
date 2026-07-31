import unittest

from robot_car.roller_calibrate import invert_crank_samples, soft_limits_from_points


class RollerCalibrationTests(unittest.TestCase):
    def test_builds_inner_soft_limits_from_manual_endpoints(self):
        self.assertEqual(soft_limits_from_points(80, -90, 5), (-85, 75))

    def test_inverts_monotonic_motor_to_tube_measurements(self):
        points = invert_crank_samples([(-40, -4), (0, 0), (40, 5)])
        self.assertEqual(points, [[-4.0, -40.0], [0.0, 0.0], [5.0, 40.0]])

    def test_rejects_non_monotonic_crank_samples(self):
        with self.assertRaisesRegex(ValueError, "monotonic"):
            invert_crank_samples([(-40, -4), (0, 1), (40, -3)])
