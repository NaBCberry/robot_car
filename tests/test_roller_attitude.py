import unittest

from robot_car.roller_control.attitude import PitchEstimator
from robot_car.roller_control.icm42688 import ImuSample


def sample(timestamp, accel_x=0, accel_z=8192, gyro_y=0):
    return ImuSample(timestamp, 0, accel_x, 0, accel_z, 0, gyro_y, 0)


class PitchEstimatorTests(unittest.TestCase):
    def test_uses_gravity_for_initial_pitch(self):
        estimator = PitchEstimator(slope_accel_axis="x", gravity_accel_axis="z", gyro_axis="y")
        estimate = estimator.update(sample(1.0, accel_x=8192, accel_z=8192))
        self.assertAlmostEqual(estimate.pitch_deg, 45.0)

    def test_integrates_gyro_between_acceleration_corrections(self):
        estimator = PitchEstimator(slope_accel_axis="x", gravity_accel_axis="z", gyro_axis="y",
                                   gyro_weight=1.0)
        estimator.update(sample(1.0))
        estimate = estimator.update(sample(1.1, gyro_y=655))
        self.assertAlmostEqual(estimate.pitch_deg, 1.0, places=3)


if __name__ == "__main__":
    unittest.main()
