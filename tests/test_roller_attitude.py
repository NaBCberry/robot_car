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

    def test_subtracts_mechanical_zero_offset(self):
        estimator = PitchEstimator(slope_accel_axis="z", gravity_accel_axis="y", gyro_axis="x",
                                   gyro_weight=0.0, pitch_zero_offset_deg=-5.0)
        estimate = estimator.update(ImuSample(1.0, 0, 0, 8192, -717, 0, 0, 0))
        self.assertAlmostEqual(estimate.pitch_deg, 0.0, places=1)


if __name__ == "__main__":
    unittest.main()
