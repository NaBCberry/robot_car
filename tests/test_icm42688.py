import unittest

from robot_car.roller_control.icm42688 import (ACCEL_CONFIG0, ACCEL_DATA_X1,
                                                GYRO_CONFIG0, PWR_MGMT0, Icm42688, WHO_AM_I)


class FakeSpi:
    def __init__(self, identity=0x47, sample=None):
        self.identity = identity
        self.sample = sample or [0] * 12
        self.max_speed_hz = 0
        self.mode = None
        self.opened = None
        self.closed = False
        self.writes = []

    def open(self, bus, chip_select):
        self.opened = (bus, chip_select)

    def close(self):
        self.closed = True

    def xfer2(self, values):
        if values == [WHO_AM_I | 0x80, 0]:
            return [0, self.identity]
        if values == [ACCEL_DATA_X1 | 0x80] + [0] * 12:
            return [0] + self.sample
        self.writes.append(values)
        return [0] * len(values)


class Icm42688Tests(unittest.TestCase):
    def test_open_verifies_identity_and_reads_sample(self):
        fake = FakeSpi(sample=[0, 1, 0, 2, 0, 3, 255, 252, 0, 5, 255, 250])
        sensor = Icm42688(1, 0, mode=3, spi_factory=lambda: fake, monotonic=lambda: 12.5)
        sensor.open()
        sample = sensor.sample()
        self.assertEqual(fake.opened, (1, 0))
        self.assertEqual(fake.max_speed_hz, 1_000_000)
        self.assertEqual(fake.mode, 3)
        self.assertEqual(sample.timestamp_s, 12.5)
        self.assertEqual((sample.temperature_raw, sample.accel_z_raw, sample.gyro_y_raw),
                         (0, 3, 5))

    def test_configures_sensor_after_identity_check(self):
        fake = FakeSpi()
        sensor = Icm42688(1, 1, spi_factory=lambda: fake)
        sensor.open()
        sensor.configure()
        self.assertEqual(fake.writes, [[PWR_MGMT0, 0x0F], [ACCEL_CONFIG0, 0x48],
                                       [GYRO_CONFIG0, 0x48]])

    def test_rejects_missing_sensor(self):
        fake = FakeSpi(identity=0)
        sensor = Icm42688(1, 0, spi_factory=lambda: fake)
        with self.assertRaisesRegex(RuntimeError, "WHO_AM_I=0x00"):
            sensor.open()
        self.assertTrue(fake.closed)


if __name__ == "__main__":
    unittest.main()
