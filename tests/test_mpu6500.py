import struct
import unittest

from robot_car.roller_control.mpu6500 import (ACCEL_CONFIG, ACCEL_XOUT_H, CONFIG,
                                               GYRO_CONFIG, PWR_MGMT_1, SMPLRT_DIV,
                                               WHO_AM_I, Mpu6500)


class FakeI2cDevice:
    def __init__(self):
        self.opened = False
        self.closed = False
        self.writes = []

    def open(self, bus, address):
        self.opened = (bus, address)

    def close(self):
        self.closed = True

    def read_register(self, address, length=1):
        if address == WHO_AM_I:
            return b"\x70"
        if address == ACCEL_XOUT_H:
            return struct.pack(">hhhhhhh", 8192, -4096, 0, 0, 655, -655, 0)
        raise AssertionError(f"unexpected read register 0x{address:02X}")

    def write_register(self, address, value):
        self.writes.append((address, value))


class Mpu6500Tests(unittest.TestCase):
    def test_opens_configures_and_decodes_i2c_sample(self):
        device = FakeI2cDevice()
        sensor = Mpu6500(0, 0x68, device_factory=lambda: device)
        sensor.open()
        sensor.configure()
        sample = sensor.sample()
        sensor.close()

        self.assertEqual(device.opened, (0, 0x68))
        self.assertEqual(device.writes, [(PWR_MGMT_1, 0x01), (CONFIG, 0x03),
                                         (SMPLRT_DIV, 0x04), (GYRO_CONFIG, 0x08),
                                         (ACCEL_CONFIG, 0x08)])
        self.assertAlmostEqual(sample.accel_x_mm_s2, 9_806.65)
        self.assertAlmostEqual(sample.accel_y_mm_s2, -4_903.325)
        self.assertAlmostEqual(sample.gyro_x_deg_s, 10.0)
        self.assertAlmostEqual(sample.gyro_y_deg_s, -10.0)
        self.assertTrue(device.closed)


if __name__ == "__main__":
    unittest.main()
