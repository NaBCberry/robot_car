"""Minimal MPU6500 I2C reader for chassis-motion feedforward."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time

from .icm42688 import I2cDevice, LinuxI2cDevice, TRANSIENT_I2C_ERRNOS


WHO_AM_I = 0x75
WHO_AM_I_MPU6500 = 0x70
PWR_MGMT_1 = 0x6B
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
SMPLRT_DIV = 0x19
ACCEL_XOUT_H = 0x3B


@dataclass(frozen=True)
class Mpu6500Sample:
    timestamp_s: float
    accel_x_mm_s2: float
    accel_y_mm_s2: float
    accel_z_mm_s2: float
    gyro_x_deg_s: float
    gyro_y_deg_s: float
    gyro_z_deg_s: float

    def acceleration(self, axis: str) -> float:
        return getattr(self, f"accel_{axis}_mm_s2")

    def gyro(self, axis: str) -> float:
        return getattr(self, f"gyro_{axis}_deg_s")


class Mpu6500:
    """Read a chassis-mounted MPU6500 configured for +/-4 g and +/-500 dps."""

    def __init__(self, bus: int, address: int, *, retries: int = 2,
                 retry_delay_ms: int = 2, device_factory=LinuxI2cDevice) -> None:
        if not 0x03 <= address <= 0x77:
            raise ValueError("MPU6500 I2C address is invalid")
        if retries < 0 or retry_delay_ms < 0:
            raise ValueError("MPU6500 retry settings must not be negative")
        self.bus = int(bus)
        self.address = int(address)
        self.retries = int(retries)
        self.retry_delay_s = retry_delay_ms / 1000.0
        self.device_factory = device_factory
        self.device: I2cDevice | None = None

    def open(self) -> None:
        if self.device is not None:
            return
        device = self.device_factory()
        try:
            device.open(self.bus, self.address)
            identity = self._read_register(device, WHO_AM_I, 1)[0]
            if identity != WHO_AM_I_MPU6500:
                raise RuntimeError(
                    "MPU6500 not found on i2c-%d address 0x%02X: WHO_AM_I=0x%02X (expected 0x70)"
                    % (self.bus, self.address, identity))
        except Exception:
            device.close()
            raise
        self.device = device

    def configure(self) -> None:
        device = self._require_device()
        device.write_register(PWR_MGMT_1, 0x01)  # PLL clock, wake up.
        time.sleep(0.01)
        device.write_register(CONFIG, 0x03)      # 44 Hz gyro DLPF.
        device.write_register(SMPLRT_DIV, 0x04)  # 200 Hz sample rate.
        device.write_register(GYRO_CONFIG, 0x08)  # +/-500 dps, 65.5 LSB/(deg/s).
        device.write_register(ACCEL_CONFIG, 0x08)  # +/-4 g, 8192 LSB/g.

    def sample(self) -> Mpu6500Sample:
        import struct

        raw = self._read_register(self._require_device(), ACCEL_XOUT_H, 14)
        ax, ay, az, _temperature, gx, gy, gz = struct.unpack(">hhhhhhh", raw)
        gravity_mm_s2 = 9_806.65
        return Mpu6500Sample(
            time.monotonic(), ax * gravity_mm_s2 / 8192.0, ay * gravity_mm_s2 / 8192.0,
            az * gravity_mm_s2 / 8192.0, gx / 65.5, gy / 65.5, gz / 65.5)

    def close(self) -> None:
        if self.device is not None:
            self.device.close()
            self.device = None

    def _read_register(self, device: I2cDevice, address: int, length: int) -> bytes:
        for attempt in range(self.retries + 1):
            try:
                data = device.read_register(address, length)
                if len(data) != length:
                    raise RuntimeError("MPU6500 returned an incomplete register read")
                return data
            except OSError as error:
                if error.errno not in TRANSIENT_I2C_ERRNOS or attempt >= self.retries:
                    raise
                time.sleep(self.retry_delay_s)
        raise RuntimeError("unreachable MPU6500 retry state")

    def _require_device(self) -> I2cDevice:
        if self.device is None:
            raise RuntimeError("MPU6500 is not open")
        return self.device
