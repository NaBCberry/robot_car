"""Minimal SPI reader for an ICM42688 inertial sensor."""

from __future__ import annotations

from dataclasses import dataclass
import struct
import time
from typing import Callable, Protocol


WHO_AM_I = 0x75
WHO_AM_I_ICM42688 = 0x47
PWR_MGMT0 = 0x4E
GYRO_CONFIG0 = 0x4F
ACCEL_CONFIG0 = 0x50
TEMP_DATA1 = 0x1D


class SpiDevice(Protocol):
    max_speed_hz: int
    mode: int

    def open(self, bus: int, chip_select: int) -> None: ...

    def close(self) -> None: ...

    def xfer2(self, values: list[int]) -> list[int]: ...


@dataclass(frozen=True)
class ImuSample:
    timestamp_s: float
    temperature_raw: int
    accel_x_raw: int
    accel_y_raw: int
    accel_z_raw: int
    gyro_x_raw: int
    gyro_y_raw: int
    gyro_z_raw: int


class Icm42688:
    """Read the ICM42688 WHO_AM_I and raw accel/gyro samples over SPI."""

    def __init__(self, bus: int, chip_select: int, *, speed_hz: int = 1_000_000,
                 mode: int = 0, spi_factory: Callable[[], SpiDevice] | None = None,
                 monotonic: Callable[[], float] = time.monotonic) -> None:
        self.bus = bus
        self.chip_select = chip_select
        self.speed_hz = speed_hz
        self.mode = mode
        self.spi_factory = spi_factory
        self.monotonic = monotonic
        self.spi: SpiDevice | None = None

    def open(self) -> None:
        if self.spi is not None:
            return
        factory = self.spi_factory
        if factory is None:
            try:
                import spidev
            except ImportError as error:
                raise RuntimeError("ICM42688 requires the spidev Python package") from error
            factory = spidev.SpiDev
        spi = factory()
        spi.open(self.bus, self.chip_select)
        spi.max_speed_hz = self.speed_hz
        spi.mode = self.mode
        identity = self.read_register(WHO_AM_I, spi)
        if identity != WHO_AM_I_ICM42688:
            spi.close()
            raise RuntimeError(
                "ICM42688 not found on spi%d.%d: WHO_AM_I=0x%02X (expected 0x47)"
                % (self.bus, self.chip_select, identity))
        self.spi = spi

    def configure(self) -> None:
        """Enable accel and gyro in low-noise mode at 1 kHz."""
        spi = self._require_spi()
        self.write_register(GYRO_CONFIG0, 0x46, spi)
        self.write_register(ACCEL_CONFIG0, 0x46, spi)
        self.write_register(PWR_MGMT0, 0x0F, spi)

    def sample(self) -> ImuSample:
        spi = self._require_spi()
        values = spi.xfer2([TEMP_DATA1 | 0x80] + [0] * 14)
        if len(values) != 15:
            raise RuntimeError("ICM42688 returned an incomplete sample")
        decoded = struct.unpack(">hhhhhhh", bytes(values[1:]))
        return ImuSample(self.monotonic(), *decoded)

    @staticmethod
    def read_register(address: int, spi: SpiDevice) -> int:
        values = spi.xfer2([address | 0x80, 0])
        if len(values) != 2:
            raise RuntimeError("ICM42688 returned an incomplete register read")
        return values[1]

    @staticmethod
    def write_register(address: int, value: int, spi: SpiDevice) -> None:
        if not 0 <= value <= 0xFF:
            raise ValueError("ICM42688 register value must fit uint8")
        spi.xfer2([address & 0x7F, value])

    def close(self) -> None:
        if self.spi is not None:
            self.spi.close()
            self.spi = None

    def _require_spi(self) -> SpiDevice:
        if self.spi is None:
            raise RuntimeError("ICM42688 is not open")
        return self.spi
