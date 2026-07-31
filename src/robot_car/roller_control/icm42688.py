"""Minimal ICM42688 reader supporting Linux SPI and I2C transports."""

from __future__ import annotations

from dataclasses import dataclass
import fcntl
import os
import struct
import time
from typing import Any, Callable, Mapping, Protocol


# ICM-42688-P register bank 0.  The register values and SPI addressing below
# follow DS-000347, rather than the incompatible HXY device manual.
WHO_AM_I = 0x75
WHO_AM_I_ICM42688 = 0x47
PWR_MGMT0 = 0x4E
GYRO_CONFIG0 = 0x4F
ACCEL_CONFIG0 = 0x50
ACCEL_DATA_X1 = 0x1F
I2C_SLAVE = 0x0703


class SpiDevice(Protocol):
    max_speed_hz: int
    mode: int

    def open(self, bus: int, chip_select: int) -> None: ...

    def close(self) -> None: ...

    def xfer2(self, values: list[int]) -> list[int]: ...


class I2cDevice(Protocol):
    def open(self, bus: int, address: int) -> None: ...

    def close(self) -> None: ...

    def read_register(self, address: int, length: int = 1) -> bytes: ...

    def write_register(self, address: int, value: int) -> None: ...


class LinuxI2cDevice:
    """Small i2c-dev adapter so the project has no smbus package dependency."""

    def __init__(self) -> None:
        self.fd: int | None = None

    def open(self, bus: int, address: int) -> None:
        if self.fd is not None:
            return
        self.fd = os.open(f"/dev/i2c-{bus}", os.O_RDWR)
        try:
            fcntl.ioctl(self.fd, I2C_SLAVE, address)
        except Exception:
            os.close(self.fd)
            self.fd = None
            raise

    def read_register(self, address: int, length: int = 1) -> bytes:
        if self.fd is None:
            raise RuntimeError("I2C device is not open")
        os.write(self.fd, bytes((address,)))
        data = os.read(self.fd, length)
        if len(data) != length:
            raise RuntimeError("ICM42688 returned an incomplete I2C register read")
        return data

    def write_register(self, address: int, value: int) -> None:
        if self.fd is None:
            raise RuntimeError("I2C device is not open")
        os.write(self.fd, bytes((address, value)))

    def close(self) -> None:
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None


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
    """Read the ICM42688 WHO_AM_I and raw accel/gyro samples."""

    def __init__(self, bus: int, chip_select: int | None = None, *, transport: str = "spi",
                 i2c_address: int = 0x68, speed_hz: int = 1_000_000,
                 mode: int = 3, spi_factory: Callable[[], SpiDevice] | None = None,
                 i2c_factory: Callable[[], I2cDevice] | None = None,
                 monotonic: Callable[[], float] = time.monotonic) -> None:
        if transport not in ("spi", "i2c"):
            raise ValueError("ICM42688 transport must be spi or i2c")
        if transport == "spi" and chip_select is None:
            raise ValueError("SPI ICM42688 requires a chip select")
        if transport == "i2c" and not 0x03 <= i2c_address <= 0x77:
            raise ValueError("I2C address is invalid")
        self.bus = bus
        self.chip_select = chip_select
        self.transport = transport
        self.i2c_address = i2c_address
        self.speed_hz = speed_hz
        self.mode = mode
        self.spi_factory = spi_factory
        self.i2c_factory = i2c_factory
        self.monotonic = monotonic
        self.spi: SpiDevice | None = None
        self.i2c: I2cDevice | None = None

    def open(self) -> None:
        if self.spi is not None or self.i2c is not None:
            return
        if self.transport == "spi":
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
            identity = self._read_register(WHO_AM_I, spi)
            if identity != WHO_AM_I_ICM42688:
                spi.close()
                raise RuntimeError(
                    "ICM42688-P not found on spi%d.%d: WHO_AM_I=0x%02X (expected 0x47)"
                    % (self.bus, self.chip_select, identity))
            self.spi = spi
            return
        factory = self.i2c_factory or LinuxI2cDevice
        i2c = factory()
        try:
            i2c.open(self.bus, self.i2c_address)
            identity = self._read_register(WHO_AM_I, i2c)
            if identity != WHO_AM_I_ICM42688:
                raise RuntimeError(
                    "ICM42688-P not found on i2c-%d address 0x%02X: WHO_AM_I=0x%02X (expected 0x47)"
                    % (self.bus, self.i2c_address, identity))
        except Exception:
            i2c.close()
            raise
        self.i2c = i2c

    def configure(self) -> None:
        """Enable ±4 g accel and ±500 dps gyro at 100 Hz."""
        device = self._require_device()
        self._write_register(PWR_MGMT0, 0x0F, device)
        time.sleep(0.01)
        self._write_register(ACCEL_CONFIG0, 0x48, device)
        self._write_register(GYRO_CONFIG0, 0x48, device)

    def sample(self) -> ImuSample:
        device = self._require_device()
        if self.transport == "spi":
            values = device.xfer2([self._read_command(ACCEL_DATA_X1)] + [0] * 12)
            if len(values) != 13:
                raise RuntimeError("ICM42688 returned an incomplete sample")
            data = bytes(values[1:])
        else:
            data = device.read_register(ACCEL_DATA_X1, 12)
        decoded = struct.unpack(">hhhhhh", data)
        return ImuSample(self.monotonic(), 0, *decoded[0:3], *decoded[3:6])

    def _read_register(self, address: int, device: Any) -> int:
        if self.transport == "spi":
            values = device.xfer2([self._read_command(address), 0])
            if len(values) != 2:
                raise RuntimeError("ICM42688 returned an incomplete register read")
            return values[1]
        values = device.read_register(address, 1)
        if len(values) != 1:
            raise RuntimeError("ICM42688 returned an incomplete I2C register read")
        return values[0]

    def _write_register(self, address: int, value: int, device: Any) -> None:
        if not 0 <= value <= 0xFF:
            raise ValueError("ICM42688 register value must fit uint8")
        if not 0 <= address <= 0x7F:
            raise ValueError("ICM42688 register address must fit uint7")
        if self.transport == "spi":
            device.xfer2([address, value])
        else:
            device.write_register(address, value)

    @staticmethod
    def _read_command(address: int) -> int:
        if not 0 <= address <= 0x7F:
            raise ValueError("ICM42688 register address must fit uint7")
        return address | 0x80

    def close(self) -> None:
        if self.spi is not None:
            self.spi.close()
            self.spi = None
        if self.i2c is not None:
            self.i2c.close()
            self.i2c = None

    def _require_device(self) -> Any:
        device = self.spi if self.transport == "spi" else self.i2c
        if device is None:
            raise RuntimeError("ICM42688 is not open")
        return device


def sensor_from_config(imu: Mapping[str, Any]) -> Icm42688:
    """Construct an IMU from the transport-specific YAML section."""
    transport = str(imu.get("transport", "spi")).lower()
    if transport == "i2c":
        address = imu.get("i2c_address", 0x68)
        address = int(address, 0) if isinstance(address, str) else int(address)
        return Icm42688(int(imu.get("i2c_bus", 0)), transport="i2c", i2c_address=address,
                        speed_hz=int(imu.get("speed_hz", 400_000)),
                        monotonic=time.monotonic)
    return Icm42688(int(imu["spi_bus"]), int(imu["chip_select"]), transport="spi",
                    speed_hz=int(imu.get("speed_hz", 1_000_000)),
                    mode=int(imu.get("mode", 3)), monotonic=time.monotonic)
