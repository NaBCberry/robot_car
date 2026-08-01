"""Fuse M0 wheel telemetry and chassis IMU data into tube-axis acceleration."""

from __future__ import annotations

from dataclasses import dataclass
import logging
import math
from typing import Any, Callable, Mapping

from .mpu6500 import Mpu6500, Mpu6500Sample


LOG = logging.getLogger(__name__)


@dataclass(frozen=True)
class VehicleMotionEstimate:
    longitudinal_mm_s2: float
    lateral_mm_s2: float
    tube_axis_mm_s2: float


class VehicleMotionFeedforward:
    """Latest-value, bounded vehicle acceleration feedforward.

    Encoder odometry supplies the primary longitudinal acceleration and turn
    rate.  The optional MPU6500 only corrects those estimates; it never makes
    an uncalibrated raw acceleration the sole feedforward source.
    """

    def __init__(self, config: Mapping[str, Any], *,
                 sensor_factory: Callable[..., Mpu6500] = Mpu6500) -> None:
        self.config = dict(config)
        self.enabled = bool(self.config.get("enabled", False))
        self.timeout_ms = int(self.config.get("telemetry_timeout_ms", 150))
        self.max_acceleration_mm_s2 = float(self.config.get("max_acceleration_mm_s2", 3_000.0))
        self.encoder_accel_alpha = float(self.config.get("encoder_accel_alpha", 0.35))
        self.imu_accel_correction_gain = float(
            self.config.get("imu_accel_correction_gain", 0.20))
        self.imu_yaw_correction_gain = float(
            self.config.get("imu_yaw_correction_gain", 0.35))
        self.pipe_acceleration_sign = float(self.config.get("pipe_acceleration_sign", 1.0))
        self._pipe_axis = self._normalised_axis(self.config.get("pipe_axis_in_chassis", [1.0, 0.0]))
        self.wheel_track_mm = float(self.config.get("wheel_track_mm", 1.0))
        if self.timeout_ms <= 0 or self.max_acceleration_mm_s2 <= 0:
            raise ValueError("vehicle motion timeout and acceleration limit must be positive")
        if self.wheel_track_mm <= 0:
            raise ValueError("vehicle motion wheel_track_mm must be positive")
        for name, value in (("encoder_accel_alpha", self.encoder_accel_alpha),
                            ("imu_accel_correction_gain", self.imu_accel_correction_gain),
                            ("imu_yaw_correction_gain", self.imu_yaw_correction_gain)):
            if not 0.0 <= value <= 1.0:
                raise ValueError(f"{name} must be in 0..1")
        if self.pipe_acceleration_sign not in {-1.0, 1.0}:
            raise ValueError("pipe_acceleration_sign must be -1 or 1")

        mpu_config = self.config.get("mpu6500", {})
        if not isinstance(mpu_config, Mapping):
            raise ValueError("vehicle_motion.mpu6500 must be a mapping")
        self.mpu_config = dict(mpu_config)
        self.mpu_enabled = self.enabled and bool(self.mpu_config.get("enabled", False))
        self.sensor = (sensor_factory(int(self.mpu_config.get("i2c_bus", 0)),
                                      int(self.mpu_config.get("i2c_address", 0x68)),
                                      retries=int(self.mpu_config.get("i2c_retries", 2)),
                                      retry_delay_ms=int(self.mpu_config.get("i2c_retry_delay_ms", 2)))
                       if self.mpu_enabled else None)
        self._last_telemetry_ms: int | None = None
        self._last_speed_mm_s: float | None = None
        self._encoder_acceleration_mm_s2 = 0.0
        self._last_mpu_sample_s: float | None = None
        self._gravity_mm_s2: list[float] | None = None

    def open(self) -> None:
        if self.sensor is not None:
            try:
                self.sensor.open()
                self.sensor.configure()
            except (OSError, RuntimeError) as error:
                # The IMU is only a correction source.  Retain encoder-based
                # feedforward rather than making vehicle control unavailable.
                LOG.warning("MPU6500 correction disabled: %s", error)
                self.sensor.close()
                self.sensor = None

    def close(self) -> None:
        if self.sensor is not None:
            self.sensor.close()

    def reset(self) -> None:
        self._last_telemetry_ms = None
        self._last_speed_mm_s = None
        self._encoder_acceleration_mm_s2 = 0.0

    def update(self, telemetry: Mapping[str, Any], updated_ms: int | None,
               now_ms: int) -> VehicleMotionEstimate | None:
        if not self.enabled or updated_ms is None or now_ms - updated_ms > self.timeout_ms:
            self.reset()
            return None
        speed = self._speed(telemetry)
        if speed is None:
            return None
        if updated_ms != self._last_telemetry_ms:
            self._update_encoder_acceleration(speed, updated_ms)
            self._last_telemetry_ms = updated_ms
        yaw_rate_rad_s = self._wheel_yaw_rate(telemetry)
        longitudinal = self._encoder_acceleration_mm_s2
        lateral = speed * yaw_rate_rad_s
        sample = self._sample_mpu()
        if sample is not None:
            imu_longitudinal, imu_lateral, imu_yaw_rate = self._mpu_motion(sample)
            longitudinal = self._blend(longitudinal, imu_longitudinal,
                                       self.imu_accel_correction_gain)
            lateral = self._blend(lateral, imu_lateral, self.imu_accel_correction_gain)
            yaw_rate_rad_s = self._blend(yaw_rate_rad_s, math.radians(imu_yaw_rate),
                                         self.imu_yaw_correction_gain)
            lateral = self._blend(lateral, speed * yaw_rate_rad_s,
                                  self.imu_yaw_correction_gain)
        longitudinal = self._clamp(longitudinal)
        lateral = self._clamp(lateral)
        tube_axis = self._clamp(self.pipe_acceleration_sign * (
            longitudinal * self._pipe_axis[0] + lateral * self._pipe_axis[1]))
        return VehicleMotionEstimate(longitudinal, lateral, tube_axis)

    def _update_encoder_acceleration(self, speed: float, timestamp_ms: int) -> None:
        if self._last_speed_mm_s is not None and self._last_telemetry_ms is not None:
            elapsed_ms = timestamp_ms - self._last_telemetry_ms
            if 5 <= elapsed_ms <= self.timeout_ms:
                instantaneous = (speed - self._last_speed_mm_s) * 1000.0 / elapsed_ms
                instantaneous = self._clamp(instantaneous)
                self._encoder_acceleration_mm_s2 = self._blend(
                    self._encoder_acceleration_mm_s2, instantaneous, self.encoder_accel_alpha)
            else:
                self._encoder_acceleration_mm_s2 = 0.0
        self._last_speed_mm_s = speed

    def _speed(self, telemetry: Mapping[str, Any]) -> float | None:
        value = self._finite(telemetry.get("speed_mm_s", telemetry.get("speed_fused_mm_s")))
        if value is not None:
            return value
        left = self._finite(telemetry.get("speed_left_mm_s"))
        right = self._finite(telemetry.get("speed_right_mm_s"))
        return None if left is None or right is None else (left + right) / 2.0

    def _wheel_yaw_rate(self, telemetry: Mapping[str, Any]) -> float:
        left = self._finite(telemetry.get("speed_left_mm_s"))
        right = self._finite(telemetry.get("speed_right_mm_s"))
        return 0.0 if left is None or right is None else (right - left) / self.wheel_track_mm

    def _sample_mpu(self) -> Mpu6500Sample | None:
        if self.sensor is None:
            return None
        try:
            return self.sensor.sample()
        except (OSError, RuntimeError) as error:
            LOG.warning("MPU6500 correction disabled after read error: %s", error)
            self.sensor.close()
            self.sensor = None
            return None

    def _mpu_motion(self, sample: Mpu6500Sample) -> tuple[float, float, float]:
        forward_axis = str(self.mpu_config.get("forward_axis", "x"))
        lateral_axis = str(self.mpu_config.get("lateral_axis", "y"))
        vertical_axis = str(self.mpu_config.get("vertical_axis", "z"))
        axes = (forward_axis, lateral_axis, vertical_axis)
        if set(axes) != {"x", "y", "z"}:
            raise ValueError("MPU6500 body axes must be x, y, z exactly once")
        acceleration = [sample.acceleration(axis) * self._axis_sign(axis) for axis in axes]
        if self._gravity_mm_s2 is None:
            self._gravity_mm_s2 = list(acceleration)
        else:
            elapsed_s = max(0.001, sample.timestamp_s - (self._last_mpu_sample_s or sample.timestamp_s))
            tau_s = float(self.mpu_config.get("gravity_time_constant_s", 1.0))
            alpha = elapsed_s / (tau_s + elapsed_s)
            self._gravity_mm_s2 = [previous + alpha * (value - previous)
                                    for previous, value in zip(self._gravity_mm_s2, acceleration)]
        self._last_mpu_sample_s = sample.timestamp_s
        gravity_norm = math.sqrt(sum(value * value for value in self._gravity_mm_s2))
        if gravity_norm < 1.0:
            return 0.0, 0.0, self._gyro_z(sample)
        gravity = [value * 9_806.65 / gravity_norm for value in self._gravity_mm_s2]
        linear = [value - correction for value, correction in zip(acceleration, gravity)]
        return linear[0], linear[1], self._gyro_z(sample)

    def _gyro_z(self, sample: Mpu6500Sample) -> float:
        axis = str(self.mpu_config.get("yaw_gyro_axis", "z"))
        if axis not in {"x", "y", "z"}:
            raise ValueError("MPU6500 yaw_gyro_axis must be x, y, or z")
        bias = float(self.mpu_config.get("yaw_gyro_bias_deg_s", 0.0))
        sign = float(self.mpu_config.get("yaw_gyro_sign", 1.0))
        if sign not in {-1.0, 1.0}:
            raise ValueError("MPU6500 yaw_gyro_sign must be -1 or 1")
        return sign * (sample.gyro(axis) - bias)

    def _axis_sign(self, axis: str) -> float:
        sign = float(self.mpu_config.get(f"{axis}_sign", 1.0))
        if sign not in {-1.0, 1.0}:
            raise ValueError(f"MPU6500 {axis}_sign must be -1 or 1")
        return sign

    def _clamp(self, value: float) -> float:
        return max(-self.max_acceleration_mm_s2, min(self.max_acceleration_mm_s2, value))

    @staticmethod
    def _blend(primary: float, correction: float, gain: float) -> float:
        return primary + gain * (correction - primary)

    @staticmethod
    def _finite(value: Any) -> float | None:
        try:
            result = float(value)
        except (TypeError, ValueError):
            return None
        return result if math.isfinite(result) else None

    @staticmethod
    def _normalised_axis(value: Any) -> tuple[float, float]:
        if not isinstance(value, (list, tuple)) or len(value) != 2:
            raise ValueError("pipe_axis_in_chassis must contain [forward, lateral]")
        try:
            forward, lateral = float(value[0]), float(value[1])
        except (TypeError, ValueError) as error:
            raise ValueError("pipe_axis_in_chassis values must be numeric") from error
        norm = math.hypot(forward, lateral)
        if not math.isfinite(norm) or norm < 1e-6:
            raise ValueError("pipe_axis_in_chassis must be non-zero")
        return forward / norm, lateral / norm
