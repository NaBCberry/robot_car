"""Single-axis tube pitch estimation from ICM42688 samples."""

from __future__ import annotations

from dataclasses import dataclass
import math

from .icm42688 import ImuSample


@dataclass(frozen=True)
class PitchEstimate:
    timestamp_s: float
    pitch_deg: float
    pitch_rate_deg_s: float


class PitchEstimator:
    """Single-axis Mahony PI attitude estimator with configurable sensor axes."""

    def __init__(self, *, slope_accel_axis: str, gravity_accel_axis: str, gyro_axis: str,
                 slope_accel_sign: int = 1, gravity_accel_sign: int = 1,
                 gyro_sign: int = 1, gyro_lsb_per_dps: float = 65.5,
                 accel_lsb_per_g: float = 8192.0, gyro_weight: float = 0.98,
                 pitch_zero_offset_deg: float = 0.0, gyro_bias_raw: float = 0.0,
                 gyro_correction_time_constant_s: float | None = None,
                 mahony_kp: float | None = None, mahony_ki: float = 0.0,
                 mahony_integral_limit_dps: float = 5.0) -> None:
        self.slope_accel_axis = self._axis(slope_accel_axis)
        self.gravity_accel_axis = self._axis(gravity_accel_axis)
        self.gyro_axis = self._axis(gyro_axis)
        if slope_accel_axis == gravity_accel_axis:
            raise ValueError("slope and gravity accelerometer axes must differ")
        if slope_accel_sign not in {-1, 1} or gravity_accel_sign not in {-1, 1}:
            raise ValueError("accelerometer signs must be -1 or 1")
        if gyro_sign not in {-1, 1}:
            raise ValueError("gyro_sign must be -1 or 1")
        if gyro_lsb_per_dps <= 0 or accel_lsb_per_g <= 0 or not 0 <= gyro_weight <= 1:
            raise ValueError("pitch estimator parameters are invalid")
        if not math.isfinite(pitch_zero_offset_deg) or not math.isfinite(gyro_bias_raw):
            raise ValueError("IMU calibration offsets must be finite")
        if gyro_correction_time_constant_s is not None and (
                not math.isfinite(gyro_correction_time_constant_s)
                or gyro_correction_time_constant_s <= 0):
            raise ValueError("gyro_correction_time_constant_s must be positive")
        if mahony_kp is not None and (not math.isfinite(mahony_kp) or mahony_kp < 0):
            raise ValueError("mahony_kp must not be negative")
        if (not math.isfinite(mahony_ki) or mahony_ki < 0
                or not math.isfinite(mahony_integral_limit_dps)
                or mahony_integral_limit_dps < 0):
            raise ValueError("Mahony parameters must not be negative")
        self.slope_accel_sign = slope_accel_sign
        self.gravity_accel_sign = gravity_accel_sign
        self.gyro_sign = gyro_sign
        self.gyro_lsb_per_dps = gyro_lsb_per_dps
        self.accel_lsb_per_g = accel_lsb_per_g
        self.gyro_weight = gyro_weight
        self.pitch_zero_offset_deg = pitch_zero_offset_deg
        self.gyro_bias_raw = gyro_bias_raw
        self.gyro_correction_time_constant_s = gyro_correction_time_constant_s
        self.mahony_kp = mahony_kp
        self.mahony_ki = mahony_ki
        self.mahony_integral_limit_rad_s = math.radians(mahony_integral_limit_dps)
        self._estimate: PitchEstimate | None = None
        self._mahony_integral_rad_s = 0.0

    def reset(self) -> None:
        self._estimate = None
        self._mahony_integral_rad_s = 0.0

    def update(self, sample: ImuSample) -> PitchEstimate:
        acceleration = self._acceleration_pitch(sample)
        gyro_rate = self.gyro_sign * (self._gyro(sample, self.gyro_axis) - self.gyro_bias_raw)
        gyro_rate /= self.gyro_lsb_per_dps
        previous = self._estimate
        if previous is None or sample.timestamp_s <= previous.timestamp_s:
            self._estimate = PitchEstimate(sample.timestamp_s, acceleration, gyro_rate)
            return self._estimate
        elapsed_s = sample.timestamp_s - previous.timestamp_s
        if self.mahony_kp is not None:
            estimate_rad = math.radians(previous.pitch_deg)
            acceleration_rad = math.radians(acceleration)
            gravity_error = math.sin(acceleration_rad - estimate_rad)
            if self.mahony_ki > 0:
                self._mahony_integral_rad_s += self.mahony_ki * gravity_error * elapsed_s
                self._mahony_integral_rad_s = max(-self.mahony_integral_limit_rad_s,
                                                  min(self.mahony_integral_limit_rad_s,
                                                      self._mahony_integral_rad_s))
            else:
                self._mahony_integral_rad_s = 0.0
            corrected_rate_rad_s = (math.radians(gyro_rate) + self.mahony_kp * gravity_error
                                    + self._mahony_integral_rad_s)
            pitch = math.degrees(estimate_rad + corrected_rate_rad_s * elapsed_s)
            self._estimate = PitchEstimate(sample.timestamp_s, pitch, gyro_rate)
            return self._estimate
        gyro_pitch = previous.pitch_deg + gyro_rate * elapsed_s
        if self.gyro_correction_time_constant_s is None:
            gyro_weight = self.gyro_weight
        else:
            gyro_weight = math.exp(-elapsed_s / self.gyro_correction_time_constant_s)
        pitch = gyro_weight * gyro_pitch + (1.0 - gyro_weight) * acceleration
        self._estimate = PitchEstimate(sample.timestamp_s, pitch, gyro_rate)
        return self._estimate

    def _acceleration_pitch(self, sample: ImuSample) -> float:
        return self.raw_acceleration_pitch_deg(sample) - self.pitch_zero_offset_deg

    def raw_acceleration_pitch_deg(self, sample: ImuSample) -> float:
        slope = self.slope_accel_sign * self._accel(sample, self.slope_accel_axis)
        gravity = self.gravity_accel_sign * self._accel(sample, self.gravity_accel_axis)
        return math.degrees(math.atan2(slope, gravity))

    @staticmethod
    def _axis(value: str) -> str:
        if value not in {"x", "y", "z"}:
            raise ValueError("IMU axis must be x, y, or z")
        return value

    @staticmethod
    def _accel(sample: ImuSample, axis: str) -> int:
        return getattr(sample, f"accel_{axis}_raw")

    @staticmethod
    def _gyro(sample: ImuSample, axis: str) -> int:
        return getattr(sample, f"gyro_{axis}_raw")
