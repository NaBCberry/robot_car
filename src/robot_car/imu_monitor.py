"""Read and display ICM-42688-P motion data without opening CAN."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

from robot_car.roller_control.attitude import PitchEstimator
from robot_car.roller_control.config import load_roller_control
from robot_car.roller_control.icm42688 import Icm42688


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="ICM-42688-P read-only monitor")
    parser.add_argument("--control-config", default="config/roller_control.yaml")
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--samples", type=int, default=0,
                        help="number of samples; 0 means run until Ctrl-C")
    parser.add_argument("--calibrate-samples", type=int, default=0,
                        help="sample a stationary mechanical zero and print YAML offsets")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.hz <= 0 or args.samples < 0 or args.calibrate_samples < 0:
        raise SystemExit("--hz must be positive and sample counts must not be negative")
    config = load_roller_control(Path(args.control_config))
    imu = config["imu"]
    sensor = Icm42688(int(imu["spi_bus"]), int(imu["chip_select"]),
                      speed_hz=int(imu.get("speed_hz", 1_000_000)), mode=int(imu.get("mode", 3)))
    pitch = PitchEstimator(
        slope_accel_axis=str(imu["slope_accel_axis"]),
        gravity_accel_axis=str(imu["gravity_accel_axis"]), gyro_axis=str(imu["gyro_axis"]),
        slope_accel_sign=int(imu.get("slope_accel_sign", 1)),
        gravity_accel_sign=int(imu.get("gravity_accel_sign", 1)),
        gyro_sign=int(imu.get("gyro_sign", 1)), gyro_weight=float(imu.get("gyro_weight", 0.98)),
        pitch_zero_offset_deg=float(imu.get("pitch_zero_offset_deg", 0.0)),
        gyro_bias_raw=float(imu.get("gyro_bias_raw", 0.0)))
    interval_s = 1.0 / args.hz
    sensor.open()
    try:
        sensor.configure()
        if args.calibrate_samples:
            raw_pitches = []
            gyro_values = []
            for _ in range(args.calibrate_samples):
                started_s = time.monotonic()
                sample = sensor.sample()
                raw_pitches.append(pitch.raw_acceleration_pitch_deg(sample))
                gyro_values.append(getattr(sample, f"gyro_{pitch.gyro_axis}_raw"))
                time.sleep(max(0.0, interval_s - (time.monotonic() - started_s)))
            print("imu:")
            print("  pitch_zero_offset_deg: %.3f" % (sum(raw_pitches) / len(raw_pitches)))
            print("  gyro_bias_raw: %.3f" % (sum(gyro_values) / len(gyro_values)))
            return 0
        print("timestamp_s accel_raw[x,y,z] gyro_raw[x,y,z] pitch_deg rate_deg_s")
        count = 0
        while args.samples == 0 or count < args.samples:
            started_s = time.monotonic()
            sample = sensor.sample()
            estimate = pitch.update(sample)
            print("%.3f %d,%d,%d %d,%d,%d %.2f %.2f" % (
                sample.timestamp_s, sample.accel_x_raw, sample.accel_y_raw, sample.accel_z_raw,
                sample.gyro_x_raw, sample.gyro_y_raw, sample.gyro_z_raw,
                estimate.pitch_deg, estimate.pitch_rate_deg_s), flush=True)
            count += 1
            time.sleep(max(0.0, interval_s - (time.monotonic() - started_s)))
    except KeyboardInterrupt:
        pass
    finally:
        sensor.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
