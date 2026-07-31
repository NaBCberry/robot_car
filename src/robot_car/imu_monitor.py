"""Read and display ICM-42688-P motion data without opening CAN."""

from __future__ import annotations

import argparse
from statistics import median
import sys
import time
from pathlib import Path

from robot_car.roller_control.attitude import PitchEstimator
from robot_car.roller_control.config import load_roller_control
from robot_car.roller_control.icm42688 import Icm42688, sensor_from_config


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="ICM-42688-P read-only monitor")
    parser.add_argument("--control-config", default="config/roller_control.yaml")
    parser.add_argument("--hz", type=float, default=20.0)
    parser.add_argument("--samples", type=int, default=0,
                        help="number of samples; 0 means run until Ctrl-C")
    parser.add_argument("--calibrate-samples", type=int, default=0,
                        help="sample a stationary mechanical zero and print YAML offsets")
    parser.add_argument("--show-filtered", action="store_true",
                        help="keep the current filtered attitude on the terminal bottom line")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.hz <= 0 or args.samples < 0 or args.calibrate_samples < 0:
        raise SystemExit("--hz must be positive and sample counts must not be negative")
    config = load_roller_control(Path(args.control_config))
    imu = config["imu"]
    sensor = sensor_from_config(imu)
    pitch = PitchEstimator(
        slope_accel_axis=str(imu["slope_accel_axis"]),
        gravity_accel_axis=str(imu["gravity_accel_axis"]), gyro_axis=str(imu["gyro_axis"]),
        slope_accel_sign=int(imu.get("slope_accel_sign", 1)),
        gravity_accel_sign=int(imu.get("gravity_accel_sign", 1)),
        gyro_sign=int(imu.get("gyro_sign", 1)), gyro_weight=float(imu.get("gyro_weight", 0.98)),
        pitch_zero_offset_deg=float(imu.get("pitch_zero_offset_deg", 0.0)),
        gyro_bias_raw=float(imu.get("gyro_bias_raw", 0.0)),
        gyro_correction_time_constant_s=imu.get("gyro_correction_time_constant_s"),
        mahony_kp=imu.get("mahony_kp"), mahony_ki=float(imu.get("mahony_ki", 0.0)),
        mahony_integral_limit_dps=float(imu.get("mahony_integral_limit_dps", 5.0)))
    interval_s = 1.0 / args.hz
    sensor.open()
    try:
        sensor.configure()
        if args.calibrate_samples:
            raw_samples = []
            progress_interval = max(1, args.calibrate_samples // 10)
            print("开始静态标定：采集 %d 点，请保持机构静止。" % args.calibrate_samples,
                  flush=True)
            for index in range(args.calibrate_samples):
                started_s = time.monotonic()
                sample = sensor.sample()
                raw_values = (
                    sample.accel_x_raw, sample.accel_y_raw, sample.accel_z_raw,
                    sample.gyro_x_raw, sample.gyro_y_raw, sample.gyro_z_raw,
                )
                # A transient bus transfer can return an all-zero frame. It
                # must not pull the static zero and gyro bias toward zero.
                if any(raw_values):
                    raw_samples.append((
                        pitch.raw_acceleration_pitch_deg(sample),
                        getattr(sample, f"gyro_{pitch.gyro_axis}_raw"),
                    ))
                time.sleep(max(0.0, interval_s - (time.monotonic() - started_s)))
                if (index + 1) % progress_interval == 0 or index + 1 == args.calibrate_samples:
                    print("标定采样：%d/%d" % (index + 1, args.calibrate_samples), flush=True)
            if not raw_samples:
                raise RuntimeError("no valid IMU samples received during calibration")
            pitch_center = median(item[0] for item in raw_samples)
            gyro_center = median(item[1] for item in raw_samples)
            pitch_mad = median(abs(item[0] - pitch_center) for item in raw_samples)
            gyro_mad = median(abs(item[1] - gyro_center) for item in raw_samples)
            pitch_limit = max(0.5, 6.0 * pitch_mad)
            gyro_limit = max(100.0, 6.0 * gyro_mad)
            filtered = [(raw_pitch, gyro) for raw_pitch, gyro in raw_samples
                        if abs(raw_pitch - pitch_center) <= pitch_limit
                        and abs(gyro - gyro_center) <= gyro_limit]
            if len(filtered) < max(10, len(raw_samples) // 2):
                raise RuntimeError("too many outliers in IMU calibration samples")
            print("imu:")
            print("  pitch_zero_offset_deg: %.3f" % (sum(item[0] for item in filtered) / len(filtered)))
            print("  gyro_bias_raw: %.3f" % (sum(item[1] for item in filtered) / len(filtered)))
            return 0
        print("timestamp_s accel_raw[x,y,z] gyro_raw[x,y,z] pitch_deg rate_deg_s")
        count = 0
        while args.samples == 0 or count < args.samples:
            started_s = time.monotonic()
            sample = sensor.sample()
            estimate = pitch.update(sample)
            if args.show_filtered:
                # Replace the prior status line so raw samples remain above it.
                sys.stdout.write("\r\033[2K")
            print("%.3f %d,%d,%d %d,%d,%d %.2f %.2f" % (
                sample.timestamp_s, sample.accel_x_raw, sample.accel_y_raw, sample.accel_z_raw,
                sample.gyro_x_raw, sample.gyro_y_raw, sample.gyro_z_raw,
                estimate.pitch_deg, estimate.pitch_rate_deg_s), flush=True)
            if args.show_filtered:
                print("滤波结果 俯仰角: %.2f°  角速度: %.2f°/s" % (
                    estimate.pitch_deg, estimate.pitch_rate_deg_s), end="", flush=True)
            count += 1
            time.sleep(max(0.0, interval_s - (time.monotonic() - started_s)))
    except KeyboardInterrupt:
        pass
    finally:
        if args.show_filtered and not args.calibrate_samples:
            print()
        sensor.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
