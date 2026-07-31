"""Interactive Y42 soft-limit and crank calibration for the roller mechanism."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import time
from typing import Iterable

from robot_car.roller_control.attitude import PitchEstimator
from robot_car.roller_control.config import load_roller_control
from robot_car.roller_control.icm42688 import Icm42688
from robot_car.roller_control.y42_actuator import Y42Actuator


def soft_limits_from_points(first_deg: float, second_deg: float, margin_deg: float) -> tuple[float, float]:
    if not all(math.isfinite(value) for value in (first_deg, second_deg, margin_deg)) or margin_deg < 0:
        raise ValueError("limit measurements and margin must be finite")
    lower, upper = sorted((first_deg, second_deg))
    if upper - lower <= 2 * margin_deg:
        raise ValueError("measured travel is too small for the requested soft-limit margin")
    return lower + margin_deg, upper - margin_deg


def invert_crank_samples(samples: Iterable[tuple[float, float]]) -> list[list[float]]:
    """Convert (motor degree, tube degree) samples into controller inverse-map points."""
    points = sorted((float(tube), float(motor)) for motor, tube in samples)
    if len(points) < 3 or any(not math.isfinite(value) for point in points for value in point):
        raise ValueError("at least three finite crank samples are required")
    tube_deltas = [right[0] - left[0] for left, right in zip(points, points[1:])]
    motor_deltas = [right[1] - left[1] for left, right in zip(points, points[1:])]
    if any(delta <= 0.1 for delta in tube_deltas):
        raise ValueError("tube angle samples are not uniquely invertible")
    if not (all(delta > 0 for delta in motor_deltas) or all(delta < 0 for delta in motor_deltas)):
        raise ValueError("crank samples are not monotonic over the selected travel")
    return [[round(tube, 3), round(motor, 3)] for tube, motor in points]


def update_config(path: Path, *, limits: tuple[float, float] | None = None,
                  crank_map: list[list[float]] | None = None,
                  soft_limit_firmware: str | None = None) -> None:
    text = path.read_text(encoding="utf-8")
    replacements: dict[str, str] = {}
    if limits is not None:
        replacements["soft_limit_min_deg"] = "%.3f" % limits[0]
        replacements["soft_limit_max_deg"] = "%.3f" % limits[1]
    if crank_map is not None:
        replacements["motor_deg_by_tube_angle"] = repr(crank_map)
    if soft_limit_firmware is not None:
        replacements["soft_limit_firmware"] = soft_limit_firmware
    lines = []
    remaining = set(replacements)
    for line in text.splitlines(keepends=True):
        stripped = line.lstrip()
        key = stripped.split(":", 1)[0] if ":" in stripped else ""
        if key in replacements:
            indent = line[:len(line) - len(stripped)]
            lines.append(f"{indent}{key}: {replacements[key]}\n")
            remaining.remove(key)
        else:
            lines.append(line)
    if remaining:
        raise ValueError("configuration is missing: " + ", ".join(sorted(remaining)))
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text("".join(lines), encoding="utf-8")
    temporary.replace(path)


def prompt_capture(label: str) -> None:
    print(f"\n{label}")
    print("1. 记录当前位置")
    print("2. 取消")
    if input("选择 [1/2]: ").strip() != "1":
        raise RuntimeError("calibration cancelled")


def display_motor_position(actuator: Y42Actuator, config: dict, label: str) -> float:
    raw_position = actuator.read_position_deg()
    encoder = actuator.read_encoder_deg()
    ratio = float(config["motor"].get("position_deg_per_output_deg", 1.0))
    if not math.isfinite(ratio) or ratio <= 0:
        raise ValueError("position_deg_per_output_deg must be positive")
    print("%s：Y42 多圈坐标 %.3f°，单圈编码器 %.3f°，换算输出轴 %.3f°" % (
        label, raw_position, encoder, raw_position / ratio))
    return raw_position


def countdown(seconds: int) -> None:
    for remaining in range(seconds, 0, -1):
        print("%d..." % remaining, flush=True)
        time.sleep(1)


def wait_for_position(actuator: Y42Actuator, target_deg: float, timeout_s: float = 30.0) -> float:
    deadline = time.monotonic() + timeout_s
    last_position = None
    while time.monotonic() < deadline:
        position = actuator.read_position_deg(timeout_s=min(2.0, deadline - time.monotonic()))
        last_position = position
        if abs(position - target_deg) <= 1.0:
            return position
        time.sleep(0.1)
    status = actuator.read_motor_status()
    raise RuntimeError("Y42 did not reach %.3f°; last position %.3f°, status 0x%02X" % (
        target_deg, last_position, status))


def wait_for_absolute_home(actuator: Y42Actuator, timeout_s: float = 30.0) -> float:
    deadline = time.monotonic() + timeout_s
    saw_active = False
    while time.monotonic() < deadline:
        remaining = min(2.0, deadline - time.monotonic())
        status = actuator.read_home_status(timeout_s=remaining)
        if status & 0x08:
            raise RuntimeError("Y42 absolute home failed (status 0x%02X)" % status)
        saw_active = saw_active or bool(status & 0x04)
        position = actuator.read_position_deg(timeout_s=remaining)
        if not status & 0x04 and (saw_active or abs(position) <= 1.0):
            return position
        time.sleep(0.1)
    raise RuntimeError("Y42 absolute home timed out")


def build_pitch(config: dict) -> tuple[Icm42688, PitchEstimator]:
    imu = config["imu"]
    sensor = Icm42688(int(imu["spi_bus"]), int(imu["chip_select"]),
                      speed_hz=int(imu.get("speed_hz", 1_000_000)), mode=int(imu.get("mode", 3)))
    pitch = PitchEstimator(
        slope_accel_axis=str(imu["slope_accel_axis"]),
        gravity_accel_axis=str(imu["gravity_accel_axis"]), gyro_axis=str(imu["gyro_axis"]),
        slope_accel_sign=int(imu.get("slope_accel_sign", 1)),
        gravity_accel_sign=int(imu.get("gravity_accel_sign", 1)),
        gyro_sign=int(imu.get("gyro_sign", 1)),
        pitch_zero_offset_deg=float(imu.get("pitch_zero_offset_deg", 0.0)),
        gyro_bias_raw=float(imu.get("gyro_bias_raw", 0.0)),
        mahony_kp=imu.get("mahony_kp"), mahony_ki=float(imu.get("mahony_ki", 0.0)),
        mahony_integral_limit_dps=float(imu.get("mahony_integral_limit_dps", 5.0)))
    return sensor, pitch


def build_actuator(config: dict) -> Y42Actuator:
    motor = config["motor"]
    return Y42Actuator(interface=str(motor["can_interface"]), address=int(motor["address"]),
                       firmware=str(motor.get("firmware", "x")),
                       packet_gap_ms=float(motor.get("packet_gap_ms", 3)),
                       soft_limit_min_deg=float(motor["soft_limit_min_deg"]),
                       soft_limit_max_deg=float(motor["soft_limit_max_deg"]),
                       pulses_per_revolution=int(motor.get("pulses_per_revolution", 3200)))


def average_tube_pitch(sensor: Icm42688, pitch: PitchEstimator, *, samples: int, hz: float) -> float:
    values = []
    interval_s = 1.0 / hz
    for _ in range(samples):
        started_s = time.monotonic()
        sample = sensor.sample()
        values.append(pitch.raw_acceleration_pitch_deg(sample) - pitch.pitch_zero_offset_deg)
        time.sleep(max(0.0, interval_s - (time.monotonic() - started_s)))
    return sum(values) / len(values)


def calibrate_limits(args: argparse.Namespace, config: dict, path: Path) -> int:
    actuator = build_actuator(config)
    keep_enabled = False
    actuator.open()
    try:
        actuator.disable()
        print("Y42 已发送 disable（松轴）。确认曲轴轴已脱开后再手动旋转电机。")
        prompt_capture("手动转到负方向安全端")
        first = display_motor_position(actuator, config, "记录负端")
        prompt_capture("手动转到正方向安全端")
        second = display_motor_position(actuator, config, "记录正端")
        limits = soft_limits_from_points(first, second, args.margin_deg)
        print("\n建议写入（Y42 多圈控制坐标）：")
        print("  soft_limit_min_deg: %.3f" % limits[0])
        print("  soft_limit_max_deg: %.3f" % limits[1])
        if args.apply:
            update_config(path, limits=limits, soft_limit_firmware=actuator.firmware)
            print("已写入 %s" % path)
        print("1. 保持松轴")
        print("2. 使能锁紧")
        if input("选择 [1/2]: ").strip() != "2":
            return 0
        actuator.soft_limit_min_deg, actuator.soft_limit_max_deg = limits
        actuator.enable()
        print("1. 不回零")
        print("2. 2 秒后回零")
        if input("选择 [1/2]: ").strip() != "2":
            keep_enabled = True
            print("已锁紧")
            return 0
        if not limits[0] <= 0 <= limits[1]:
            raise RuntimeError("Y42 坐标零点不在刚采集的软限位内，拒绝回中")
        print("2 秒后回零")
        countdown(2)
        actuator.home_absolute_zero()
        position = wait_for_absolute_home(actuator)
        keep_enabled = True
        print("已回零：%.3f°" % position)
    finally:
        actuator.close(disable=not keep_enabled)
    return 0


def calibrate_crank(args: argparse.Namespace, config: dict, path: Path) -> int:
    if not bool(config.get("enabled", False)):
        raise RuntimeError("自动曲轴标定要求 roller_control.yaml 的 enabled: true")
    motor = config["motor"]
    firmware = str(motor.get("firmware", "x"))
    if str(motor.get("soft_limit_firmware", firmware)) != firmware:
        raise RuntimeError("soft limits use another firmware unit; rerun limits and write the result first")
    lower = float(motor["soft_limit_min_deg"])
    upper = float(motor["soft_limit_max_deg"])
    targets = [lower + (upper - lower) * index / (args.points - 1)
               for index in range(args.points)]
    actuator = build_actuator(config)
    sensor, pitch = build_pitch(config)
    measurements: list[tuple[float, float]] = []
    sensor.open()
    try:
        sensor.configure()
        actuator.open()
        actuator.enable(confirm=True)
        motor_status = actuator.read_motor_status()
        if not motor_status & 0x01:
            raise RuntimeError("Y42 is not enabled (status 0x%02X)" % motor_status)
        if motor_status & 0x08:
            raise RuntimeError("Y42 stall protection is active (status 0x%02X)" % motor_status)
        current = actuator.read_position_deg()
        previous_target = actuator.read_target_position_deg() if actuator.firmware == "emm" else current
        for target in targets:
            print("移动至电机 %.3f°" % target)
            movement_timeout_s = max(5.0, abs(target - current) / (args.speed_rpm * 6.0) + 5.0)
            if actuator.firmware == "emm":
                actuator.move_relative_target(target - previous_target, speed_rpm=args.speed_rpm,
                                             acceleration_rpm_s=args.acceleration_rpm_s,
                                             deceleration_rpm_s=args.deceleration_rpm_s)
                accepted_target = actuator.read_target_position_deg()
                if abs(accepted_target - target) > 1.0:
                    raise RuntimeError("Y42 did not accept %.3f°; target is %.3f°" % (
                        target, accepted_target))
                previous_target = accepted_target
            else:
                actuator.move_to_coordinate(target, current_angle_deg=current, speed_rpm=args.speed_rpm,
                                            acceleration_rpm_s=args.acceleration_rpm_s,
                                            deceleration_rpm_s=args.deceleration_rpm_s, confirm=True,
                                            confirm_timeout_s=movement_timeout_s)
            wait_for_position(actuator, target, timeout_s=movement_timeout_s)
            time.sleep(args.settle_s)
            actual = display_motor_position(actuator, config, "  采样")
            tube = average_tube_pitch(sensor, pitch, samples=args.samples, hz=args.sample_hz)
            measurements.append((actual, tube))
            print("  实际电机 %.3f°，水管 %.3f°" % (actual, tube))
            current = actual
    finally:
        actuator.close()
        sensor.close()
    crank_map = invert_crank_samples(measurements)
    print("\n建议写入：")
    print("  motor_deg_by_tube_angle: %s" % crank_map)
    if args.apply:
        update_config(path, crank_map=crank_map)
        print("已写入 %s" % path)
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Roller Y42 soft-limit and crank calibration")
    parser.add_argument("--control-config", default="config/roller_control.yaml")
    subparsers = parser.add_subparsers(dest="mode")
    limits = subparsers.add_parser("limits", help="manually capture Y42 safe travel with the shaft loose")
    limits.add_argument("--margin-deg", type=float, default=2.0)
    limits.add_argument("--apply", action="store_true", help="write measured soft limits to the config")
    crank = subparsers.add_parser("map", help="move Y42 slowly and sample the tube angle")
    crank.add_argument("--points", type=int, default=7)
    crank.add_argument("--speed-rpm", type=float, default=5.0)
    crank.add_argument("--acceleration-rpm-s", type=int, default=30)
    crank.add_argument("--deceleration-rpm-s", type=int, default=30)
    crank.add_argument("--settle-s", type=float, default=2.0)
    crank.add_argument("--samples", type=int, default=40)
    crank.add_argument("--sample-hz", type=float, default=20.0)
    crank.add_argument("--apply", action="store_true", help="write the measured inverse crank map to the config")
    return parser.parse_args()


def choose_interactive_mode(args: argparse.Namespace) -> argparse.Namespace:
    print("滚珠机构标定")
    print("1. 松轴后手动采集电机软限位（不驱动电机）")
    print("2. 锁紧曲轴后自动采集电机角度与水管倾角关系（会驱动电机）")
    choice = input("选择 [1/2]: ").strip()
    if choice == "1":
        args.mode = "limits"
        margin = input("每端安全余量（度）[2]: ").strip()
        args.margin_deg = float(margin or 2.0)
        print("1. 不写入")
        print("2. 写入软限位")
        args.apply = input("选择 [1/2]: ").strip() == "2"
        return args
    if choice == "2":
        args.mode = "map"
        print("1. 取消")
        print("2. 已锁紧且急停有效，继续")
        if input("选择 [1/2]: ").strip() != "2":
            raise SystemExit("已取消")
        points = input("采样点数 [7]: ").strip()
        args.points = int(points or 7)
        args.speed_rpm = 5.0
        args.acceleration_rpm_s = 30
        args.deceleration_rpm_s = 30
        args.settle_s = 2.0
        args.samples = 40
        args.sample_hz = 20.0
        print("1. 不写入")
        print("2. 写入标定表")
        args.apply = input("选择 [1/2]: ").strip() == "2"
        return args
    raise SystemExit("请选择 1 或 2")


def main() -> int:
    args = parse_args()
    if args.mode is None:
        args = choose_interactive_mode(args)
    if args.mode == "map" and (args.points < 3 or args.speed_rpm <= 0 or args.acceleration_rpm_s < 0
                                or args.deceleration_rpm_s < 0 or args.settle_s < 0
                                or args.samples <= 0 or args.sample_hz <= 0):
        raise SystemExit("invalid automatic calibration parameters")
    path = Path(args.control_config)
    config = load_roller_control(path)
    if args.mode == "limits":
        return calibrate_limits(args, config, path)
    return calibrate_crank(args, config, path)


if __name__ == "__main__":
    raise SystemExit(main())
