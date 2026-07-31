"""Direct CAN roller controller that subscribes to the existing vision daemon."""

from __future__ import annotations

import argparse
import logging
import signal
import threading
import time
from pathlib import Path

from robot_car.config import ensure_runtime_dirs, load_config
from robot_car.ipc.vision_socket import VisionEventSubscriber
from robot_car.observability.logging import configure_logging
from robot_car.roller_control.attitude import PitchEstimator
from robot_car.roller_control.config import build_controller, load_roller_control
from robot_car.roller_control.control import BallState, BallStateEstimator
from robot_car.roller_control.icm42688 import Icm42688
from robot_car.roller_control.y42_actuator import Y42Actuator


LOG = logging.getLogger(__name__)


class RollerControlDaemon:
    def __init__(self, app_config: dict, control_config: dict, *, armed: bool,
                 dry_run: bool, target_mm: float | None) -> None:
        self.app_config = app_config
        self.control_config = control_config
        self.armed = armed and bool(control_config.get("enabled", False))
        self.dry_run = dry_run
        self.stop_event = threading.Event()
        imu = control_config["imu"]
        motor = control_config["motor"]
        estimator = control_config["estimator"]
        self.sensor = Icm42688(int(imu["spi_bus"]), int(imu["chip_select"]),
                               speed_hz=int(imu.get("speed_hz", 1_000_000)),
                               mode=int(imu.get("mode", 0)))
        self.pitch = PitchEstimator(
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
        self.ball_estimator = BallStateEstimator(
            velocity_alpha=float(estimator.get("velocity_alpha", 0.35)),
            acceleration_alpha=float(estimator.get("acceleration_alpha", 0.20)),
            max_gap_ms=int(estimator.get("max_gap_ms", 250)))
        self.controller = build_controller(control_config)
        self.actuator = Y42Actuator(interface=str(motor["can_interface"]),
                                    address=int(motor["address"]),
                                    firmware=str(motor.get("firmware", "x")),
                                    packet_gap_ms=float(motor.get("packet_gap_ms", 3)), dry_run=dry_run,
                                    soft_limit_min_deg=float(motor["soft_limit_min_deg"]),
                                    soft_limit_max_deg=float(motor["soft_limit_max_deg"]),
                                    pulses_per_revolution=int(motor.get("pulses_per_revolution", 3200)))
        self.speed_rpm = float(motor["speed_rpm"])
        self.acceleration_rpm_s = int(motor["acceleration_rpm_s"])
        self.deceleration_rpm_s = int(motor["deceleration_rpm_s"])
        self.command_interval_s = 1.0 / float(motor.get("command_hz", 40))
        self.state_timeout_ms = int(control_config.get("state_timeout_ms", 120))
        self.target_mm = float(control_config.get("target_mm", 0) if target_mm is None else target_mm)
        self.subscriber = VisionEventSubscriber(app_config["runtime"]["vision_socket"])
        self.ball: BallState | None = None
        self.last_command_s = 0.0
        self.last_safe = False

    def run(self) -> None:
        self.sensor.open()
        self.sensor.configure()
        self.actuator.open()
        if self.armed:
            self.actuator.enable()
        LOG.info("roller CAN controller started; armed=%s dry_run=%s target_mm=%.1f",
                 self.armed, self.dry_run, self.target_mm)
        try:
            while not self.stop_event.is_set():
                self._receive_event()
                now_s = time.monotonic()
                if now_s - self.last_command_s >= self.command_interval_s:
                    estimate = self.pitch.update(self.sensor.sample())
                    self._control(now_s, estimate.pitch_deg)
                    self.last_command_s = estimate.timestamp_s
                self.stop_event.wait(0.001)
        finally:
            self.subscriber.close()
            self.actuator.close()
            self.sensor.close()

    def _receive_event(self) -> None:
        event = self.subscriber.receive(timeout=0.001)
        if event is None or event.event_type != "BALL_BALANCE_STATE":
            return
        now_ms = time.monotonic_ns() // 1_000_000
        if event.is_expired(now_ms):
            return
        try:
            self.ball = self.ball_estimator.update(event.timestamp_monotonic_ms,
                                                    float(event.payload["error_mm"]))
        except (KeyError, TypeError, ValueError) as error:
            LOG.warning("ignored invalid ball state: %s", error)

    def _control(self, now_s: float, tube_angle_deg: float) -> None:
        ball = self.ball
        now_ms = int(now_s * 1000)
        if ball is None or now_ms - ball.timestamp_ms > self.state_timeout_ms:
            self.controller.reset()
            self._safe_stop("ball state timeout")
            return
        self.last_safe = False
        command = self.controller.step(self.target_mm, ball, tube_angle_deg,
                                       self.command_interval_s)
        if self.armed or self.dry_run:
            self.actuator.move_absolute(command.target_motor_angle_deg, speed_rpm=self.speed_rpm,
                                        acceleration_rpm_s=self.acceleration_rpm_s,
                                        deceleration_rpm_s=self.deceleration_rpm_s)
        LOG.debug("ball=%.1f mm v=%.1f mm/s a=%.1f mm/s2 tube=%.2f deg target=%.2f deg motor=%.2f deg",
                  ball.position_mm, ball.velocity_mm_s, ball.acceleration_mm_s2, tube_angle_deg,
                  command.target_tube_angle_deg, command.target_motor_angle_deg)

    def _safe_stop(self, reason: str) -> None:
        if self.last_safe:
            return
        self.last_safe = True
        LOG.warning("roller controller safety stop: %s", reason)
        if self.armed or self.dry_run:
            self.actuator.stop()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Direct Y42 CAN roller controller")
    parser.add_argument("--config-dir", default="config")
    parser.add_argument("--control-config", default=None)
    parser.add_argument("--target-mm", type=float, default=None)
    parser.add_argument("--arm", action="store_true", help="allow the configured motor to be enabled")
    parser.add_argument("--dry-run", action="store_true", help="print Y42 CAN frames without sending them")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        app_config = load_config(args.config_dir)
        paths = ensure_runtime_dirs(app_config)
        configure_logging("rollercontrold", app_config["runtime"].get("log_level", "INFO"), paths["logs"])
        control_path = args.control_config or str(Path(args.config_dir) / "roller_control.yaml")
        daemon = RollerControlDaemon(app_config, load_roller_control(control_path), armed=args.arm,
                                     dry_run=args.dry_run, target_mm=args.target_mm)
    except Exception as error:
        logging.basicConfig(level=logging.INFO)
        LOG.error("safe startup failure: %s", error)
        return 2
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda _signum, _frame: daemon.stop_event.set())
    try:
        daemon.run()
    except Exception as error:
        LOG.error("roller controller stopped after a hardware or control error: %s", error)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
