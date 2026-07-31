"""Direct CAN roller controller that subscribes to the existing vision daemon."""

from __future__ import annotations

import argparse
import logging
from queue import Empty, Queue
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
from robot_car.roller_control.icm42688 import Icm42688, sensor_from_config
from robot_car.roller_control.y42_actuator import Y42Actuator


LOG = logging.getLogger(__name__)


class RollerControlDaemon:
    def __init__(self, app_config: dict, control_config: dict, *, armed: bool,
                 dry_run: bool, target_mm: float | None, telemetry_hz: float,
                 task: int = 1) -> None:
        self.app_config = app_config
        self.control_config = control_config
        self.armed = armed and bool(control_config.get("enabled", False))
        self.dry_run = dry_run
        self.stop_event = threading.Event()
        imu = control_config["imu"]
        motor = control_config["motor"]
        estimator = control_config["estimator"]
        self.sensor = sensor_from_config(imu)
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
        self.target_mm = 0.0 if task in (1, 2) else float(
            control_config.get("target_mm", 0) if target_mm is None else target_mm)
        self.task = task
        self.sequence_state = "CENTERING" if task == 2 else "CONTINUOUS"
        self.sequence_target_mm = 0.0
        self.sequence_stable_since_s: float | None = None
        self.sequence_prompted = False
        self.sequence_started_s: float | None = None
        self.sequence_confirmations: Queue[str] = Queue()
        self.sequence_input_thread: threading.Thread | None = None
        if telemetry_hz < 0 or telemetry_hz > 20:
            raise ValueError("telemetry_hz must be in 0..20")
        self.telemetry_interval_s = 0.0 if telemetry_hz == 0 else 1.0 / telemetry_hz
        self.feedback_period_ms = 0 if telemetry_hz == 0 else max(50, round(1000.0 / telemetry_hz))
        self.next_telemetry_s = 0.0
        self.motor_position_deg: float | None = None
        self.motor_home_status: int | None = None
        self.motor_status: int | None = None
        self.feedback_enabled = False
        self.subscriber = VisionEventSubscriber(app_config["runtime"]["vision_socket"])
        self.ball: BallState | None = None
        self.last_command_s = 0.0
        self.last_safe = False

    def run(self) -> None:
        self.sensor.open()
        self.sensor.configure()
        self.actuator.open()
        if self.telemetry_interval_s and not self.dry_run:
            self.actuator.configure_feedback("home-and-status", self.feedback_period_ms)
            self.feedback_enabled = True
        if self.armed:
            self.actuator.enable()
        LOG.info("roller CAN controller started; armed=%s dry_run=%s target_mm=%.1f",
                 self.armed, self.dry_run, self.target_mm)
        try:
            while not self.stop_event.is_set():
                self._receive_event()
                self._receive_motor_feedback()
                now_s = time.monotonic()
                if now_s - self.last_command_s >= self.command_interval_s:
                    estimate = self.pitch.update(self.sensor.sample())
                    self._control(now_s, estimate.pitch_deg)
                    self.last_command_s = estimate.timestamp_s
                self.stop_event.wait(0.001)
        finally:
            if self.feedback_enabled:
                self.actuator.configure_feedback("position", 0)
                self.actuator.configure_feedback("home-and-status", 0)
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
        if (self.task == 2 and self.sequence_state in {"TO_POSITIVE", "TO_NEGATIVE"}
                and self.sequence_started_s is not None
                and now_s - self.sequence_started_s > 5.0):
            self._sequence_timeout(now_s)
            return
        ball = self.ball
        now_ms = int(now_s * 1000)
        if ball is None or now_ms - ball.timestamp_ms > self.state_timeout_ms:
            self.controller.reset()
            self._safe_stop("ball state timeout")
            return
        self.last_safe = False
        if self.task == 2:
            self._update_two_point_sequence(now_s)
            if self.sequence_state == "COMPLETE":
                return
            target_mm = self.sequence_target_mm
        else:
            target_mm = self.target_mm
        command = self.controller.step(target_mm, ball, tube_angle_deg,
                                       self.command_interval_s)
        if self.armed or self.dry_run:
            self.actuator.move_absolute(command.target_motor_angle_deg, speed_rpm=self.speed_rpm,
                                        acceleration_rpm_s=self.acceleration_rpm_s,
                                        deceleration_rpm_s=self.deceleration_rpm_s)
        LOG.debug("ball=%.1f mm v=%.1f mm/s a=%.1f mm/s2 tube=%.2f deg target=%.2f deg motor=%.2f deg",
                  ball.position_mm, ball.velocity_mm_s, ball.acceleration_mm_s2, tube_angle_deg,
                  command.target_tube_angle_deg, command.target_motor_angle_deg)
        if self.telemetry_interval_s and now_s >= self.next_telemetry_s:
            self.next_telemetry_s = now_s + self.telemetry_interval_s
            try:
                self.motor_position_deg = self.actuator.read_position_deg(timeout_s=0.05)
            except RuntimeError:
                self.motor_position_deg = None
            position = "-" if self.motor_position_deg is None else f"{self.motor_position_deg:.2f}"
            status = "-" if self.motor_status is None else f"0x{self.motor_status:02X}"
            home = "-" if self.motor_home_status is None else f"0x{self.motor_home_status:02X}"
            LOG.info("telemetry ball=%.1fmm v=%.1fmm/s tube=%.2fdeg target=%.2fdeg cmd=%.2fdeg "
                     "motor=%sdeg status=%s home=%s",
                     ball.position_mm, ball.velocity_mm_s, tube_angle_deg,
                     command.target_tube_angle_deg, command.target_motor_angle_deg,
                     position, status, home)

    def _read_sequence_confirmation(self) -> None:
        try:
            answer = input("请输入 Y 并回车，开始 +5cm/-5cm 计时：")
        except (EOFError, OSError):
            answer = ""
        self.sequence_confirmations.put(answer.strip().lower())

    def _update_two_point_sequence(self, now_s: float) -> None:
        ball = self.ball
        if ball is None:
            return
        target = self.sequence_target_mm
        stable = abs(ball.position_mm - target) <= 10.0 and abs(ball.velocity_mm_s) <= 40.0
        if stable:
            if self.sequence_stable_since_s is None:
                self.sequence_stable_since_s = now_s
        else:
            self.sequence_stable_since_s = None
        held_s = (0.0 if self.sequence_stable_since_s is None
                  else now_s - self.sequence_stable_since_s)
        if (self.sequence_state in {"TO_POSITIVE", "TO_NEGATIVE"}
                and self.sequence_started_s is not None
                and now_s - self.sequence_started_s > 5.0):
            self._sequence_timeout(now_s)
            return
        if self.sequence_state == "CENTERING":
            if held_s >= 0.30 and not self.sequence_prompted:
                self.sequence_prompted = True
                self.sequence_state = "WAIT_CONFIRM"
                LOG.info("钢球已回中并稳定，请确认后开始题3计时")
                print("钢球已回中并稳定。输入 Y 并回车后开始计时。", flush=True)
                self.sequence_input_thread = threading.Thread(
                    target=self._read_sequence_confirmation,
                    name="roller-sequence-input", daemon=True)
                self.sequence_input_thread.start()
        elif self.sequence_state == "WAIT_CONFIRM":
            try:
                answer = self.sequence_confirmations.get_nowait()
            except Empty:
                return
            if answer not in {"y", "yes"}:
                self.sequence_prompted = False
                self.sequence_state = "CENTERING"
                self.sequence_stable_since_s = None
                print("未确认，继续回中；稳定后会再次提示。", flush=True)
                return
            self.sequence_started_s = now_s
            self.sequence_state = "TO_POSITIVE"
            self.sequence_target_mm = 50.0
            self.sequence_stable_since_s = None
            self.controller.reset()
            print("已开始计时，前往 +5cm。", flush=True)
        elif self.sequence_state == "TO_POSITIVE" and held_s >= 0.20:
            self.sequence_state = "TO_NEGATIVE"
            self.sequence_target_mm = -50.0
            self.sequence_stable_since_s = None
            self.controller.reset()
            print("+5cm 已稳定，前往 -5cm。", flush=True)
        elif self.sequence_state == "TO_NEGATIVE" and held_s >= 0.20:
            elapsed = 0.0 if self.sequence_started_s is None else now_s - self.sequence_started_s
            self.sequence_state = "COMPLETE"
            print(f"-5cm 已稳定，计时结束：{elapsed:.3f}s", flush=True)
            LOG.info("题3两点运动完成，总时长 %.3fs", elapsed)
            self.stop_event.set()

    def _sequence_timeout(self, now_s: float) -> None:
        elapsed = 0.0 if self.sequence_started_s is None else now_s - self.sequence_started_s
        self.sequence_state = "COMPLETE"
        print(f"题3两点运动超过5s，已停止（用时 {elapsed:.3f}s）。", flush=True)
        LOG.warning("题3两点运动超时：%.3fs", elapsed)
        self.stop_event.set()

    def _receive_motor_feedback(self) -> None:
        if not self.feedback_enabled:
            return
        for _ in range(4):
            feedback = self.actuator.receive_feedback()
            if feedback is None:
                return
            item, value = feedback
            if item == "position":
                self.motor_position_deg = float(value)
            else:
                self.motor_home_status, self.motor_status = value

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
    parser.add_argument("--telemetry-hz", type=float, default=0,
                        help="log vision/IMU/Y42 feedback at 0..20 Hz; 0 disables feedback")
    parser.add_argument("--task", type=int, choices=(1, 2), default=1,
                        help="1: continuously hold center; 2: center then +50/-50 cm test")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        app_config = load_config(args.config_dir)
        paths = ensure_runtime_dirs(app_config)
        configure_logging("rollercontrold", app_config["runtime"].get("log_level", "INFO"), paths["logs"])
        control_path = args.control_config or str(Path(args.config_dir) / "roller_control.yaml")
        daemon = RollerControlDaemon(app_config, load_roller_control(control_path), armed=args.arm,
                                     dry_run=args.dry_run, target_mm=args.target_mm,
                                     telemetry_hz=args.telemetry_hz, task=args.task)
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
