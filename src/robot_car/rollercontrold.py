"""Direct CAN roller controller that subscribes to the existing vision daemon."""

from __future__ import annotations

import argparse
import logging
import math
from queue import Empty, Full, Queue
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

CENTER_CONFIRM_WINDOW_MM = 5.0
CENTER_CONFIRM_HOLD_S = 2.1
FINAL_STABLE_WINDOW_MM = 10.0
FINAL_STABLE_VELOCITY_MM_S = 40.0
FINAL_STABLE_HOLD_S = 0.20
POSITIVE_REVERSAL_MIN_MM = 40.0
POSITIVE_REVERSAL_MAX_MM = 50.0
POSITIVE_REVERSAL_VELOCITY_MM_S = 8.0


class RollerControlDaemon:
    def __init__(self, app_config: dict, control_config: dict, *, armed: bool,
                 dry_run: bool, target_mm: float | None, telemetry_hz: float,
                 task: int = 1, show_tx: bool = False, home: bool = False,
                 auto_confirm: bool = False, start_immediately: bool = False,
                 quiet: bool = False, external_vision_events: bool = False) -> None:
        self.app_config = app_config
        self.control_config = control_config
        self.armed = armed and bool(control_config.get("enabled", False))
        self.dry_run = dry_run
        self.home_requested = home
        self.auto_confirm = auto_confirm
        self.start_immediately = start_immediately
        self.quiet = quiet
        self.external_vision_events = external_vision_events
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
        two_point = control_config.get("two_point", {})
        if not isinstance(two_point, dict):
            raise ValueError("roller two_point configuration must be a mapping")
        self.two_point = two_point
        self.center_controller = build_controller(control_config)
        self.two_point_controller = build_controller(control_config, pid_profile=two_point)
        self.controller = (self.two_point_controller if task == 2 and start_immediately
                           else self.center_controller)
        self.actuator = Y42Actuator(interface=str(motor["can_interface"]),
                                    address=int(motor["address"]),
                                    firmware=str(motor.get("firmware", "x")),
                                    packet_gap_ms=float(motor.get("packet_gap_ms", 3)), dry_run=dry_run,
                                    soft_limit_min_deg=float(motor["soft_limit_min_deg"]),
                                    soft_limit_max_deg=float(motor["soft_limit_max_deg"]),
                                    pulses_per_revolution=int(motor.get("pulses_per_revolution", 3200)),
                                    show_tx=show_tx)
        self.speed_rpm = float(motor["speed_rpm"])
        self.acceleration_rpm_s = int(motor["acceleration_rpm_s"])
        self.deceleration_rpm_s = int(motor["deceleration_rpm_s"])
        self.two_point_speed_rpm = float(two_point.get("speed_rpm", self.speed_rpm))
        self.two_point_acceleration_rpm_s = int(two_point.get("acceleration_rpm_s",
                                                               self.acceleration_rpm_s))
        self.two_point_deceleration_rpm_s = int(two_point.get("deceleration_rpm_s",
                                                               self.deceleration_rpm_s))
        self.positive_drive_target_mm = float(two_point.get("positive_drive_target_mm", 50.0))
        self.positive_reversal_min_mm = float(two_point.get("positive_reversal_min_mm",
                                                             POSITIVE_REVERSAL_MIN_MM))
        self.positive_reversal_max_mm = float(two_point.get("positive_reversal_max_mm",
                                                             POSITIVE_REVERSAL_MAX_MM))
        self.positive_reversal_velocity_mm_s = float(two_point.get(
            "positive_reversal_velocity_mm_s", POSITIVE_REVERSAL_VELOCITY_MM_S))
        self.final_stable_window_mm = float(two_point.get("final_stable_window_mm",
                                                           FINAL_STABLE_WINDOW_MM))
        self.final_stable_velocity_mm_s = float(two_point.get("final_stable_velocity_mm_s",
                                                               FINAL_STABLE_VELOCITY_MM_S))
        self.final_stable_hold_s = float(two_point.get("final_stable_hold_s",
                                                        FINAL_STABLE_HOLD_S))
        if (self.two_point_speed_rpm <= 0 or self.two_point_acceleration_rpm_s < 0
                or self.two_point_deceleration_rpm_s < 0
                or not self.center_controller.target_min_mm <= self.positive_drive_target_mm
                <= self.center_controller.target_max_mm
                or not 0 <= self.positive_reversal_min_mm <= self.positive_reversal_max_mm
                or self.positive_reversal_velocity_mm_s < 0
                or self.final_stable_window_mm <= 0 or self.final_stable_velocity_mm_s < 0
                or self.final_stable_hold_s < 0):
            raise ValueError("roller two_point configuration is invalid")
        self.command_interval_s = 1.0 / float(motor.get("command_hz", 40))
        self.state_timeout_ms = int(control_config.get("state_timeout_ms", 120))
        self.target_mm = float(control_config.get("target_mm", 0) if target_mm is None else target_mm)
        self.task = task
        self.sequence_state = "TO_POSITIVE" if task == 2 and start_immediately else (
            "CENTERING" if task == 2 else "CONTINUOUS")
        self.sequence_target_mm = 50.0 if task == 2 and start_immediately else 0.0
        self.sequence_stable_since_s: float | None = None
        self.sequence_prompted = False
        self.sequence_started_s: float | None = time.monotonic() if task == 2 and start_immediately else None
        self.sequence_timed_out = False
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
        self._external_vision_queue = Queue(maxsize=1) if external_vision_events else None
        self.subscriber = (None if external_vision_events else
                           VisionEventSubscriber(app_config["runtime"]["vision_socket"]))
        self.ball: BallState | None = None
        self.received_ball_state = False
        self.reported_stale_ball_state = False
        self.last_ball_frame_id: int | None = None
        self.last_ball_event_age_ms: int | None = None
        self.accepted_ball_events = 0
        self.discarded_stale_ball_events = 0
        self._last_stale_ball_log_ms = 0
        self.last_command_s = 0.0
        self.last_safe = False
        self._feedforward_lock = threading.Lock()
        self._feedforward_mm_s2 = 0.0

    def set_feedforward_mm_s2(self, value: float | None) -> None:
        """Set the latest M0-provided, tube-axis acceleration feedforward.

        Missing, malformed, or non-finite telemetry must never leave a stale
        acceleration command active, so it is represented as zero.
        """
        try:
            feedforward = float(value) if value is not None else 0.0
        except (TypeError, ValueError):
            feedforward = 0.0
        if not math.isfinite(feedforward):
            feedforward = 0.0
        with self._feedforward_lock:
            self._feedforward_mm_s2 = feedforward

    def _feedforward(self) -> float:
        with self._feedforward_lock:
            return self._feedforward_mm_s2

    def _announce(self, message: str) -> None:
        if not self.quiet:
            print(message, flush=True)

    def run(self) -> None:
        self.sensor.open()
        self.sensor.configure()
        self.actuator.open()
        try:
            if self.telemetry_interval_s and not self.dry_run:
                self.actuator.configure_feedback("home-and-status", self.feedback_period_ms)
                self.feedback_enabled = True
            if self.home_requested and not (self.armed or self.dry_run):
                raise RuntimeError("--home 会驱动电机，请同时传入 --arm（且 config.enabled 必须为 true）")
            if self.armed:
                self.actuator.enable()
            if self.home_requested:
                self._announce("开始回零（绝对坐标零点）...")
                position = self.actuator.home_absolute_zero(wait=True, timeout_s=30.0)
                if position is not None:
                    LOG.info("absolute home complete: %.3f deg", position)
            LOG.info("roller CAN controller started; task=%d armed=%s dry_run=%s target_mm=%.1f",
                     self.task, self.armed, self.dry_run, self.target_mm)
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
            if self.subscriber is not None:
                self.subscriber.close()
            self.actuator.close()
            self.sensor.close()

    def _receive_event(self) -> None:
        if self._external_vision_queue is not None:
            try:
                event = self._external_vision_queue.get_nowait()
            except Empty:
                return
        else:
            if self.subscriber is None:
                return
            event = self.subscriber.receive(timeout=0.001)
        if event is None or event.event_type != "BALL_BALANCE_STATE":
            return
        now_ms = time.monotonic_ns() // 1_000_000
        age_ms = max(0, now_ms - event.timestamp_monotonic_ms)
        if event.is_expired(now_ms):
            self.discarded_stale_ball_events += 1
            if now_ms - self._last_stale_ball_log_ms >= 1000:
                LOG.warning("discarded stale ball event: frame=%d age=%dms ttl=%dms discarded=%d",
                            event.frame_id, age_ms, event.ttl_ms,
                            self.discarded_stale_ball_events)
                self._last_stale_ball_log_ms = now_ms
            if not self.reported_stale_ball_state:
                self.reported_stale_ball_state = True
                self._announce("收到钢球视觉数据但已过期；请检查视觉推理延迟和 TTL 配置。")
            return
        try:
            self.ball = self.ball_estimator.update(event.timestamp_monotonic_ms,
                                                    float(event.payload["error_mm"]))
            self.last_ball_frame_id = event.frame_id
            self.last_ball_event_age_ms = age_ms
            self.accepted_ball_events += 1
            if not self.received_ball_state:
                self.received_ball_state = True
                self._announce("已接收滚珠视觉数据，开始回中判定。")
        except (KeyError, TypeError, ValueError) as error:
            LOG.warning("ignored invalid ball state: %s", error)

    def submit_vision_event(self, event: object) -> None:
        """Supply the latest vehicled vision event to a direct controller.

        Standalone roller control keeps its Unix-socket subscription.  The
        high-level vehicle daemon calls this method only while it owns the
        direct roller controller, and a one-slot queue prevents an old frame
        from adding latency to the motor loop.
        """
        if self._external_vision_queue is None:
            return
        if getattr(event, "event_type", None) != "BALL_BALANCE_STATE":
            return
        try:
            self._external_vision_queue.put_nowait(event)
        except Full:
            try:
                self._external_vision_queue.get_nowait()
            except Empty:
                pass
            try:
                self._external_vision_queue.put_nowait(event)
            except Full:
                return

    def _control(self, now_s: float, tube_angle_deg: float) -> None:
        if (self.task == 2 and self.sequence_state in {"TO_POSITIVE", "TO_NEGATIVE"}
                and self.sequence_started_s is not None
                and now_s - self.sequence_started_s > 5.0
                and not self.sequence_timed_out):
            self._sequence_timeout(now_s)
        ball = self.ball
        now_ms = int(now_s * 1000)
        if ball is None or now_ms - ball.timestamp_ms > self.state_timeout_ms:
            self.controller.reset()
            self._safe_stop("ball state timeout")
            return
        self.last_safe = False
        if self.task == 2:
            self._update_two_point_sequence(now_s)
            target_mm = self.sequence_target_mm
            if self.sequence_state == "TO_POSITIVE":
                target_mm = self.positive_drive_target_mm
        else:
            target_mm = self.target_mm
        command = self.controller.step(target_mm, ball, tube_angle_deg,
                                       self.command_interval_s,
                                       target_acceleration_mm_s2=self._feedforward())
        if self.task == 2 and self.sequence_state in {"TO_POSITIVE", "TO_NEGATIVE"}:
            speed_rpm = self.two_point_speed_rpm
            acceleration_rpm_s = self.two_point_acceleration_rpm_s
            deceleration_rpm_s = self.two_point_deceleration_rpm_s
        else:
            speed_rpm = self.speed_rpm
            acceleration_rpm_s = self.acceleration_rpm_s
            deceleration_rpm_s = self.deceleration_rpm_s
        if self.armed or self.dry_run:
            self.actuator.move_absolute(command.target_motor_angle_deg, speed_rpm=speed_rpm,
                                        acceleration_rpm_s=acceleration_rpm_s,
                                        deceleration_rpm_s=deceleration_rpm_s)
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
            phase = self.sequence_state if self.task == 2 else "CONTINUOUS"
            ball_target = self.sequence_target_mm if self.task == 2 else self.target_mm
            elapsed = "-" if self.sequence_started_s is None else f"{now_s - self.sequence_started_s:.3f}s"
            LOG.info("telemetry phase=%s ball_target=%.1fmm elapsed=%s ball=%.1fmm v=%.1fmm/s "
                     "tube=%.2fdeg target=%.2fdeg cmd=%.2fdeg "
                     "motor=%sdeg status=%s home=%s",
                     phase, ball_target, elapsed, ball.position_mm, ball.velocity_mm_s, tube_angle_deg,
                     command.target_tube_angle_deg, command.target_motor_angle_deg,
                     position, status, home)

    def _read_sequence_confirmation(self) -> None:
        try:
            answer = input("请输入 Y 并回车，开始 +5cm/-5cm 计时：")
        except (EOFError, OSError):
            answer = ""
        self.sequence_confirmations.put(answer.strip().lower())

    def _begin_two_point_sequence(self, now_s: float, *, automatic: bool) -> None:
        self.sequence_state = "TO_POSITIVE"
        self.sequence_started_s = now_s
        self.sequence_target_mm = 50.0
        self.sequence_stable_since_s = None
        self.controller = self.two_point_controller
        self.controller.reset()
        prefix = "自动确认：" if automatic else "已确认："
        self._announce(prefix + "开始计时，前往 +5cm。")

    def _update_two_point_sequence(self, now_s: float) -> None:
        ball = self.ball
        if ball is None:
            return
        target = self.sequence_target_mm
        if self.sequence_state == "CENTERING":
            # Pixel-to-mm rounding makes the differentiated velocity noisy at
            # rest.  The initial operator confirmation needs a stable visual
            # center, not a velocity estimate that can be reset by one pixel.
            stable = abs(ball.position_mm) <= CENTER_CONFIRM_WINDOW_MM
        else:
            stable = (abs(ball.position_mm - target) <= self.final_stable_window_mm
                      and abs(ball.velocity_mm_s) <= self.final_stable_velocity_mm_s)
        if stable:
            if self.sequence_stable_since_s is None:
                self.sequence_stable_since_s = now_s
        else:
            self.sequence_stable_since_s = None
        held_s = (0.0 if self.sequence_stable_since_s is None
                  else now_s - self.sequence_stable_since_s)
        if (self.sequence_state in {"TO_POSITIVE", "TO_NEGATIVE"}
                and self.sequence_started_s is not None
                and now_s - self.sequence_started_s > 5.0
                and not self.sequence_timed_out):
            self._sequence_timeout(now_s)
        if self.sequence_state == "CENTERING":
            if held_s >= CENTER_CONFIRM_HOLD_S and not self.sequence_prompted:
                self.sequence_prompted = True
                if self.auto_confirm:
                    self._begin_two_point_sequence(now_s, automatic=True)
                else:
                    self.sequence_state = "WAIT_CONFIRM"
                    LOG.info("钢球已回中并稳定，请确认后开始题3计时")
                    self._announce("钢球已在 ±5mm 内稳定超过2s。输入 Y 并回车后开始计时。")
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
                self._announce("未确认，继续回中；稳定后会再次提示。")
                return
            self._begin_two_point_sequence(now_s, automatic=False)
        elif (self.sequence_state == "TO_POSITIVE"
              and self.positive_reversal_min_mm <= ball.position_mm <= self.positive_reversal_max_mm
              and abs(ball.velocity_mm_s) <= self.positive_reversal_velocity_mm_s):
            self.sequence_state = "TO_NEGATIVE"
            self.sequence_target_mm = -50.0
            self.sequence_stable_since_s = None
            self.controller.reset()
            self._announce("已到达 +4~+5cm 且速度接近零，开始平滑返回 -5cm。")
        elif self.sequence_state == "TO_NEGATIVE" and held_s >= self.final_stable_hold_s:
            elapsed = 0.0 if self.sequence_started_s is None else now_s - self.sequence_started_s
            self.sequence_state = "COMPLETE"
            timeout_note = "（已超过5s）" if self.sequence_timed_out else ""
            self._announce(f"-5cm 已稳定，计时结束：{elapsed:.3f}s{timeout_note}；继续保持 -5cm。")
            LOG.info("题3两点运动完成，总时长 %.3fs%s；继续保持 -5cm", elapsed, timeout_note)

    def _sequence_timeout(self, now_s: float) -> None:
        elapsed = 0.0 if self.sequence_started_s is None else now_s - self.sequence_started_s
        self.sequence_timed_out = True
        self._announce(f"题3两点运动已超过5s（当前 {elapsed:.3f}s），继续运行至 -5cm。")
        LOG.warning("题3两点运动超时：%.3fs", elapsed)

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
        now_ms = time.monotonic_ns() // 1_000_000
        ball_age_ms = ("-" if self.ball is None else
                       str(max(0, now_ms - self.ball.timestamp_ms)))
        received_age_ms = getattr(self, "last_ball_event_age_ms", None)
        LOG.warning("roller controller safety hold: %s; last_frame=%s age=%sms "
                    "received_age=%sms accepted=%d stale_discarded=%d", reason,
                    self.last_ball_frame_id if self.last_ball_frame_id is not None else "-",
                    ball_age_ms, "-" if received_age_ms is None else received_age_ms,
                    self.accepted_ball_events, self.discarded_stale_ball_events)
        if reason == "ball state timeout" and not self.received_ball_state:
            self._announce("等待有效钢球识别数据；请确认网页持续显示钢球误差。")
        if self.armed or self.dry_run:
            if not self.actuator.hold_current_position(
                    speed_rpm=self.speed_rpm, acceleration_rpm_s=self.acceleration_rpm_s,
                    deceleration_rpm_s=self.deceleration_rpm_s):
                LOG.warning("roller safety hold failed; falling back to immediate stop")
                self.actuator.stop()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Direct Y42 CAN roller controller")
    parser.add_argument("--config-dir", default="config")
    parser.add_argument("--control-config", default=None)
    parser.add_argument("--target-mm", type=float, default=None)
    parser.add_argument("--arm", action="store_true", help="allow the configured motor to be enabled")
    parser.add_argument("--dry-run", action="store_true", help="不发送 CAN，仅运行控制逻辑；显示报文请再加 --show-tx")
    parser.add_argument("--show-tx", action="store_true", help="显示发送的 CAN TX 帧（默认不显示）")
    parser.add_argument("--home", action="store_true", help="启动控制前执行一次绝对坐标回零；需要 --arm")
    parser.add_argument("--auto-confirm", action="store_true",
                        help="调参用：回中稳定后自动确认；默认仍等待人工输入 Y")
    parser.add_argument("--telemetry-hz", type=float, default=0,
                        help="log vision/IMU/Y42 feedback at 0..20 Hz; 0 disables feedback")
    parser.add_argument("--task", type=int, choices=(1, 2), default=1,
                        help="1: continuously hold center; 2: center then +50/-50 cm test (timeout reports only)")
    parser.add_argument("--immediate", action="store_true",
                        help="题3专用：假定钢球已手工置于 0 点，立即执行 +50mm -> -50mm")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        app_config = load_config(args.config_dir)
        paths = ensure_runtime_dirs(app_config)
        configure_logging("rollercontrold", app_config["runtime"].get("log_level", "INFO"), paths["logs"])
        control_path = args.control_config or str(Path(args.config_dir) / "roller_control.yaml")
        if args.immediate and args.task != 2:
            raise ValueError("--immediate 仅适用于 --task 2")
        daemon = RollerControlDaemon(app_config, load_roller_control(control_path), armed=args.arm,
                                     dry_run=args.dry_run, target_mm=args.target_mm,
                                     telemetry_hz=args.telemetry_hz, task=args.task,
                                     show_tx=args.show_tx, home=args.home,
                                     auto_confirm=args.auto_confirm,
                                     start_immediately=args.immediate)
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
