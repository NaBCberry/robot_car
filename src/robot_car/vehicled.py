"""Exclusive high-level vehicle decision and MSPM0 gateway daemon."""

from __future__ import annotations

import argparse
import logging
import signal
import threading
import time
from pathlib import Path
from typing import Any, Dict

from robot_car.config import ensure_runtime_dirs, load_config
from robot_car.decision.control_arbiter import ControlArbiter
from robot_car.decision.action_dispatcher import ActionDispatcher, ActionId
from robot_car.decision.motion_target import MotionTarget
from robot_car.decision.state_machine import VehicleStateMachine, monotonic_ms
from robot_car.ipc.vision_socket import VisionEventSubscriber
from robot_car.observability.logging import configure_logging
from robot_car.observability.status_led import StatusLedController
from robot_car.roller_control.homing import HomeCancelled, home_from_config_file
from robot_car.vehicle_link.can_transport import CanTransport
from robot_car.vehicle_link.fake_transport import FakeTransport
from robot_car.vehicle_link.gateway import VehicleGateway
from robot_car.vehicle_link.transport_base import Transport
from robot_car.vehicle_link.uart_transport import UartTransport


LOG = logging.getLogger(__name__)


def build_transport(config: Dict[str, Any], override: str | None) -> Transport:
    transport_config = config["transport"]
    kind = override or str(transport_config.get("type", "fake"))
    if kind == "fake":
        return FakeTransport()
    if not transport_config.get("enabled", False):
        raise RuntimeError(f"real {kind} transport requested while transport.enabled is false")
    if kind == "uart":
        return UartTransport(transport_config.get("uart", {}))
    if kind == "can":
        return CanTransport(transport_config.get("can", {}))
    raise ValueError(f"unsupported transport: {kind}")


class VehicleDaemon:
    def __init__(self, config: Dict[str, Any], transport: Transport,
                 roller_control_path: str | Path = "config/roller_control.yaml") -> None:
        self.config = config
        self.transport = transport
        self.gateway = VehicleGateway(transport, config["vehicle"])
        self.state_machine = VehicleStateMachine(config["vehicle"])
        self.control_arbiter = ControlArbiter(config["vehicle"])
        self.action_dispatcher = ActionDispatcher(config["vehicle"])
        self.action_lock = threading.Lock()
        self.roller_control_path = Path(roller_control_path)
        self._roller_home_thread: threading.Thread | None = None
        self._roller_home_cancel = threading.Event()
        self._last_vision_event_ms: int | None = None
        self.status_led = StatusLedController(config["vehicle"].get("status_led", {}))
        self.ui_error = ""
        self._last_action_report = None
        self.subscriber = VisionEventSubscriber(config["runtime"]["vision_socket"])
        self.stop_event = threading.Event()
        self.is_fake = isinstance(transport, FakeTransport)

    def run(self) -> None:
        self.gateway.open()
        self.state_machine.start()
        heartbeat_hz = max(1.0, float(self.config["vehicle"].get("heartbeat_hz", 20)))
        heartbeat_enabled = bool(self.config.get("transport", {}).get("heartbeat", {}).get(
            "enabled", True))
        interval = 1.0 / heartbeat_hz
        next_send = time.monotonic()
        LOG.info("vehicled started; transport=%s control_enabled=%s heartbeat_enabled=%s",
                 type(self.transport).__name__, self.config["vehicle"].get("control_enabled", False),
                 heartbeat_enabled)
        try:
            while not self.stop_event.is_set():
                event = self.subscriber.receive(timeout=min(interval, 0.05))
                now_ms = monotonic_ms()
                if self.stop_event.is_set():
                    break
                if event is not None:
                    self._last_vision_event_ms = now_ms
                    self.state_machine.handle_event(event, now_ms)
                    self.control_arbiter.handle_event(event, now_ms)
                    self.action_dispatcher.handle_event(event, now_ms)
                while True:
                    request = self.gateway.receive_action_request()
                    if request is None:
                        break
                    try:
                        self.request_action(request["action_id"], request.get("parameters"),
                                            source="uart", now_ms=now_ms)
                    except (KeyError, TypeError, ValueError, RuntimeError) as error:
                        LOG.warning("rejected action request: %s", error)
                with self.action_lock:
                    self.action_dispatcher.update(now_ms)
                try:
                    self.gateway.poll(0.0)
                except Exception:
                    LOG.exception("vehicle receive error")
                telemetry = self.gateway.telemetry.snapshot()["data"]
                with self.action_lock:
                    self.action_dispatcher.handle_telemetry(telemetry, now_ms)
                    action_snapshot = self.action_dispatcher.snapshot(now_ms).to_dict()
                action_token = (action_snapshot["action_id"], action_snapshot["status"],
                                action_snapshot["phase"], action_snapshot["reason"])
                if action_token != self._last_action_report:
                    try:
                        # Keep the wire event below the protocol's 255-byte
                        # payload limit.  Timestamps and elapsed time remain
                        # available in the local TUI and are not needed by M0.
                        wire_action_status = {
                            key: action_snapshot[key] for key in (
                                "action_id", "status", "source", "phase", "target_mm",
                                "last_remote_action_id", "ball_error_mm", "reason")
                        }
                        self.gateway.send_event("ACTION_STATUS", wire_action_status, 1000,
                                                expect_ack=False)
                        self._last_action_report = action_token
                    except Exception:
                        LOG.exception("failed to report action status")
                capture = self.config["vehicle"].get("capture", {})
                balance = self.config["vehicle"].get("balance", {})
                # Output-only capture tests send coordinates to an analyzer with motors
                # disabled. They cannot require an MSPM0 reply, unlike real motion.
                allow_unseen = (self.is_fake
                                 or (bool(capture.get("enabled", False))
                                     and bool(capture.get("output_only", False)))
                                 or (bool(balance.get("enabled", False))
                                     and (bool(balance.get("output_only", False))
                                          or not bool(balance.get("require_feedback", True)))))
                link_ok = self.gateway.watchdog.healthy(now_ms, allow_unseen=allow_unseen)
                self.state_machine.update_safety(link_ok, bool(telemetry.get("estop", False)),
                                                 str(telemetry.get("fault", "")), now_ms)
                self.status_led.update({
                    "action": action_snapshot,
                    "gateway": self.gateway.stats,
                    "ui_error": self.ui_error,
                    "recent_vision": (self._last_vision_event_ms is not None
                                      and now_ms - self._last_vision_event_ms
                                      <= int(self.config["vehicle"].get("vision_timeout_ms", 500))),
                    "control_enabled": bool(self.config["vehicle"].get("control_enabled", False)),
                    "balance_enabled": bool(self.config["vehicle"].get("balance", {}).get("enabled", False)),
                    "fault": (self.state_machine.state.value in {"FAILSAFE", "E_STOP", "FAULT"}
                              or bool(telemetry.get("estop", False))
                              or bool(telemetry.get("fault", ""))),
                })
                now = time.monotonic()
                if now >= next_send:
                    if heartbeat_enabled:
                        self.gateway.send_heartbeat()
                    fallback = self.state_machine.target(now_ms)
                    with self.action_lock:
                        mode = self.action_dispatcher.motion_mode()
                        action_target_mm = self.action_dispatcher.target_mm
                    if self._roller_home_active():
                        fallback = MotionTarget(enabled=False, valid_for_ms=int(
                            self.config["vehicle"].get("default_valid_for_ms", 200)))
                    elif mode is not None and self.state_machine.state.value not in {
                            "FAILSAFE", "E_STOP", "FAULT"}:
                        fallback = MotionTarget(mode=mode,
                                                enabled=bool(self.config["vehicle"].get(
                                                    "control_enabled", False)),
                                                valid_for_ms=int(self.config["vehicle"].get(
                                                    "default_valid_for_ms", 200)))
                    self.control_arbiter.set_balance_target(
                        action_target_mm or 0.0)
                    motion = self.control_arbiter.select(now_ms, fallback)
                    self.gateway.send_motion(motion)
                    next_send = now + interval
        finally:
            self.status_led.close()
            self.subscriber.close()
            self.gateway.close()
            LOG.info("vehicled stopped; state=%s stats=%s", self.state_machine.state.value,
                     self.gateway.stats)

    def request_action(self, action_id: int, parameters: Dict[str, Any] | None = None,
                       *, source: str = "tui", now_ms: int | None = None) -> None:
        if int(action_id) == int(ActionId.STOP):
            self._cancel_roller_home()
        if int(action_id) == int(ActionId.ROLLER_HOME):
            self._request_roller_home(parameters, source=source, now_ms=now_ms)
            return
        with self.action_lock:
            self.action_dispatcher.request(
                action_id, parameters, source=source, now_ms=now_ms,
                current_ball_error_mm=self.action_dispatcher.ball_error_mm)

    def _request_roller_home(self, parameters: Dict[str, Any] | None, *, source: str,
                             now_ms: int | None) -> None:
        home_config = self.config["vehicle"].get("roller_home", {})
        if not bool(home_config.get("enabled", False)):
            raise RuntimeError("vehicle.roller_home.enabled is false")
        parameters = dict(parameters or {})
        parameters.setdefault("timeout_ms", int(home_config.get("timeout_ms", 30000)))
        with self.action_lock:
            if self._roller_home_thread is not None and self._roller_home_thread.is_alive():
                raise RuntimeError("roller motor home is already in progress")
            self.action_dispatcher.request(ActionId.ROLLER_HOME, parameters, source=source,
                                           now_ms=now_ms,
                                           current_ball_error_mm=self.action_dispatcher.ball_error_mm)
            self._roller_home_cancel.clear()
            self._roller_home_thread = threading.Thread(
                target=self._run_roller_home, args=(int(parameters["timeout_ms"]),),
                name="roller-home", daemon=True)
            self._roller_home_thread.start()

    def _run_roller_home(self, timeout_ms: int) -> None:
        try:
            home_from_config_file(self.roller_control_path, timeout_ms,
                                  cancel_event=self._roller_home_cancel)
        except HomeCancelled:
            LOG.info("roller motor home cancelled")
        except Exception as error:
            LOG.exception("roller motor home failed")
            self.set_ui_error(str(error))
            with self.action_lock:
                if self.action_dispatcher.active_action == ActionId.ROLLER_HOME:
                    self.action_dispatcher.fail("motor_home_failed")
        else:
            with self.action_lock:
                if self.action_dispatcher.active_action == ActionId.ROLLER_HOME:
                    self.action_dispatcher.complete("motor_home_complete")

    def _cancel_roller_home(self) -> None:
        with self.action_lock:
            active = (self.action_dispatcher.active_action == ActionId.ROLLER_HOME
                      and self.action_dispatcher.status == "RUNNING")
            if active:
                self._roller_home_cancel.set()

    def _roller_home_active(self) -> bool:
        with self.action_lock:
            return (self.action_dispatcher.active_action == ActionId.ROLLER_HOME
                    and self.action_dispatcher.status == "RUNNING")

    def set_ui_error(self, value: str) -> None:
        self.ui_error = value

    def ui_snapshot(self) -> Dict[str, Any]:
        with self.action_lock:
            action = self.action_dispatcher.snapshot().to_dict()
        return {"action": action, "gateway": dict(self.gateway.stats), "ui_error": self.ui_error}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="RDK robot-car vehicle daemon")
    parser.add_argument("--config-dir", default="config")
    parser.add_argument("--transport", choices=("fake", "uart", "can"), default=None)
    parser.add_argument("--tui", action="store_true", help="run the curses action selector")
    parser.add_argument("--quiet", action="store_true", help="suppress informational console logging")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        config = load_config(args.config_dir)
        paths = ensure_runtime_dirs(config)
        configure_logging("vehicled", "WARNING" if args.quiet else
                          config["runtime"].get("log_level", "INFO"), paths["logs"],
                          console=not args.tui)
        control_path = Path(args.config_dir) / "roller_control.yaml"
        daemon = VehicleDaemon(config, build_transport(config, args.transport), control_path)
    except Exception as error:
        logging.basicConfig(level=logging.INFO)
        LOG.error("safe startup failure: %s", error)
        return 2
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda _signum, _frame: daemon.stop_event.set())
    if args.tui:
        from robot_car.vehicle_tui import run_vehicle_tui
        try:
            import yaml
            control_path = Path(args.config_dir) / "roller_control.yaml"
            with control_path.open("r", encoding="utf-8") as stream:
                roller_config = yaml.safe_load(stream) or {}
        except (OSError, ValueError):
            roller_config = {}
        worker = threading.Thread(target=daemon.run, name="vehicled-control", daemon=True)
        worker.start()
        try:
            run_vehicle_tui(daemon, roller_config)
        finally:
            daemon.stop_event.set()
            worker.join(timeout=3.0)
    else:
        daemon.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
