"""Exclusive high-level vehicle decision and MSPM0 gateway daemon."""

from __future__ import annotations

import argparse
import logging
import signal
import threading
import time
from typing import Any, Dict

from robot_car.config import ensure_runtime_dirs, load_config
from robot_car.decision.control_arbiter import ControlArbiter
from robot_car.decision.state_machine import VehicleStateMachine, monotonic_ms
from robot_car.ipc.vision_socket import VisionEventSubscriber
from robot_car.observability.logging import configure_logging
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
    def __init__(self, config: Dict[str, Any], transport: Transport) -> None:
        self.config = config
        self.transport = transport
        self.gateway = VehicleGateway(transport, config["vehicle"])
        self.state_machine = VehicleStateMachine(config["vehicle"])
        self.control_arbiter = ControlArbiter(config["vehicle"])
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
                if event is not None:
                    self.state_machine.handle_event(event, now_ms)
                    self.control_arbiter.handle_event(event, now_ms)
                try:
                    self.gateway.poll(0.0)
                except Exception:
                    LOG.exception("vehicle receive error")
                telemetry = self.gateway.telemetry.snapshot()["data"]
                link_ok = self.gateway.watchdog.healthy(now_ms, allow_unseen=self.is_fake)
                self.state_machine.update_safety(link_ok, bool(telemetry.get("estop", False)),
                                                 str(telemetry.get("fault", "")), now_ms)
                now = time.monotonic()
                if now >= next_send:
                    if heartbeat_enabled:
                        self.gateway.send_heartbeat()
                    motion = self.control_arbiter.select(now_ms, self.state_machine.target(now_ms))
                    self.gateway.send_motion(motion)
                    next_send = now + interval
        finally:
            self.subscriber.close()
            self.gateway.close()
            LOG.info("vehicled stopped; state=%s stats=%s", self.state_machine.state.value,
                     self.gateway.stats)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="RDK robot-car vehicle daemon")
    parser.add_argument("--config-dir", default="config")
    parser.add_argument("--transport", choices=("fake", "uart", "can"), default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        config = load_config(args.config_dir)
        paths = ensure_runtime_dirs(config)
        configure_logging("vehicled", config["runtime"].get("log_level", "INFO"), paths["logs"])
        daemon = VehicleDaemon(config, build_transport(config, args.transport))
    except Exception as error:
        logging.basicConfig(level=logging.INFO)
        LOG.error("safe startup failure: %s", error)
        return 2
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda _signum, _frame: daemon.stop_event.set())
    daemon.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
