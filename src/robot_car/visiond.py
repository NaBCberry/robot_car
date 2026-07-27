"""Single-camera visual perception daemon."""

from __future__ import annotations

import argparse
import logging
import signal
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

from robot_car.camera.capture import CameraCapture, monotonic_ms
from robot_car.camera.frame import CameraFrame
from robot_car.config import ensure_runtime_dirs, load_config
from robot_car.ipc.vision_socket import VisionEventPublisher
from robot_car.observability.logging import configure_logging
from robot_car.observability.metrics import Metrics
from robot_car.perception.events import VisionEvent
from robot_car.perception.plugin_registry import create_plugin
from robot_car.perception.scheduler import PluginScheduler
from robot_car.perception.stabilizer import EventStabilizer
from robot_car.web.server import DebugServer


LOG = logging.getLogger(__name__)


class VisionDaemon:
    def __init__(self, config: Dict[str, Any], simulate: bool = False) -> None:
        self.config = config
        self.simulate = simulate
        self.stop_event = threading.Event()
        self.camera = CameraCapture(config["camera"])
        plugins = [create_plugin(item) for item in config["vision"].get("plugins", [])]
        default_ttl = int(config["vision"].get("default_ttl_ms", 150))
        for plugin in plugins:
            plugin.config.setdefault("ttl_ms", default_ttl)
        self.scheduler = PluginScheduler(plugins)
        self.stabilizer = EventStabilizer(int(config["vision"].get("confirmation_frames", 3)))
        self.publisher = VisionEventPublisher(config["runtime"]["vision_socket"])
        self.metrics = Metrics()
        self.latest_events: List[Dict[str, Any]] = []
        self.started_ms = monotonic_ms()
        self.last_frame_ms: Optional[int] = None
        self.next_health_ms = 0
        self.web: Optional[DebugServer] = None

    def start(self) -> None:
        self.publisher.start()
        self.scheduler.initialize()
        self.camera.start()
        web_config = self.config.get("web", {})
        if web_config.get("enabled", False):
            self.web = DebugServer(str(web_config.get("host", "127.0.0.1")),
                                   int(web_config.get("port", 8090)),
                                   self.status, self.results, self.metrics.snapshot)
            self.web.start()
        LOG.info("visiond started; simulate=%s socket=%s", self.simulate, self.publisher.path)

    def run(self) -> None:
        self.start()
        simulated_frame_id = 0
        next_simulation_ms = 0
        try:
            while not self.stop_event.is_set():
                now = monotonic_ms()
                if self.simulate:
                    if now >= next_simulation_ms:
                        simulated_frame_id += 1
                        event = VisionEvent(now, "simulator", "SLOW_DOWN", 1.0,
                                            {"stable_id": "demo"}, simulated_frame_id,
                                            int(self.config["vision"].get("default_ttl_ms", 150)), 640, 480, False)
                        self._publish(self.stabilizer.update([event], now, {"simulator"}))
                        next_simulation_ms = now + 40
                    self.stop_event.wait(0.01)
                    continue
                frame = self.camera.latest(timeout=0.1)
                if frame is not None:
                    self.last_frame_ms = now
                    self.metrics.increment("frames_received")
                    events, observed_sources = self.scheduler.process_latest(frame, now)
                    self._publish(self.stabilizer.update(events, now, observed_sources))
                health_interval = int(self.config["vision"].get("health_interval_ms", 100))
                if (self.camera.enabled and not self.camera.error and self.last_frame_ms is not None
                        and now - self.last_frame_ms <= int(self.config["vehicle"].get("vision_timeout_ms", 500))
                        and now >= self.next_health_ms):
                    health = VisionEvent(now, "visiond", "VISION_HEALTH", 1.0,
                                         {"camera_ok": True}, frame.frame_id if frame else 0,
                                         health_interval * 2, frame.width if frame else 0,
                                         frame.height if frame else 0, True)
                    self._publish([health])
                    self.next_health_ms = now + health_interval
        finally:
            self.close()

    def _publish(self, events: List[VisionEvent]) -> None:
        if not events:
            return
        self.latest_events = [event.to_dict() for event in events]
        for event in events:
            self.publisher.publish(event)
            self.metrics.increment("events_published")

    def status(self) -> Dict[str, Any]:
        return {"service": "visiond", "healthy": not bool(self.camera.error),
                "uptime_ms": monotonic_ms() - self.started_ms, "simulate": self.simulate,
                "camera_enabled": self.camera.enabled, "camera_error": self.camera.error,
                "ipc_clients": self.publisher.client_count, "plugins": self.scheduler.health()}

    def results(self) -> Dict[str, Any]:
        return {"events": self.latest_events, "timestamp_monotonic_ms": monotonic_ms()}

    def close(self) -> None:
        self.stop_event.set()
        if self.web:
            self.web.close()
            self.web = None
        self.camera.close()
        self.scheduler.close()
        self.publisher.close()
        LOG.info("visiond stopped")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="RDK robot-car vision daemon")
    parser.add_argument("--config-dir", default="config")
    parser.add_argument("--simulate", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        config = load_config(args.config_dir)
        paths = ensure_runtime_dirs(config)
        configure_logging("visiond", config["runtime"].get("log_level", "INFO"), paths["logs"])
        daemon = VisionDaemon(config, args.simulate)
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
