"""Bounded, failure-isolating visual plugin scheduler."""

from __future__ import annotations

import logging
import time
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass
from typing import Dict, List, Optional, Set, Tuple

from robot_car.camera.frame import CameraFrame

from .events import VisionEvent
from .plugin import VisionPlugin


LOG = logging.getLogger(__name__)


@dataclass
class PluginSlot:
    plugin: VisionPlugin
    interval_ms: int
    max_processing_ms: int
    priority: int
    next_run_ms: int = 0
    failures: int = 0
    future: Optional[Future] = None
    started_ms: int = 0


class PluginScheduler:
    def __init__(self, plugins: List[VisionPlugin]) -> None:
        ordered = sorted(plugins, key=lambda item: int(item.config.get("priority", 0)), reverse=True)
        self.slots = [PluginSlot(item, max(0, int(item.config.get("interval_ms", 100))),
                     max(0, int(item.config.get("max_processing_ms", 100))),
                     int(item.config.get("priority", 0))) for item in ordered]
        self.executor = ThreadPoolExecutor(max_workers=max(1, len(ordered)), thread_name_prefix="vision-plugin")

    def initialize(self) -> None:
        for slot in self.slots:
            if not slot.plugin.enabled:
                continue
            try:
                slot.plugin.initialize()
            except Exception as error:
                slot.failures += 1
                slot.plugin.enabled = False
                if hasattr(slot.plugin, "error"):
                    slot.plugin.error = str(error)
                LOG.error("plugin %s disabled after initialization error: %s", slot.plugin.name, error)

    def process_latest(self, frame: CameraFrame, now_ms: int) -> Tuple[List[VisionEvent], Set[str]]:
        results: List[VisionEvent] = []
        completed_sources: Set[str] = set()
        for slot in self.slots:
            if not slot.plugin.enabled:
                continue
            if slot.future is not None:
                if slot.future.done():
                    completed_sources.add(slot.plugin.name)
                    try:
                        results.extend(slot.future.result())
                        slot.failures = 0
                    except Exception as error:
                        slot.failures += 1
                        delay = min(30_000, 500 * (2 ** min(slot.failures, 6)))
                        slot.next_run_ms = now_ms + delay
                        if hasattr(slot.plugin, "error"):
                            slot.plugin.error = str(error)
                        LOG.exception("plugin %s failed; retry in %d ms", slot.plugin.name, delay)
                    slot.future = None
                elif (slot.max_processing_ms > 0
                      and now_ms - slot.started_ms > slot.max_processing_ms):
                    slot.failures += 1
                    slot.next_run_ms = now_ms + min(30_000, 500 * (2 ** min(slot.failures, 6)))
                    LOG.warning("plugin %s exceeded %d ms; skipping new frames", slot.plugin.name,
                                slot.max_processing_ms)
                    continue
            if now_ms >= slot.next_run_ms:
                slot.started_ms = now_ms
                slot.next_run_ms = now_ms + slot.interval_ms
                slot.future = self.executor.submit(slot.plugin.process, frame)
        return results, completed_sources

    def health(self) -> Dict[str, object]:
        return {slot.plugin.name: {**slot.plugin.health(), "failures": slot.failures,
                "busy": slot.future is not None and not slot.future.done()} for slot in self.slots}

    def close(self) -> None:
        self.executor.shutdown(wait=False, cancel_futures=True)
        for slot in self.slots:
            slot.plugin.close()
