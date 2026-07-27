"""Latest telemetry cache."""

import threading
import time
from typing import Any, Dict, Optional


class TelemetryCache:
    def __init__(self) -> None:
        self._value: Dict[str, Any] = {}
        self._updated_ms: Optional[int] = None
        self._lock = threading.Lock()

    def update(self, value: Dict[str, Any]) -> None:
        with self._lock:
            self._value = dict(value)
            self._updated_ms = time.monotonic_ns() // 1_000_000

    def snapshot(self) -> Dict[str, Any]:
        with self._lock:
            return {"data": dict(self._value), "updated_monotonic_ms": self._updated_ms}
