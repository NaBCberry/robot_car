"""Small thread-safe metric registry."""

import threading
from typing import Dict


class Metrics:
    def __init__(self) -> None:
        self._values: Dict[str, float] = {}
        self._lock = threading.Lock()

    def increment(self, name: str, amount: float = 1.0) -> None:
        with self._lock:
            self._values[name] = self._values.get(name, 0.0) + amount

    def set(self, name: str, value: float) -> None:
        with self._lock:
            self._values[name] = value

    def snapshot(self) -> Dict[str, float]:
        with self._lock:
            return dict(self._values)
