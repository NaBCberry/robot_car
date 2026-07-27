"""In-memory transport for development and tests."""

import queue
import threading
import time
from typing import List, Optional

from .transport_base import Transport


class FakeTransport(Transport):
    def __init__(self) -> None:
        self.sent: List[bytes] = []
        self._incoming: "queue.Queue[bytes]" = queue.Queue()
        self._lock = threading.Lock()
        self.is_open = False

    def open(self) -> None:
        self.is_open = True

    def send(self, data: bytes) -> None:
        if not self.is_open:
            raise RuntimeError("transport is closed")
        with self._lock:
            self.sent.append(bytes(data))

    def receive(self, timeout: float = 0.0) -> Optional[bytes]:
        try:
            return self._incoming.get(timeout=timeout)
        except queue.Empty:
            return None

    def inject(self, data: bytes) -> None:
        self._incoming.put(bytes(data))

    def simulate_timeout(self, seconds: float = 0.01) -> None:
        time.sleep(seconds)

    def close(self) -> None:
        self.is_open = False
