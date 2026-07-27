"""Monotonic link watchdog."""

import time
from typing import Optional


class LinkWatchdog:
    def __init__(self, timeout_ms: int) -> None:
        self.timeout_ms = timeout_ms
        self.last_receive_ms: Optional[int] = None

    def feed(self, now_ms: Optional[int] = None) -> None:
        self.last_receive_ms = now_ms if now_ms is not None else time.monotonic_ns() // 1_000_000

    def healthy(self, now_ms: Optional[int] = None, allow_unseen: bool = False) -> bool:
        if self.last_receive_ms is None:
            return allow_unseen
        now = now_ms if now_ms is not None else time.monotonic_ns() // 1_000_000
        return now - self.last_receive_ms <= self.timeout_ms
