"""Shared camera frame envelope."""

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class CameraFrame:
    frame_id: int
    timestamp_monotonic_ms: int
    image: Any
    width: int
    height: int
