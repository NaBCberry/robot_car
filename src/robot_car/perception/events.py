"""Model-independent visual event schema."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any, Dict


@dataclass(frozen=True)
class VisionEvent:
    timestamp_monotonic_ms: int
    source: str
    event_type: str
    confidence: float
    payload: Dict[str, Any]
    frame_id: int
    ttl_ms: int = 150
    image_width: int = 0
    image_height: int = 0
    confirmed: bool = True

    def is_expired(self, now_ms: int) -> bool:
        return now_ms - self.timestamp_monotonic_ms > self.ttl_ms

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, value: Dict[str, Any]) -> "VisionEvent":
        required = {"timestamp_monotonic_ms", "source", "event_type", "confidence", "payload",
                    "frame_id", "ttl_ms", "image_width", "image_height"}
        missing = required - value.keys()
        if missing:
            raise ValueError(f"VisionEvent missing fields: {sorted(missing)}")
        if not isinstance(value["payload"], dict):
            raise ValueError("VisionEvent.payload must be an object")
        return cls(**{key: value[key] for key in cls.__dataclass_fields__ if key in value})

    def key(self) -> str:
        stable_id = self.payload.get("stable_id", self.payload.get("class_id", self.payload.get("command", "")))
        return f"{self.source}:{self.event_type}:{stable_id}"
