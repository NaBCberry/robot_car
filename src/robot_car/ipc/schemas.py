"""IPC envelope validation."""

from typing import Any, Dict

from robot_car.perception.events import VisionEvent


SCHEMA_VERSION = 1


def event_envelope(event: VisionEvent) -> Dict[str, Any]:
    return {"schema_version": SCHEMA_VERSION, "kind": "vision_event", "event": event.to_dict()}


def parse_envelope(value: Dict[str, Any]) -> VisionEvent:
    if value.get("schema_version") != SCHEMA_VERSION or value.get("kind") != "vision_event":
        raise ValueError("unsupported IPC envelope")
    return VisionEvent.from_dict(value.get("event", {}))
