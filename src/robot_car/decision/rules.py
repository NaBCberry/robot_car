"""Mapping from normalized vision events to high-level actions."""

from typing import Optional

from robot_car.perception.events import VisionEvent


SAFETY_EVENTS = {"STOP", "E_STOP"}


def action_for(event: VisionEvent) -> Optional[str]:
    event_type = event.event_type.upper()
    if event_type in {"STOP", "STOP_SIGN"}:
        return "STOP"
    if event_type in {"SLOW_DOWN", "SPEED_LIMIT"}:
        return "SLOW_DOWN"
    if event_type == "INTERSECTION":
        return "INTERSECTION"
    if event_type == "OCR_COMMAND":
        command = str(event.payload.get("command", "")).upper()
        return command if command in {"STOP", "SLOW_DOWN"} else None
    return None
