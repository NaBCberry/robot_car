"""Optional frame overlay hook kept independent from control logic."""

from typing import Any, Iterable

from robot_car.perception.events import VisionEvent


def draw_overlay(image: Any, events: Iterable[VisionEvent]) -> Any:
    """Return the input until a deployment-specific renderer is selected."""
    del events
    return image
