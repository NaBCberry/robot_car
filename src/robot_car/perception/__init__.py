"""Pluggable perception framework."""

from .events import VisionEvent
from .plugin import VisionPlugin

__all__ = ["VisionEvent", "VisionPlugin"]
