"""Disabled extension points for future visual capabilities."""

from typing import Any, Dict, List

from robot_car.camera.frame import CameraFrame

from .events import VisionEvent
from .plugin import VisionPlugin


class PlaceholderPlugin(VisionPlugin):
    def __init__(self, name: str, config: Dict[str, Any]) -> None:
        super().__init__(name, config)
        self._error = "placeholder plugin has no implementation"

    def initialize(self) -> None:
        if self.enabled:
            raise RuntimeError(self._error)

    def process(self, frame: CameraFrame) -> List[VisionEvent]:
        return []

    def health(self) -> Dict[str, Any]:
        return {"available": False, "enabled": self.enabled, "error": self._error, "last_inference_ms": None}

    def close(self) -> None:
        pass
