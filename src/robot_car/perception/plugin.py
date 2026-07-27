"""Uniform visual plugin contract."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any, Dict, List

from robot_car.camera.frame import CameraFrame

from .events import VisionEvent


class VisionPlugin(ABC):
    def __init__(self, name: str, config: Dict[str, Any]) -> None:
        self.name = name
        self.config = config
        self.enabled = bool(config.get("enabled", False))

    @abstractmethod
    def initialize(self) -> None:
        pass

    @abstractmethod
    def process(self, frame: CameraFrame) -> List[VisionEvent]:
        pass

    @abstractmethod
    def health(self) -> Dict[str, Any]:
        pass

    @abstractmethod
    def close(self) -> None:
        pass
