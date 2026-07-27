"""Declarative creation of visual plugins."""

from typing import Any, Callable, Dict

from .ocr_adapter import OcrAdapter
from .placeholders import PlaceholderPlugin
from .plugin import VisionPlugin
from .yolo_adapter import YoloAdapter


Factory = Callable[[str, Dict[str, Any]], VisionPlugin]
_REGISTRY: Dict[str, Factory] = {
    "yolo_detector": YoloAdapter,
    "paddle_ocr": OcrAdapter,
    "qr_code": PlaceholderPlugin,
    "color_marker": PlaceholderPlugin,
    "lane_vision": PlaceholderPlugin,
    "segmentation": PlaceholderPlugin,
    "pose_estimation": PlaceholderPlugin,
}


def register_plugin(plugin_type: str, factory: Factory) -> None:
    if not plugin_type:
        raise ValueError("plugin type cannot be empty")
    _REGISTRY[plugin_type] = factory


def create_plugin(config: Dict[str, Any]) -> VisionPlugin:
    plugin_type = str(config.get("type", ""))
    try:
        factory = _REGISTRY[plugin_type]
    except KeyError as error:
        raise ValueError(f"unknown vision plugin type: {plugin_type}") from error
    return factory(str(config.get("name", plugin_type)), config)
