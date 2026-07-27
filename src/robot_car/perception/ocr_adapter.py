"""Adapter around the existing PaddleOCR repository; never owns a camera."""

from __future__ import annotations

import importlib
import sys
import time
from pathlib import Path
from typing import Any, Dict, List

from robot_car.camera.frame import CameraFrame

from .events import VisionEvent
from .plugin import VisionPlugin


PROJECTS_ROOT = Path(__file__).resolve().parents[4]
RUNTIME_DIR = PROJECTS_ROOT / "26TI-PaddleOCR" / "runtime" / "python"


class OcrAdapter(VisionPlugin):
    def __init__(self, name: str, config: Dict[str, Any]) -> None:
        super().__init__(name, config)
        self.model = None
        self.error = ""
        self.last_inference_ms = None

    def initialize(self) -> None:
        options = self.config.get("config", {})
        detection_path = Path(options.get("detection_model_path", ""))
        recognition_path = Path(options.get("recognition_model_path", ""))
        if not detection_path.is_file() or not recognition_path.is_file():
            raise FileNotFoundError("PaddleOCR detection/recognition model paths are missing")
        if str(RUNTIME_DIR) not in sys.path:
            sys.path.insert(0, str(RUNTIME_DIR))
        module = importlib.import_module("paddleocr")
        model_config = module.PaddleOCRConfig(
            det_model_path=str(detection_path), rec_model_path=str(recognition_path),
            det_threshold=float(options.get("detection_threshold", 0.7)),
            det_min_area=int(options.get("minimum_area", 300)),
        )
        self.model = module.PaddleOCR(model_config)
        self.model.set_scheduling_params(priority=int(self.config.get("priority", 0)),
                                         bpu_cores=self.config.get("bpu_cores"))

    def process(self, frame: CameraFrame) -> List[VisionEvent]:
        if self.model is None:
            return []
        started = time.monotonic()
        boxes, texts = self.model.predict(frame.image)
        self.last_inference_ms = (time.monotonic() - started) * 1000.0
        ttl = int(self.config.get("ttl_ms", 150))
        events = []
        for box, text in zip(boxes, texts):
            if not text:
                continue
            events.append(VisionEvent(
                timestamp_monotonic_ms=frame.timestamp_monotonic_ms, frame_id=frame.frame_id,
                source=self.name, event_type="TEXT_RECOGNIZED", confidence=1.0, ttl_ms=ttl,
                payload={"text": text, "polygon": [[int(value) for value in point] for point in box]},
                image_width=frame.width, image_height=frame.height, confirmed=False,
            ))
        return events

    def health(self) -> Dict[str, Any]:
        return {"available": self.model is not None, "enabled": self.enabled,
                "error": self.error, "last_inference_ms": self.last_inference_ms}

    def close(self) -> None:
        self.model = None
