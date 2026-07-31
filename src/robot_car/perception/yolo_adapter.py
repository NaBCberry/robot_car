"""Adapter around the existing YOLO repository; never owns a camera."""

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
RUNTIME_DIR = PROJECTS_ROOT / "ultralytics_yolo26" / "runtime" / "python"


class YoloAdapter(VisionPlugin):
    def __init__(self, name: str, config: Dict[str, Any]) -> None:
        super().__init__(name, config)
        self.model = None
        self.error = ""
        self.last_inference_ms = None

    def initialize(self) -> None:
        model_path = Path(self.config.get("config", {}).get("model_path", ""))
        if not str(model_path) or not model_path.is_file():
            raise FileNotFoundError(f"YOLO model is not configured or missing: {model_path}")
        # yolo26_*.py imports shared helpers as ``utils.py_utils``.  Those
        # helpers live beside the repositories under the projects root; add
        # both absolute roots so imports do not depend on the launch cwd.
        for import_root in (PROJECTS_ROOT, RUNTIME_DIR):
            if str(import_root) not in sys.path:
                sys.path.insert(0, str(import_root))
        module = importlib.import_module("yolo26_det")
        options = self.config.get("config", {})
        model_config = module.YOLO26Config(
            model_path=str(model_path), classes_num=int(options.get("classes_num", 80)),
            score_thres=float(options.get("score_threshold", 0.25)),
            nms_thres=float(options.get("nms_threshold", 0.65)),
        )
        self.model = module.YOLO26Detect(model_config)
        self.model.set_scheduling_params(priority=int(self.config.get("priority", 0)),
                                         bpu_cores=self.config.get("bpu_cores"))

    def process(self, frame: CameraFrame) -> List[VisionEvent]:
        if self.model is None:
            return []
        started = time.monotonic()
        boxes, scores, class_ids = self.model.predict(frame.image)
        self.last_inference_ms = (time.monotonic() - started) * 1000.0
        names = self.config.get("config", {}).get("class_names", [])
        ttl = int(self.config.get("ttl_ms", 150))
        events = []
        for box, score, class_id in zip(boxes, scores, class_ids):
            class_number = int(class_id)
            class_name = names[class_number] if class_number < len(names) else str(class_number)
            events.append(VisionEvent(
                timestamp_monotonic_ms=frame.timestamp_monotonic_ms, frame_id=frame.frame_id,
                source=self.name, event_type="OBJECT_DETECTED", confidence=float(score), ttl_ms=ttl,
                payload={"class_id": class_number, "class_name": class_name,
                         "bbox_xyxy": [float(item) for item in box]},
                image_width=frame.width, image_height=frame.height, confirmed=False,
            ))
        return events

    def health(self) -> Dict[str, Any]:
        return {"available": self.model is not None, "enabled": self.enabled,
                "error": self.error, "last_inference_ms": self.last_inference_ms}

    def close(self) -> None:
        self.model = None
