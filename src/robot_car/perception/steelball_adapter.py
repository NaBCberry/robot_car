"""Steel-ball segmentation adapter reusing the existing YOLO26Seg runtime."""

from __future__ import annotations

import importlib
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import yaml

from robot_car.camera.frame import CameraFrame

from .events import VisionEvent
from .plugin import VisionPlugin
from .steelball_geometry import ImageToCaptureProjector
from .yolo_adapter import RUNTIME_DIR


class SteelballAdapter(VisionPlugin):
    """Detect the lowest steel ball and emit a calibrated target when available."""

    def __init__(self, name: str, config: Dict[str, Any]) -> None:
        super().__init__(name, config)
        self.model = None
        self.error = ""
        self.last_inference_ms: Optional[float] = None
        self.projector: Optional[ImageToCaptureProjector] = None

    def initialize(self) -> None:
        options = self.config.get("config", {})
        model_path = Path(options.get("model_path", ""))
        if not str(model_path) or not model_path.is_file():
            raise FileNotFoundError(f"steel-ball segmentation model is missing: {model_path}")
        calibration = self._load_calibration(options)
        if calibration:
            self.projector = ImageToCaptureProjector(calibration)
        if str(RUNTIME_DIR) not in sys.path:
            sys.path.insert(0, str(RUNTIME_DIR))
        module = importlib.import_module("yolo26_seg")
        model_config = module.YOLO26SegConfig(
            model_path=str(model_path),
            classes_num=int(options.get("classes_num", 1)),
            score_thres=float(options.get("score_threshold", 0.25)),
            nms_thres=float(options.get("nms_threshold", 0.65)),
        )
        self.model = module.YOLO26Seg(model_config)
        self.model.set_scheduling_params(priority=int(self.config.get("priority", 0)),
                                         bpu_cores=self.config.get("bpu_cores"))

    @staticmethod
    def _load_calibration(options: Dict[str, Any]) -> Sequence[float]:
        direct = options.get("image_to_capture_homography", [])
        if direct:
            return direct
        path_value = options.get("calibration_path", "")
        if not path_value:
            return []
        path = Path(path_value)
        if not path.is_file():
            raise FileNotFoundError(f"steel-ball calibration file is missing: {path}")
        with path.open("r", encoding="utf-8") as stream:
            calibration = yaml.safe_load(stream) or {}
        values = calibration.get("image_to_capture_homography", [])
        if not isinstance(values, list):
            raise ValueError("steel-ball calibration homography must be a list")
        return values

    def process(self, frame: CameraFrame) -> List[VisionEvent]:
        if self.model is None:
            return []
        started = time.monotonic()
        boxes, scores, class_ids, _masks = self.model.predict(frame.image)
        self.last_inference_ms = (time.monotonic() - started) * 1000.0
        target = self._select_primary(boxes, scores, class_ids)
        if target is None:
            return []
        return [self._event_from_detection(frame, *target)]

    def _select_primary(self, boxes: Any, scores: Any, class_ids: Any) -> Optional[Tuple[Any, float, int]]:
        target_class = int(self.config.get("config", {}).get("target_class_id", 0))
        candidates = [(box, float(score), int(class_id)) for box, score, class_id in
                      zip(boxes, scores, class_ids) if int(class_id) == target_class]
        # The lowest bbox edge approximates the nearest ground contact point. Confidence
        # only breaks ties so another, higher ball cannot steal the capture target.
        return max(candidates, key=lambda item: (float(item[0][3]), item[1])) if candidates else None

    def _event_from_detection(self, frame: CameraFrame, box: Any, score: float,
                              class_id: int) -> VisionEvent:
        x1, y1, x2, y2 = (float(value) for value in box)
        center_x = (x1 + x2) / 2.0
        center_y = (y1 + y2) / 2.0
        ttl = int(self.config.get("ttl_ms", 150))
        payload: Dict[str, Any] = {
            "stable_id": "primary_ball",
            "track_id": 1,
            "class_id": class_id,
            "center_px": [round(center_x, 2), round(center_y, 2)],
            "bbox_xyxy": [round(value, 2) for value in (x1, y1, x2, y2)],
        }
        event_type = "STEELBALL_DETECTED"
        if self.projector is not None:
            try:
                coordinates = self.projector.project(center_x, y2)
            except ValueError as error:
                payload["projection_error"] = str(error)
            else:
                event_type = "BALL_TARGET"
                payload.update({
                    "forward_mm": coordinates.forward_mm,
                    "lateral_mm": coordinates.lateral_mm,
                    "bearing_mdeg": coordinates.bearing_mdeg,
                    "range_mm": coordinates.range_mm,
                })
        return VisionEvent(frame.timestamp_monotonic_ms, self.name, event_type, score, payload,
                           frame.frame_id, ttl, frame.width, frame.height, False)

    def health(self) -> Dict[str, Any]:
        return {"available": self.model is not None, "enabled": self.enabled,
                "calibrated": self.projector is not None, "error": self.error,
                "last_inference_ms": self.last_inference_ms}

    def close(self) -> None:
        self.model = None
