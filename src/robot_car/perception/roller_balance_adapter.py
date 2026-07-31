"""Top-view steel-ball state for the 25 cm roller-balance tube."""

from __future__ import annotations

import math
from bisect import bisect_left
from typing import Any, Dict, Optional, Tuple

from robot_car.camera.frame import CameraFrame

from .events import VisionEvent
from .steelball_adapter import SteelballAdapter


class RollerBalanceAdapter(SteelballAdapter):
    """Reuse the configured steel-ball model, but emit one-dimensional balance state."""

    def __init__(self, name: str, config: Dict[str, Any]) -> None:
        super().__init__(name, config)
        self.roi: Optional[Tuple[float, float, float, float]] = None
        self.center_x_px = 0.0
        self.mm_per_pixel = 0.0
        self.axis_direction = 1.0
        self.axis_points: Tuple[Tuple[float, float], ...] = ()

    def initialize(self) -> None:
        options = self.config.get("config", {})
        self.roi = self._parse_roi(options.get("roi_xyxy", []))
        try:
            self.center_x_px = float(options["center_x_px"])
            self.mm_per_pixel = float(options["mm_per_pixel"])
            self.axis_direction = float(options.get("axis_direction", 1))
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError("roller-balance calibration is incomplete") from error
        if not math.isfinite(self.center_x_px):
            raise ValueError("roller-balance center must be finite")
        if not math.isfinite(self.mm_per_pixel) or self.mm_per_pixel <= 0:
            raise ValueError("roller-balance mm_per_pixel must be positive")
        if self.axis_direction not in {-1.0, 1.0}:
            raise ValueError("roller-balance axis_direction must be -1 or 1")
        axis_points = options.get("axis_points")
        self.axis_points = self._parse_axis_points(axis_points) if axis_points else ()
        super().initialize()

    @staticmethod
    def _parse_roi(value: Any) -> Tuple[float, float, float, float]:
        if not isinstance(value, (list, tuple)) or len(value) != 4:
            raise ValueError("roller-balance roi_xyxy must contain four pixel coordinates")
        try:
            x1, y1, x2, y2 = (float(item) for item in value)
        except (TypeError, ValueError) as error:
            raise ValueError("roller-balance roi_xyxy must be numeric") from error
        if not all(math.isfinite(item) for item in (x1, y1, x2, y2)) or x2 <= x1 or y2 <= y1:
            raise ValueError("roller-balance roi_xyxy is invalid")
        return x1, y1, x2, y2

    def _parse_axis_points(self, value: Any) -> Tuple[Tuple[float, float], ...]:
        """Return tick mappings ordered by pixel coordinate for interpolation."""
        if not isinstance(value, (list, tuple)) or len(value) < 2:
            raise ValueError("roller-balance axis_points must contain at least two ticks")
        if self.roi is None:
            raise RuntimeError("roller-balance ROI must be initialized before axis_points")
        roi_x1, _, roi_x2, _ = self.roi
        points = []
        for point in value:
            if not isinstance(point, dict):
                raise ValueError("roller-balance axis_points entries must be mappings")
            try:
                pixel_x = float(point["pixel_x"])
                position_mm = float(point["position_mm"])
            except (KeyError, TypeError, ValueError) as error:
                raise ValueError("roller-balance axis_points entries are invalid") from error
            if not math.isfinite(pixel_x) or not math.isfinite(position_mm):
                raise ValueError("roller-balance axis_points must be finite")
            if not roi_x1 <= pixel_x <= roi_x2:
                raise ValueError("roller-balance axis_points must lie within the ROI")
            points.append((pixel_x, position_mm))

        points.sort(key=lambda point: point[0])
        pixel_steps = [points[index + 1][0] - points[index][0] for index in range(len(points) - 1)]
        position_steps = [points[index + 1][1] - points[index][1]
                          for index in range(len(points) - 1)]
        if any(math.isclose(step, 0.0, abs_tol=1e-6) for step in pixel_steps):
            raise ValueError("roller-balance axis_points pixel positions must not repeat")
        if any(math.isclose(step, 0.0, abs_tol=1e-6) for step in position_steps):
            raise ValueError("roller-balance axis_points positions must not repeat")
        if not (all(step > 0 for step in position_steps) or all(step < 0 for step in position_steps)):
            raise ValueError("roller-balance axis_points positions must be ordered along the tube")
        return tuple(points)

    def _position_mm(self, pixel_x: float) -> float:
        """Map a ball center to millimetres using ticks, falling back to a fixed scale."""
        if not self.axis_points:
            return self.axis_direction * (pixel_x - self.center_x_px) * self.mm_per_pixel
        if pixel_x <= self.axis_points[0][0]:
            return self.axis_points[0][1]
        if pixel_x >= self.axis_points[-1][0]:
            return self.axis_points[-1][1]
        upper_index = bisect_left(self.axis_points, (pixel_x, -math.inf))
        lower_pixel, lower_position = self.axis_points[upper_index - 1]
        upper_pixel, upper_position = self.axis_points[upper_index]
        ratio = (pixel_x - lower_pixel) / (upper_pixel - lower_pixel)
        return lower_position + ratio * (upper_position - lower_position)

    def _select_primary(self, boxes: Any, scores: Any, class_ids: Any) -> Optional[Tuple[Any, float, int]]:
        if self.roi is None:
            return None
        target_class = int(self.config.get("config", {}).get("target_class_id", 0))
        x1, y1, x2, y2 = self.roi
        candidates = []
        for box, score, class_id in zip(boxes, scores, class_ids):
            if int(class_id) != target_class:
                continue
            left, top, right, bottom = (float(value) for value in box)
            center_x, center_y = (left + right) / 2.0, (top + bottom) / 2.0
            if x1 <= center_x <= x2 and y1 <= center_y <= y2:
                candidates.append((box, float(score), int(class_id)))
        return max(candidates, key=lambda item: item[1]) if candidates else None

    def _event_from_detection(self, frame: CameraFrame, box: Any, score: float,
                              class_id: int) -> VisionEvent:
        if self.roi is None:
            raise RuntimeError("roller-balance adapter is not initialized")
        x1, y1, x2, y2 = (float(value) for value in box)
        center_x = (x1 + x2) / 2.0
        center_y = (y1 + y2) / 2.0
        error_mm = int(round(self._position_mm(center_x)))
        payload = {
            "stable_id": "roller_balance_ball",
            "class_id": class_id,
            "bbox_xyxy": [round(value, 2) for value in (x1, y1, x2, y2)],
            "center_px": [round(center_x, 2), round(center_y, 2)],
            "roller_roi_xyxy": [round(value, 2) for value in self.roi],
            "center_x_px": round(self.center_x_px, 2),
            "error_mm": error_mm,
        }
        return VisionEvent(frame.timestamp_monotonic_ms, self.name, "BALL_BALANCE_STATE", score,
                           payload, frame.frame_id, int(self.config.get("ttl_ms", 120)),
                           frame.width, frame.height, False)

    def health(self) -> Dict[str, Any]:
        return {**super().health(), "calibrated": self.roi is not None}
