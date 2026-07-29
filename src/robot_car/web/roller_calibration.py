"""Validation and durable storage for the roller-balance web calibration."""

from __future__ import annotations

import math
import os
import tempfile
from pathlib import Path
from typing import Any, Dict

import yaml


def build_roller_calibration(payload: Dict[str, Any]) -> Dict[str, float | int | list[float]]:
    """Build the camera.yaml roller-balance mapping from checked browser input."""
    if not isinstance(payload, dict):
        raise ValueError("calibration request must be an object")
    try:
        roi = [float(value) for value in payload["roi_xyxy"]]
        center_x = float(payload["center_x_px"])
        center_y = float(payload.get("center_y_px", (roi[1] + roi[3]) / 2))
        tube_length_mm = float(payload["tube_length_mm"])
        axis_direction = int(payload.get("axis_direction", 1))
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError("calibration fields are invalid") from error
    if len(roi) != 4 or not all(math.isfinite(value) for value in (*roi, center_x, center_y, tube_length_mm)):
        raise ValueError("calibration values must be finite")
    x1, y1, x2, y2 = roi
    if x2 <= x1 or y2 <= y1:
        raise ValueError("ROI must have positive width and height")
    if not x1 <= center_x <= x2 or not y1 <= center_y <= y2:
        raise ValueError("center point must lie within the ROI")
    if not 20 <= tube_length_mm <= 2_000:
        raise ValueError("tube length must be between 20 and 2000 mm")
    if axis_direction not in {-1, 1}:
        raise ValueError("axis_direction must be -1 or 1")
    return {
        "roi_xyxy": [round(value, 2) for value in roi],
        "center_x_px": round(center_x, 2),
        "mm_per_pixel": round(tube_length_mm / (x2 - x1), 6),
        "axis_direction": axis_direction,
    }


def save_roller_calibration(camera_config_path: str | Path,
                            payload: Dict[str, Any]) -> Dict[str, float | int | list[float]]:
    """Atomically persist the roller calibration in the selected camera YAML file."""
    calibration = build_roller_calibration(payload)
    path = Path(camera_config_path)
    if not path.is_file():
        raise ValueError(f"camera configuration does not exist: {path}")
    with path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}
    if not isinstance(document, dict) or not isinstance(document.get("camera"), dict):
        raise ValueError("camera configuration root is invalid")
    camera_calibration = document["camera"].setdefault("calibration", {})
    if not isinstance(camera_calibration, dict):
        raise ValueError("camera.calibration must be a mapping")
    camera_calibration["roller_balance"] = calibration

    fd, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            yaml.safe_dump(document, stream, allow_unicode=True, sort_keys=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, path)
    except Exception:
        Path(temporary_name).unlink(missing_ok=True)
        raise
    return calibration
