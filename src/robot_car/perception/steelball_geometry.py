"""Calibrated image-to-electromagnet capture-point geometry."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Sequence


@dataclass(frozen=True)
class CaptureCoordinates:
    """Steel-ball ground position relative to the electromagnet capture point."""

    forward_mm: int
    lateral_mm: int
    bearing_mdeg: int
    range_mm: int


class ImageToCaptureProjector:
    """Maps a ground-plane image point into the calibrated capture-point frame.

    The configured 3x3 homography maps image pixel (u, v, 1) to a ground point
    (forward_mm, lateral_mm, 1). Positive lateral direction must be documented
    during calibration and agreed with MSPM0 firmware.
    """

    def __init__(self, homography: Sequence[float]) -> None:
        if len(homography) != 9:
            raise ValueError("image_to_capture_homography must contain 9 values")
        self._h = tuple(float(value) for value in homography)
        if not all(math.isfinite(value) for value in self._h):
            raise ValueError("image_to_capture_homography must be finite")

    def project(self, pixel_x: float, pixel_y: float) -> CaptureCoordinates:
        h = self._h
        denominator = h[6] * pixel_x + h[7] * pixel_y + h[8]
        if abs(denominator) < 1e-9:
            raise ValueError("image point projects to infinity")
        forward = (h[0] * pixel_x + h[1] * pixel_y + h[2]) / denominator
        lateral = (h[3] * pixel_x + h[4] * pixel_y + h[5]) / denominator
        if forward < 0:
            raise ValueError("target projects behind the capture point")
        bearing = math.degrees(math.atan2(lateral, forward)) * 1000.0
        distance = math.hypot(forward, lateral)
        return CaptureCoordinates(int(round(forward)), int(round(lateral)),
                                  int(round(bearing)), int(round(distance)))
