"""Latest-frame camera source; this is the only module that opens a camera."""

from __future__ import annotations

import logging
import queue
import threading
import time
from pathlib import Path
from typing import Any, Dict, Optional

from .frame import CameraFrame


LOG = logging.getLogger(__name__)


class CameraCapture:
    def __init__(self, config: Dict[str, Any]) -> None:
        self.config = config
        self.frames: "queue.Queue[CameraFrame]" = queue.Queue(maxsize=1)
        self._capture = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self.error = ""
        self.frame_count = 0

    @property
    def enabled(self) -> bool:
        return bool(self.config.get("enabled", False))

    def start(self) -> None:
        if not self.enabled:
            test_image = self.config.get("test_image", "")
            if test_image:
                self._load_test_image(Path(test_image))
            LOG.info("camera disabled; no hardware device opened")
            return
        device = self.config.get("device", "")
        if device == "":
            raise ValueError("camera.enabled is true but camera.device is empty")
        try:
            import cv2
        except ImportError as error:
            raise RuntimeError("OpenCV is required when camera is enabled") from error
        resolved_device = int(device) if isinstance(device, str) and device.isdigit() else device
        self._capture = cv2.VideoCapture(resolved_device)
        if not self._capture.isOpened():
            self._capture.release()
            self._capture = None
            raise RuntimeError(f"failed to open configured camera: {device}")
        pixel_format = str(self.config.get("pixel_format", "MJPG"))
        if len(pixel_format) == 4:
            self._capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*pixel_format))
        self._capture.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.config.get("width", 1280)))
        self._capture.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.config.get("height", 720)))
        self._capture.set(cv2.CAP_PROP_FPS, int(self.config.get("fps", 30)))
        actual_format = int(self._capture.get(cv2.CAP_PROP_FOURCC)).to_bytes(4, "little").decode(
            "ascii", errors="replace")
        LOG.info("camera negotiated device=%s format=%s size=%dx%d fps=%.1f",
                 device, actual_format, int(self._capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
                 int(self._capture.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                 self._capture.get(cv2.CAP_PROP_FPS))
        self._thread = threading.Thread(target=self._run, name="camera-capture", daemon=True)
        self._thread.start()

    def _load_test_image(self, path: Path) -> None:
        if not path.is_file():
            raise FileNotFoundError(f"test image does not exist: {path}")
        import cv2
        image = cv2.imread(str(path))
        if image is None:
            raise RuntimeError(f"failed to read test image: {path}")
        height, width = image.shape[:2]
        self._publish(CameraFrame(0, monotonic_ms(), image, width, height))

    def _run(self) -> None:
        while not self._stop.is_set():
            ok, image = self._capture.read()
            if not ok:
                self.error = "camera read failed"
                LOG.error(self.error)
                break
            self.frame_count += 1
            height, width = image.shape[:2]
            self._publish(CameraFrame(self.frame_count, monotonic_ms(), image, width, height))

    def _publish(self, frame: CameraFrame) -> None:
        try:
            self.frames.put_nowait(frame)
        except queue.Full:
            try:
                self.frames.get_nowait()
            except queue.Empty:
                pass
            self.frames.put_nowait(frame)

    def latest(self, timeout: float = 0.1) -> Optional[CameraFrame]:
        try:
            return self.frames.get(timeout=timeout)
        except queue.Empty:
            return None

    def close(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=1.0)
        if self._capture is not None:
            self._capture.release()
            self._capture = None


def monotonic_ms() -> int:
    return time.monotonic_ns() // 1_000_000
