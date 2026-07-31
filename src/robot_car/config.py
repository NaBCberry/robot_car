"""YAML configuration loading with fail-safe defaults."""

from __future__ import annotations

import copy
import math
from pathlib import Path
from typing import Any, Dict

import yaml


SAFE_DEFAULTS: Dict[str, Any] = {
    "runtime": {
        "root": "/userdata/robot-car",
        "log_level": "INFO",
        "vision_socket": "/userdata/robot-car/runtime/vision.sock",
    },
    "web": {"enabled": False, "host": "127.0.0.1", "port": 8090,
            "preview_fps": 35, "preview_width": 640, "jpeg_quality": 75,
            "calibration_enabled": False},
    "camera": {"enabled": False, "device": "", "width": 640, "height": 480, "fps": 10,
               "calibration": {"image_to_capture_homography": [], "roller_balance": {}}},
    "vision": {"confirmation_frames": 3, "default_ttl_ms": 150, "plugins": []},
    "vehicle": {
        "control_enabled": False,
        "initial_mode": "IDLE",
        "default_valid_for_ms": 200,
        "heartbeat_hz": 20,
        "vision_timeout_ms": 500,
        "link_timeout_ms": 500,
        "capture": {
            "enabled": False,
            "output_only": False,
            "target_timeout_ms": 200,
            "feedback": {"enabled": False, "type": "none", "timeout_ms": 800},
            "no_feedback_policy": {"result": "CAPTURE_ATTEMPTED", "post_capture_action": "HOLD",
                                   "magnet_max_hold_ms": 3000},
        },
        "balance": {"enabled": False, "output_only": False, "require_feedback": True,
                    "state_timeout_ms": 120},
    },
    "transport": {
        "enabled": False,
        "type": "fake",
        "heartbeat": {"enabled": True},
        "uart": {"device": ""},
        "can": {"interface": "", "channel": "", "ids": {}},
    },
}


def _merge(base: Dict[str, Any], update: Dict[str, Any]) -> Dict[str, Any]:
    for key, value in update.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _merge(base[key], value)
        else:
            base[key] = value
    return base


def load_config(config_dir: str | Path) -> Dict[str, Any]:
    """Load all known files, retaining safe defaults on missing files."""
    result = copy.deepcopy(SAFE_DEFAULTS)
    directory = Path(config_dir)
    for name in ("base.yaml", "camera.yaml", "vision.yaml", "vehicle.yaml", "transport.yaml"):
        path = directory / name
        if not path.exists():
            continue
        with path.open("r", encoding="utf-8") as stream:
            document = yaml.safe_load(stream) or {}
        if not isinstance(document, dict):
            raise ValueError(f"configuration root must be a mapping: {path}")
        _merge(result, document)
    _validate_safe(result)
    return result


def _validate_safe(config: Dict[str, Any]) -> None:
    transport = config.get("transport", {})
    vehicle = config.get("vehicle", {})
    camera = config.get("camera", {})
    web = config.get("web", {})
    for label, value in (("transport.enabled", transport.get("enabled")),
                         ("vehicle.control_enabled", vehicle.get("control_enabled")),
                         ("camera.enabled", camera.get("enabled")),
                         ("web.enabled", web.get("enabled"))):
        if not isinstance(value, bool):
            raise ValueError(f"{label} must be a YAML boolean")
    preview_fps = float(web.get("preview_fps", 25))
    if not 1 <= preview_fps <= 60:
        raise ValueError("web.preview_fps must be between 1 and 60")
    preview_width = int(web.get("preview_width", 640))
    if preview_width < 160:
        raise ValueError("web.preview_width must be at least 160")
    jpeg_quality = int(web.get("jpeg_quality", 75))
    if not 1 <= jpeg_quality <= 100:
        raise ValueError("web.jpeg_quality must be between 1 and 100")
    if not isinstance(web.get("calibration_enabled", False), bool):
        raise ValueError("web.calibration_enabled must be a YAML boolean")
    if transport.get("enabled"):
        kind = transport.get("type")
        if kind == "uart" and not transport.get("uart", {}).get("device"):
            raise ValueError("enabled UART transport requires transport.uart.device")
        if kind == "can" and not transport.get("can", {}).get("channel"):
            raise ValueError("enabled CAN transport requires transport.can.channel")
    heartbeat = transport.get("heartbeat", {})
    if not isinstance(heartbeat, dict) or not isinstance(heartbeat.get("enabled", True), bool):
        raise ValueError("transport.heartbeat.enabled must be a YAML boolean")
    calibration = camera.get("calibration", {})
    if not isinstance(calibration, dict):
        raise ValueError("camera.calibration must be a mapping")
    homography = calibration.get("image_to_capture_homography", [])
    if not isinstance(homography, list):
        raise ValueError("camera.calibration.image_to_capture_homography must be a list")
    if homography:
        if len(homography) != 9:
            raise ValueError("camera.calibration.image_to_capture_homography must contain 9 values")
        try:
            if not all(math.isfinite(float(value)) for value in homography):
                raise ValueError
        except (TypeError, ValueError) as error:
            raise ValueError("camera.calibration.image_to_capture_homography must be finite") from error
    roller = calibration.get("roller_balance", {})
    if not isinstance(roller, dict):
        raise ValueError("camera.calibration.roller_balance must be a mapping")
    roi = roller.get("roi_xyxy", [])
    if roi and (not isinstance(roi, list) or len(roi) != 4):
        raise ValueError("camera.calibration.roller_balance.roi_xyxy must contain four values")
    validity = int(vehicle.get("default_valid_for_ms", 0))
    if not 0 < validity <= 0xFFFF:
        raise ValueError("vehicle.default_valid_for_ms must fit uint16 and be positive")
    if float(vehicle.get("heartbeat_hz", 0)) <= 0:
        raise ValueError("vehicle.heartbeat_hz must be positive")
    capture = vehicle.get("capture", {})
    feedback = capture.get("feedback", {})
    if not isinstance(capture.get("enabled", False), bool):
        raise ValueError("vehicle.capture.enabled must be a YAML boolean")
    if not isinstance(capture.get("output_only", False), bool):
        raise ValueError("vehicle.capture.output_only must be a YAML boolean")
    if not isinstance(feedback.get("enabled", False), bool):
        raise ValueError("vehicle.capture.feedback.enabled must be a YAML boolean")
    if feedback.get("type") not in {"none", "hall", "current", "switch", "vision"}:
        raise ValueError("vehicle.capture.feedback.type is invalid")
    if int(capture.get("target_timeout_ms", 0)) <= 0:
        raise ValueError("vehicle.capture.target_timeout_ms must be positive")
    if int(feedback.get("timeout_ms", 0)) <= 0:
        raise ValueError("vehicle.capture.feedback.timeout_ms must be positive")
    balance = vehicle.get("balance", {})
    if not isinstance(balance.get("enabled", False), bool):
        raise ValueError("vehicle.balance.enabled must be a YAML boolean")
    if not isinstance(balance.get("output_only", False), bool):
        raise ValueError("vehicle.balance.output_only must be a YAML boolean")
    if not isinstance(balance.get("require_feedback", True), bool):
        raise ValueError("vehicle.balance.require_feedback must be a YAML boolean")
    if not 0 < int(balance.get("state_timeout_ms", 0)) <= 0xFFFF:
        raise ValueError("vehicle.balance.state_timeout_ms must fit uint16 and be positive")


def ensure_runtime_dirs(config: Dict[str, Any]) -> Dict[str, Path]:
    root = Path(config["runtime"]["root"])
    paths = {name: root / name for name in ("logs", "recordings", "runtime", "calibration")}
    for path in paths.values():
        path.mkdir(parents=True, exist_ok=True)
    return paths
