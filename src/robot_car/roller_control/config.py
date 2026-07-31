"""Configuration loader for the isolated roller CAN control chain."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from .control import LinearTable, Pid, PidParameters, RollerController


def load_roller_control(path: str | Path) -> dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream) or {}
    if not isinstance(config, dict):
        raise ValueError("roller control configuration must be a mapping")
    _require_mapping(config, "imu")
    _require_mapping(config, "motor")
    _require_mapping(config, "control")
    return config


def build_controller(config: dict[str, Any]) -> RollerController:
    control = _require_mapping(config, "control")
    limits = _require_mapping(control, "limits")
    return RollerController(
        _pid(_require_mapping(control, "position_pid")),
        _pid(_require_mapping(control, "velocity_pid")),
        _pid(_require_mapping(control, "angle_pid")),
        LinearTable(control.get("slope_bias_deg_by_position", []), name="slope_bias_deg_by_position"),
        LinearTable(control.get("motor_deg_by_tube_angle", []), name="motor_deg_by_tube_angle"),
        target_min_mm=float(limits["target_min_mm"]), target_max_mm=float(limits["target_max_mm"]),
        tube_angle_min_deg=float(limits["tube_angle_min_deg"]),
        tube_angle_max_deg=float(limits["tube_angle_max_deg"]),
        tilt_sign=float(control.get("tilt_sign", 1)),
    )


def _pid(value: dict[str, Any]) -> Pid:
    return Pid(PidParameters(kp=float(value["kp"]), ki=float(value["ki"]), kd=float(value["kd"]),
                             output_limit=float(value["output_limit"]),
                             integral_limit=float(value["integral_limit"])))


def _require_mapping(value: dict[str, Any], key: str) -> dict[str, Any]:
    result = value.get(key)
    if not isinstance(result, dict):
        raise ValueError(f"roller control {key} must be a mapping")
    return result
