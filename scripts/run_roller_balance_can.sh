#!/usr/bin/env bash
# Start roller vision plus the isolated direct-CAN controller.
set -euo pipefail

project_root=/userdata/rdkstudio/projects/robot_car
vision_pid=
control_pid=

cleanup() {
    [[ -n "${vision_pid}" ]] && kill "${vision_pid}" 2>/dev/null || true
    [[ -n "${control_pid}" ]] && kill "${control_pid}" 2>/dev/null || true
    [[ -n "${vision_pid}" ]] && wait "${vision_pid}" 2>/dev/null || true
    [[ -n "${control_pid}" ]] && wait "${control_pid}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

cd "${project_root}"
echo "默认仅观测。请先修复 ICM42688 WHO_AM_I=0x47 检测，再在 roller_control.yaml 启用并显式传入 --arm。" >&2
PYTHONPATH=/userdata/rdkstudio/projects:"${project_root}/src" \
    python3 -m robot_car.visiond --config-dir config &
vision_pid=$!
PYTHONPATH=/userdata/rdkstudio/projects:${project_root}/src \
    python3 -m robot_car.rollercontrold --config-dir config "$@" &
control_pid=$!
wait -n "${vision_pid}" "${control_pid}"
