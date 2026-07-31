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
echo "ICM42688 已配置为 I2C；请先确认 can0 已配置并处于 UP，再显式传入 --arm。" >&2
PYTHONPATH=/userdata/rdkstudio/projects:"${project_root}/src" \
    python3 -m robot_car.visiond --config-dir config &
vision_pid=$!
PYTHONPATH=/userdata/rdkstudio/projects:${project_root}/src \
    python3 -m robot_car.rollercontrold --config-dir config "$@" &
control_pid=$!
wait -n "${vision_pid}" "${control_pid}"
