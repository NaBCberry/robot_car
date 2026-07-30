#!/usr/bin/env bash
# Start the top-view roller-balance vision and safe UART output together.
set -euo pipefail

project_root=/userdata/rdkstudio/projects/robot_car
vision_pid=
vehicle_pid=

cleanup() {
    [[ -n "${vision_pid}" ]] && kill "${vision_pid}" 2>/dev/null || true
    [[ -n "${vehicle_pid}" ]] && kill "${vehicle_pid}" 2>/dev/null || true
    [[ -n "${vision_pid}" ]] && wait "${vision_pid}" 2>/dev/null || true
    [[ -n "${vehicle_pid}" ]] && wait "${vehicle_pid}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if [[ ! -c /dev/ttyS1 ]]; then
    echo "UART 不是字符设备：/dev/ttyS1" >&2
    exit 2
fi

cd "${project_root}"
echo "输出模式：向 /dev/ttyS1 发送 BALANCE_ROLLER/error_mm，enabled=0。" >&2
echo "请确认 MSPM0 已按当前 v2 的 6 字节平衡 payload 解析。" >&2

./scripts/run_visiond.sh &
vision_pid=$!
./scripts/run_vehicled.sh --transport uart &
vehicle_pid=$!
wait -n "${vision_pid}" "${vehicle_pid}"
