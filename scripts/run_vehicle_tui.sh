#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
export PYTHONPATH="/userdata/rdkstudio/projects:$project_root/src${PYTHONPATH:+:$PYTHONPATH}"

config_dir="${CONFIG_DIR:-config}"
can_interface="${CAN_INTERFACE:-can0}"
can_bitrate="${CAN_BITRATE:-500000}"
vision_pid=""
vision_log_dir="${VISION_LOG_DIR:-/userdata/robot-car/logs}"
vehicle_args=("$@")
has_transport=false
for arg in "${vehicle_args[@]}"; do
    case "$arg" in
        --transport|--transport=*)
            has_transport=true
            ;;
    esac
done
if [[ "$has_transport" == false ]]; then
    vehicle_args+=(--transport uart)
fi
mkdir -p "$vision_log_dir"

configure_can() {
    if ! ip link show dev "$can_interface" >/dev/null 2>&1; then
        echo "未找到 CAN 接口：$can_interface" >&2
        return 1
    fi
    if ! ip -o link show dev "$can_interface" | grep -q '<[^>]*UP[^>]*>' || \
            ! ip -details link show dev "$can_interface" | grep -q "bitrate $can_bitrate"; then
        echo "配置 $can_interface：${can_bitrate} bit/s" >&2
        ip link set dev "$can_interface" down
        ip link set dev "$can_interface" up type can bitrate "$can_bitrate"
    fi
    if ! ip -o link show dev "$can_interface" | grep -q '<[^>]*UP[^>]*>'; then
        echo "CAN 接口未成功启动：$can_interface" >&2
        return 1
    fi
}

cleanup() {
    if [[ -n "$vision_pid" ]]; then
        kill "$vision_pid" 2>/dev/null || true
        wait "$vision_pid" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

configure_can

# Keep model/BPU output out of the curses terminal; visiond also writes its
# structured log to the same runtime log directory.
python3 -m robot_car.visiond --config-dir "$config_dir" \
    >"$vision_log_dir/visiond-console.log" 2>&1 &
vision_pid=$!

python3 -m robot_car.vehicled --config-dir "$config_dir" --tui "${vehicle_args[@]}"
