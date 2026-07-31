#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
export PYTHONPATH="/userdata/rdkstudio/projects:$project_root/src${PYTHONPATH:+:$PYTHONPATH}"

config_dir="${CONFIG_DIR:-config}"
vision_pid=""
vision_log_dir="${VISION_LOG_DIR:-/userdata/robot-car/logs}"
mkdir -p "$vision_log_dir"

cleanup() {
    if [[ -n "$vision_pid" ]]; then
        kill "$vision_pid" 2>/dev/null || true
        wait "$vision_pid" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

# Keep model/BPU output out of the curses terminal; visiond also writes its
# structured log to the same runtime log directory.
python3 -m robot_car.visiond --config-dir "$config_dir" \
    >"$vision_log_dir/visiond-console.log" 2>&1 &
vision_pid=$!

python3 -m robot_car.vehicled --config-dir "$config_dir" --tui "$@"
