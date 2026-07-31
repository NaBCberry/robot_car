#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
export PYTHONPATH="$project_root/src${PYTHONPATH:+:$PYTHONPATH}"

config_dir="${CONFIG_DIR:-config}"
vision_pid=""

cleanup() {
    if [[ -n "$vision_pid" ]]; then
        kill "$vision_pid" 2>/dev/null || true
        wait "$vision_pid" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

python3 -m robot_car.visiond --config-dir "$config_dir" &
vision_pid=$!

python3 -m robot_car.vehicled --config-dir "$config_dir" --tui "$@"
