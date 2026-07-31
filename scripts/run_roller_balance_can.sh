#!/usr/bin/env bash
# Start roller vision plus the isolated direct-CAN controller.
set -euo pipefail

project_root=/userdata/rdkstudio/projects/robot_car
vision_pid=
control_pid=
task_mode=

while [[ $# -gt 0 ]]; do
    case "$1" in
        --task)
            [[ $# -ge 2 ]] || { echo "--task 需要 1 或 2" >&2; exit 2; }
            task_mode="$2"
            shift 2
            ;;
        --task=*)
            task_mode="${1#*=}"
            shift
            ;;
        *)
            break
            ;;
    esac
done

if [[ -z "${task_mode}" ]]; then
    printf '选择滚珠动作：\n1. 控制钢球位于中心\n2. 回中稳定后执行 +5cm -> -5cm（目标5s，超时仅提示）\n选择 [1/2]: '
    read -r task_mode
fi
case "${task_mode}" in
    1|2) ;;
    *) echo "动作选项必须是 1 或 2" >&2; exit 2 ;;
esac

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
    python3 -m robot_car.rollercontrold --config-dir config --task "${task_mode}" "$@"
