#!/usr/bin/env bash
# Start roller vision plus the isolated direct-CAN controller.
set -euo pipefail

project_root=/userdata/rdkstudio/projects/robot_car
vision_pid=
control_pid=
task_mode=
forward_args=()
vision_socket=/userdata/robot-car/runtime/vision.sock

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
            forward_args+=("$1")
            shift
            ;;
    esac
done

if [[ -z "${task_mode}" ]]; then
    printf '选择滚珠动作：\n1. 控制钢球位于中心\n2. 回中（±5mm 持续超过2s）后执行 +5cm -> -5cm（默认需 Y 确认，目标5s，超时仅提示）\n选择 [1/2]: '
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
echo "ICM42688 已配置为 I2C；请先确认 can0 已配置并处于 UP。默认隐藏 CAN TX，需显示时追加 --show-tx。" >&2
PYTHONPATH=/userdata/rdkstudio/projects:"${project_root}/src" \
    python3 -m robot_car.visiond --config-dir config &
vision_pid=$!
for ((attempt = 0; attempt < 100; attempt++)); do
    [[ -S "${vision_socket}" ]] && break
    if ! kill -0 "${vision_pid}" 2>/dev/null; then
        wait "${vision_pid}" || true
        echo "视觉服务启动失败，未创建 ${vision_socket}" >&2
        exit 2
    fi
    sleep 0.1
done
if [[ ! -S "${vision_socket}" ]]; then
    echo "等待视觉事件 socket 超时：${vision_socket}" >&2
    exit 2
fi
echo "视觉事件服务已就绪，启动滚珠 CAN 控制器。" >&2
PYTHONPATH=/userdata/rdkstudio/projects:${project_root}/src \
    python3 -m robot_car.rollercontrold --config-dir config --task "${task_mode}" "${forward_args[@]}"
