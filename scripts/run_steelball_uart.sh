#!/usr/bin/env bash
# Start the steel-ball perception and UART target-output pipeline.
set -euo pipefail

project_root=$(cd "$(dirname "$0")/.." && pwd)
camera_device=/dev/video0
uart_device=/dev/ttyS1
baudrate=115200
allow_motion=0
allow_temporary_calibration=0

usage() {
    cat <<'EOF'
用法：
  run_steelball_uart.sh [--camera DEVICE] [--uart DEVICE] [--baudrate RATE]
                         [--allow-motion --allow-temporary-calibration]

默认行为：打开相机和 /dev/ttyS1，识别画面中最下方的钢球，并向 UART 持续发送
v2 CAPTURE_TARGET_POLAR 的角度和距离。默认强制 enabled=0，MSPM0 必须不驱动电机。

选项：
  --camera DEVICE                    摄像头，默认 /dev/video0
  --uart DEVICE                      UART，默认 /dev/ttyS1
  --baudrate RATE                    UART 波特率，默认 115200
  --allow-motion                     允许 MSPM0 根据目标驱动车辆
  --allow-temporary-calibration      确认目前使用的是临时标定，必须与 --allow-motion 同时给出
  -h, --help                         显示本帮助
EOF
}

while (($#)); do
    case "$1" in
        --camera) camera_device=$2; shift 2 ;;
        --uart) uart_device=$2; shift 2 ;;
        --baudrate) baudrate=$2; shift 2 ;;
        --allow-motion) allow_motion=1; shift ;;
        --allow-temporary-calibration) allow_temporary_calibration=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "未知参数：$1" >&2; usage >&2; exit 2 ;;
    esac
done

if ((allow_temporary_calibration && !allow_motion)); then
    echo "--allow-temporary-calibration 只能与 --allow-motion 一起使用" >&2
    exit 2
fi
if ((allow_motion && !allow_temporary_calibration)); then
    echo "拒绝启动运动：临时标定必须同时传 --allow-temporary-calibration" >&2
    exit 2
fi
if [[ ! -c "$camera_device" ]]; then
    echo "摄像头不是字符设备：$camera_device" >&2
    exit 2
fi
if [[ ! -c "$uart_device" ]]; then
    echo "UART 不是字符设备：$uart_device" >&2
    exit 2
fi
if [[ ! "$baudrate" =~ ^[0-9]+$ ]] || ((baudrate <= 0)); then
    echo "--baudrate 必须为正整数" >&2
    exit 2
fi

model_path=/userdata/rdkstudio/projects/ultralytics_yolo26/model/steelball_seg_bpu_bayese_640x640_nv12.bin
if [[ ! -f "$model_path" ]]; then
    echo "钢球模型不存在" >&2
    exit 2
fi
if ! python3 -c 'import serial' >/dev/null 2>&1; then
    echo "缺少 pyserial，无法打开真实 UART" >&2
    exit 2
fi

runtime_config=$(mktemp -d /tmp/robot-car-steelball.XXXXXX)
vehicle_pid=
vision_pid=
cleanup() {
    [[ -n "$vision_pid" ]] && kill "$vision_pid" 2>/dev/null || true
    [[ -n "$vehicle_pid" ]] && kill "$vehicle_pid" 2>/dev/null || true
    [[ -n "$vision_pid" ]] && wait "$vision_pid" 2>/dev/null || true
    [[ -n "$vehicle_pid" ]] && wait "$vehicle_pid" 2>/dev/null || true
    rm -rf -- "$runtime_config"
}
trap cleanup EXIT INT TERM

PYTHONPATH="$project_root/src" python3 - "$project_root/config" "$runtime_config" \
    "$camera_device" "$uart_device" "$baudrate" "$model_path" "$allow_motion" <<'PY'
from pathlib import Path
import math
import sys

import yaml

(source_dir, output_dir, camera, uart, baudrate, model, allow_motion) = sys.argv[1:]
source = Path(source_dir)
output = Path(output_dir)
documents = {}
for name in ("base.yaml", "camera.yaml", "vision.yaml", "vehicle.yaml", "transport.yaml"):
    documents[name] = yaml.safe_load((source / name).read_text(encoding="utf-8")) or {}

web = documents["base.yaml"].setdefault("web", {})
web.setdefault("enabled", False)
web.setdefault("host", "127.0.0.1")
web.setdefault("port", 8090)
documents["camera.yaml"]["camera"].update({
    "enabled": True, "device": camera, "width": 1280, "height": 720, "fps": 30,
    "pixel_format": "MJPG",
})
calibration = documents["camera.yaml"]["camera"].get("calibration", {})
homography = calibration.get("image_to_capture_homography", [])
try:
    valid_homography = (isinstance(homography, list) and len(homography) == 9
                        and all(math.isfinite(float(value)) for value in homography))
except (TypeError, ValueError):
    valid_homography = False
if not valid_homography:
    raise SystemExit("camera.yaml 中缺少有效的 camera.calibration.image_to_capture_homography")
vision = documents["vision.yaml"]["vision"]
vision["confirmation_frames"] = 1
for plugin in vision.get("plugins", []):
    plugin["enabled"] = plugin.get("name") == "steelball"
    if plugin.get("name") == "steelball":
        plugin["config"].update({
            "model_path": model,
            "calibration_path": "",
            "image_to_capture_homography": homography,
        })
vehicle = documents["vehicle.yaml"]["vehicle"]
vehicle["control_enabled"] = True
vehicle["initial_mode"] = "IDLE"
vehicle["capture"].update({"enabled": True, "output_only": allow_motion != "1"})
documents["transport.yaml"]["transport"].update({"enabled": True, "type": "uart"})
documents["transport.yaml"]["transport"]["uart"].update({"device": uart, "baudrate": int(baudrate)})

for name, document in documents.items():
    (output / name).write_text(yaml.safe_dump(document, allow_unicode=True, sort_keys=False), encoding="utf-8")
PY

if ((allow_motion)); then
    echo "警告：已授权临时标定下的车辆运动。仅可在车轮悬空、急停有效的环境使用。" >&2
else
    echo "输出模式：UART 会收到角度和距离，但 enabled=0，MSPM0 不得驱动电机。" >&2
fi

"$project_root/scripts/run_vehicled.sh" --config-dir "$runtime_config" --transport uart &
vehicle_pid=$!
"$project_root/scripts/run_visiond.sh" --config-dir "$runtime_config" &
vision_pid=$!
wait -n "$vision_pid" "$vehicle_pid"
