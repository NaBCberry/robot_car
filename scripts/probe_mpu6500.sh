#!/usr/bin/env bash
# Read-only MPU6500 identity probe for the optional chassis-motion IMU.
set -euo pipefail

project_root="/userdata/rdkstudio/projects/robot_car"
bus="${1:-0}"
address="${2:-0x68}"

PYTHONPATH="/userdata/rdkstudio/projects:${project_root}/src" \
    python3 - "$bus" "$address" <<'PY'
import sys

from robot_car.roller_control.mpu6500 import Mpu6500

bus = int(sys.argv[1], 0)
address = int(sys.argv[2], 0)
sensor = Mpu6500(bus, address)
try:
    sensor.open()
    print(f"MPU6500 detected on i2c-{bus} address 0x{address:02X}: WHO_AM_I=0x70")
finally:
    sensor.close()
PY
