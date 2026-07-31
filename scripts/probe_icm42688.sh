#!/usr/bin/env bash
# Read only: verify an ICM42688 identity register on a SPI1 chip select.
set -euo pipefail

project_root=/userdata/rdkstudio/projects/robot_car
chip_select=${1:-1}

if [[ ! "${chip_select}" =~ ^[01]$ ]]; then
    echo "用法：$0 [0|1]" >&2
    exit 2
fi

PYTHONPATH=/userdata/rdkstudio/projects:"${project_root}/src" \
    python3 - "${chip_select}" <<'PY'
import sys

from robot_car.roller_control.icm42688 import Icm42688

chip_select = int(sys.argv[1])
sensor = Icm42688(1, chip_select, speed_hz=100_000, mode=0)
try:
    sensor.open()
except RuntimeError as error:
    print(error)
    raise SystemExit(2)
else:
    print(f"ICM42688 detected on spi1.{chip_select}: WHO_AM_I=0x47")
finally:
    sensor.close()
PY
