#!/usr/bin/env bash
# Read only: verify an ICM42688 identity register using roller_control.yaml.
set -euo pipefail

project_root=/userdata/rdkstudio/projects/robot_car
control_config=${1:-"${project_root}/config/roller_control.yaml"}
if [[ "${control_config}" != /* ]]; then
    control_config="${project_root}/${control_config}"
fi

PYTHONPATH=/userdata/rdkstudio/projects:"${project_root}/src" \
    python3 - "${control_config}" <<'PY'
import sys
from pathlib import Path

from robot_car.roller_control.config import load_roller_control
from robot_car.roller_control.icm42688 import sensor_from_config

config = load_roller_control(Path(sys.argv[1]))
sensor = sensor_from_config(config["imu"])
try:
    sensor.open()
except (OSError, RuntimeError) as error:
    print(error)
    raise SystemExit(2)
else:
    if sensor.transport == "i2c":
        print("ICM42688-P detected on i2c-%d address 0x%02X: WHO_AM_I=0x47" %
              (sensor.bus, sensor.i2c_address))
    else:
        print("ICM42688-P detected on spi%d.%d: WHO_AM_I=0x47" %
              (sensor.bus, sensor.chip_select))
finally:
    sensor.close()
PY
