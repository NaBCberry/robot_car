#!/usr/bin/env bash
# Print ICM-42688-P samples and pitch estimates. This script never opens CAN.
set -euo pipefail

project_root=/userdata/rdkstudio/projects/robot_car
cd "${project_root}"
PYTHONPATH=/userdata/rdkstudio/projects:"${project_root}/src" \
    python3 -m robot_car.imu_monitor "$@"
