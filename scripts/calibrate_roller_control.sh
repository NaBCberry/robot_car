#!/usr/bin/env bash
# Interactive Y42 travel and crank calibration. See --help before use.
set -euo pipefail

project_root=/userdata/rdkstudio/projects/robot_car
cd "${project_root}"
PYTHONPATH=/userdata/rdkstudio/projects:"${project_root}/src" \
    python3 -m robot_car.roller_calibrate "$@"
