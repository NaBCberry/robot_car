#!/bin/sh
set -eu
PROJECT_ROOT=/userdata/rdkstudio/projects/robot_car
export PYTHONPATH=/userdata/rdkstudio/projects:${PROJECT_ROOT}/src
cd "${PROJECT_ROOT}"
exec python3 -m robot_car.visiond --config-dir config "$@"
