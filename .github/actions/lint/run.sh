#!/bin/bash
set -e

sudo apt-get update
source /opt/ros/$ROS_DISTRO/setup.bash
if [[ "ament_flake8" == "ament_${LINTER}" ]]; then
    ament_${LINTER} . --config python_linter.flake8
else
    ament_${LINTER}
fi
