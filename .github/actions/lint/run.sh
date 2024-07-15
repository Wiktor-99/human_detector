#!/bin/bash
set -e

sudo apt-get update
./setup.sh

if [[ "ament_flake8" == "ament_${LINTER}" ]]; then
    ament_${LINTER} . --config python_linter.flake8
else
    ament_${LINTER}
fi
