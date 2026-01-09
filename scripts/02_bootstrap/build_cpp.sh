#!/bin/bash
set -e
SCRIPT_DIR=$(dirname "$0")
source "$SCRIPT_DIR/../env_setup.sh"
sanitize_environment


echo "--> Building C++ Bootstrap..."
colcon build --packages-select utils_cpp lesson_00_bootstrap_cpp
echo "C++ Bootstrap built."
