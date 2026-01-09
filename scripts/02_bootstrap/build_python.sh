#!/bin/bash
set -e
SCRIPT_DIR=$(dirname "$0")
source "$SCRIPT_DIR/../env_setup.sh"
sanitize_environment


echo "--> Building Python Bootstrap..."
colcon build --packages-select utils_py lesson_00_bootstrap_py
echo "Python Bootstrap built."
