#!/bin/bash
set -e
SCRIPT_DIR=$(dirname "$0")
# SCRIPT_DIR is scripts/01_setup, so we need to go up one level to find env_setup.sh
source "$SCRIPT_DIR/../env_setup.sh"
sanitize_environment

echo "--> Building Interfaces..."
colcon build --packages-select rosidl_generator_rs lesson_interfaces
echo "Interfaces built."
