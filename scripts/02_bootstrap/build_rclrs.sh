#!/bin/bash
set -e
SCRIPT_DIR=$(dirname "$0")
source "$SCRIPT_DIR/../env_setup.sh"
sanitize_environment


echo "--> Building rclrs (Native Rust) Bootstrap..."
colcon build --packages-select utils_rclrs lesson_00_bootstrap_rclrs
echo "rclrs Bootstrap built."
