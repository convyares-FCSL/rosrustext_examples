#!/bin/bash
set -e
SCRIPT_DIR=$(dirname "$0")
source "$SCRIPT_DIR/../env_setup.sh"
sanitize_environment


echo "--> Building rclrs (Native Rust) Lessons..."
# Builds rclrs lessons 01, 02, and 03
colcon build --packages-select lesson_01_node_rclrs lesson_02_publisher_rclrs lesson_03_subscriber_rclrs lesson_04_service_rclrs lesson_05_parameters_rclrs
echo "rclrs Lessons built."
