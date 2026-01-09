#!/bin/bash
set -e

SCRIPT_DIR=$(dirname "$0")
source "$SCRIPT_DIR/../env_setup.sh"
sanitize_environment

echo "--> Building Python Lessons..."
# Builds Python lessons 01, 02, and 03
colcon build --packages-select lesson_01_node_py lesson_02_publisher_py lesson_03_subscriber_py
echo "Python Lessons built."
