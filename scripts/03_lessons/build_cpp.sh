#!/bin/bash
set -e
SCRIPT_DIR=$(dirname "$0")
source "$SCRIPT_DIR/../env_setup.sh"
sanitize_environment


echo "--> Building C++ Lessons..."
# Builds C++ lessons 01, 02, and 03
colcon build --packages-select lesson_01_node_cpp lesson_02_publisher_cpp lesson_03_subscriber_cpp lesson_04_service_cpp
echo "C++ Lessons built."
