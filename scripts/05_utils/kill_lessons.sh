#!/usr/bin/env bash
set -euo pipefail

echo "Stopping ros2 daemon..."
ros2 daemon stop >/dev/null 2>&1 || true

echo "Killing lesson processes (python + native)..."
# Kill real installed lesson executables (python nodes and compiled nodes) by path.
pkill -9 -f "/home/ecm/ros2_ws_tutorial/install/lesson_" >/dev/null 2>&1 || true

echo "Killing stray ros2 CLI wrappers that are still running lesson nodes..."
# Kill ros2 wrapper invocations that can outlive/respawn children.
pkill -9 -f "/opt/ros/.*/bin/ros2 run lesson_" >/dev/null 2>&1 || true
pkill -9 -f "ros2 run lesson_" >/dev/null 2>&1 || true

echo "Restarting ros2 daemon..."
ros2 daemon start >/dev/null 2>&1 || true

echo "Remaining lesson processes (should be empty):"
ps aux | grep -E "ros2_ws_tutorial/install/lesson_|/opt/ros/.*/bin/ros2 run lesson_" | grep -v grep || true

echo "Remaining lesson nodes in ROS graph (should be empty):"
sleep 2
ros2 node list | grep -E "^/lesson_|^/lifecycle_manager" || echo "NONE"
