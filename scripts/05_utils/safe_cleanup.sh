#!/bin/bash
# Safer cleanup than pkill -f ros2

# Kill specific lesson nodes
pkill -f "lesson_01"
pkill -f "lesson_02"
pkill -f "lesson_03"
pkill -f "lesson_04"
pkill -f "lesson_05"
pkill -f "lesson_06"
pkill -f "lifecycle_manager"

# Wait a moment for them to die
sleep 2

echo "Cleanup complete."
