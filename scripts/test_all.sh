#!/bin/bash
set -e

# Source environment
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
else
    echo "Error: install/setup.bash not found. Did you build the workspace?"
    exit 1
fi

# Run python test runner
python3 scripts/04_tests/run_tests.py
