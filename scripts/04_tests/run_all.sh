#!/bin/bash
# Combined Test Runner for all Tutorial Tracks
# This script executes per-track suites and provides a unified summary.

source "$(dirname "$0")/../../scripts/05_utils/test_helpers.sh"

echo "=================================================="
echo "RUNNING ALL TEST TRACKS"
echo "=================================================="

declare -A RESULTS

run_suite() {
    local name=$1
    local script=$2
    
    echo -e "\n>>> Suite: $name ($script)"
    bash "$script"
    local ret=$?
    
    if [ $ret -eq 0 ]; then
        RESULTS["$name"]="PASS"
    else
        RESULTS["$name"]="FAIL"
    fi
    
    # Hygiene barrier between suites
    if ! assert_clean_graph; then
        echo "WARNING: Hygiene check failed after $name suite."
    fi
}

# Source environment
PROJ_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
source "$PROJ_ROOT/scripts/env_setup.sh" > /dev/null 2>&1

# Run suites
run_suite "Python" "$PROJ_ROOT/scripts/04_tests/run_python.sh"
run_suite "C++" "$PROJ_ROOT/scripts/04_tests/run_cpp.sh"
run_suite "Rust/rclrs" "$PROJ_ROOT/scripts/04_tests/run_rclrs.sh"

# Final Table
echo -e "\n=================================================="
echo "FINAL TEST SUMMARY"
echo "=================================================="
printf "%-20s | %s\n" "Track" "Status"
echo "---------------------|----------"

TOTAL_FAIL=0
for track in "Python" "C++" "Rust/rclrs"; do
    status=${RESULTS[$track]}
    if [ "$status" == "PASS" ]; then
        printf "%-20s | \033[0;32m%-10s\033[0m\n" "$track" "$status"
    else
        printf "%-20s | \033[0;31m%-10s\033[0m\n" "$track" "$status"
        TOTAL_FAIL=$((TOTAL_FAIL + 1))
    fi
done
echo "=================================================="

if [ $TOTAL_FAIL -eq 0 ]; then
    echo -e "\033[0;32mALL TRACKS PASSED\033[0m"
    exit 0
else
    echo -e "\033[0;31m$TOTAL_FAIL TRACK(S) FAILED\033[0m"
    exit 1
fi
