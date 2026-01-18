#!/bin/bash

# Shared Test Helpers for ROS 2 Tutorial
# Provides process group management, cleanup, and assertion logic.

# Global tracking associative array for PIDs (Process Group IDs)
# Keys are PIDs, values are human-readable labels (optional)
declare -A TRACKED_PIDS=()
TEST_DIR="$(dirname "${BASH_SOURCE[0]}")"

# --- Process Management ---

# Start a command in a new session (process group) and track it
# Usage: start_node_bg <log_file> <cmd> [args...]
# Returns: PGID in $RET_PID
start_node_bg() {
    local log_file=$1
    shift
    local cmd=("$@")

    # Start node in background. We avoid setsid -w here to ensure 
    # signal propagation and wait behavior are predictable.
    "$@" > "$log_file" 2>&1 &
    local pid=$!
    
    TRACKED_PIDS[$pid]=1
    RET_PID=$pid
}

# Stop a specific process and its children
stop_node() {
    local pid=$1
    if [ -z "$pid" ] || [ -z "${TRACKED_PIDS[$pid]:-}" ]; then return; fi

    # 1. Identify current children (e.g. ROS nodes under the wrapper)
    local children=$(pgrep -P $pid || true)

    # 2. Try SIGINT gracefully to the whole tree
    kill -INT $pid $children 2>/dev/null
    
    # Wait up to 5 seconds for the entire tree to vanish
    for ((i=0; i<50; i++)); do
        if ! ps -p $pid $children > /dev/null 2>&1; then
            unset TRACKED_PIDS[$pid]
            return 0
        fi
        sleep 0.1
        # Refresh children list in case new ones appeared (unlikely but safe)
        children=$(pgrep -P $pid 2>/dev/null || true)
    done

    # 3. Try SIGTERM
    kill -TERM $pid $children 2>/dev/null
    for ((i=0; i<20; i++)); do
        if ! ps -p $pid $children > /dev/null 2>&1; then
            unset TRACKED_PIDS[$pid]
            return 0
        fi
        sleep 0.1
    done

    # 4. Force kill residue
    kill -KILL $pid $children 2>/dev/null
    sleep 0.5
    unset TRACKED_PIDS[$pid]
}

# Kill all tracked processes
cleanup_all_lessons() {
    if [ ${#TRACKED_PIDS[@]} -eq 0 ]; then return; fi
    
    # Send SIGINT to all tracked processes
    for pid in "${!TRACKED_PIDS[@]}"; do
        local children=$(pgrep -P $pid 2>/dev/null || true)
        kill -INT $pid $children 2>/dev/null
    done
    
    # Sequential wait
    for pid in "${!TRACKED_PIDS[@]}"; do
        for ((i=0; i<20; i++)); do
            local children=$(pgrep -P $pid 2>/dev/null || true)
            if ! ps -p $pid $children > /dev/null 2>&1; then break; fi
            sleep 0.1
        done
    done
    
    # Force kill any remaining
    for pid in "${!TRACKED_PIDS[@]}"; do
        if ps -p $pid > /dev/null 2>&1; then
             local children=$(pgrep -P $pid 2>/dev/null || true)
             kill -KILL $pid $children 2>/dev/null
        fi
    done
    
    TRACKED_PIDS=()
}

# Signal handler
_handle_signal() {
    local sig=$1
    # echo "Caught $sig"
    cleanup_all_lessons
    exit 1
}

# Register traps
setup_traps() {
    trap cleanup_all_lessons EXIT
    trap "_handle_signal INT" INT
    trap "_handle_signal TERM" TERM
}

# --- Assertions ---

# Check that the ROS graph and process list are clean
assert_clean_graph() {
    local fail=0
    local tutorial_regex='^/lesson_[0-9]{2}_[A-Za-z0-9_]+$|/lifecycle_manager'
    
    # Give ROS graph discovery a moment to settle
    sleep 5

    # 1. Check ROS 2 Node List
    local nodes=$(ros2 node list 2>/dev/null | grep -E "$tutorial_regex" || true)
    if [ -n "$nodes" ]; then
        echo "WARNING: Residue found in graph: $nodes"
        echo "Flushing daemon..."
        ros2 daemon stop >/dev/null 2>&1 || true
        sleep 2
        ros2 daemon start >/dev/null 2>&1 || true
        sleep 2
        
        nodes=$(ros2 node list 2>/dev/null | grep -E "$tutorial_regex" || true)
        if [ -n "$nodes" ]; then
            echo "ERROR: Graph not clean after flush. Found: $nodes"
            fail=1
        else
             echo "Graph flushed successfully."
        fi
    fi
    
    # 2. Check Process List (ps)
    local procs=$(pgrep -af "lesson_|lifecycle_manager|service_server" | grep -v "$$" | grep -v "run_tests" || true)
    if [ -n "$procs" ]; then
         echo "ERROR: Process list not clean. Found leftover processes:"
         echo "$procs"
         fail=1
    fi
    
    return $fail
}

# Pre-check to ensure we start clean
ensure_clean_start() {
    if ! assert_clean_graph; then
        echo "WARNING: System dirty before test start. Attempting cleanup..."
        # Try to kill anything matching our patterns
        pkill -f "lesson_|service_server|lifecycle_manager" 2>/dev/null
        sleep 2
        
        if ! assert_clean_graph; then
             echo "CRITICAL: Cannot clean system. Aborting test."
             exit 1
        fi
        echo "System cleaned."
    fi
}
