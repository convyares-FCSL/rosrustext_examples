#!/bin/bash
set +e

set -o pipefail
# Track any failure across steps so the final exit reflects reality.
FAILED=0

source "$(dirname "$0")/../env_setup.sh"
source "$(dirname "$0")/../../scripts/05_utils/test_helpers.sh"

sanitize_environment
# Source Workspace
FILES="$(dirname "$0")/../../install/setup.bash"
if [ -f "$FILES" ]; then source "$FILES"; else echo "Error: install/setup.bash not found. Build first."; exit 1; fi

TOPICS_YAML="src/4_interfaces/lesson_interfaces/config/topics_config.yaml"
if [ ! -f "$TOPICS_YAML" ]; then
  echo "Error: $TOPICS_YAML not found"
  exit 1
fi

# Extract telemetry topic from the same config the lessons use.
TELEMETRY_TOPIC="$(python3 - <<'PY'
import sys
try:
  import yaml
except Exception as e:
  print("", end="")
  sys.exit(0)

with open("src/4_interfaces/lesson_interfaces/config/topics_config.yaml","r") as f:
  d = yaml.safe_load(f) or {}

# Handle complex nested structure: /** -> ros__parameters -> topics -> telemetry
topic = None
if isinstance(d, dict):
    # Try fully qualified path from YAML
    wildcard = d.get("/**")
    if isinstance(wildcard, dict):
        params = wildcard.get("ros__parameters")
        if isinstance(params, dict):
            topics = params.get("topics")
            if isinstance(topics, dict):
                topic = topics.get("telemetry")

    # Fallback for simpler structures
    if topic is None:
        topic = d.get("telemetry")
        if topic is None and isinstance(d.get("tutorial"), dict):
            topic = d["tutorial"].get("telemetry")

print(topic or "", end="")
PY
)"
if [ -z "$TELEMETRY_TOPIC" ]; then
  echo "Error: could not resolve telemetry topic from $TOPICS_YAML"
  exit 1
fi

LOG_BASE="log/tests/roslibrust"
mkdir -p "$LOG_BASE"
rm -f "$LOG_BASE"/*.log

# Hardened cleanup: use repo util + standard traps
custom_cleanup() {
    echo "Performing hardened cleanup..."
    cleanup_all_lessons
    # Force kill any remaining infra
    "$(dirname "$0")/../../scripts/05_utils/kill_lessons.sh" > /dev/null 2>&1
    fuser -k 9090/tcp > /dev/null 2>&1 || true
}
trap custom_cleanup EXIT

# Pre-test cleanup
custom_cleanup
ensure_clean_start

echo "=================================================="
echo "TEST SUITE: roslibrust Lesson 06 Lifecycle"
echo "=================================================="

check_ret() {
    if [ $1 -eq 0 ]; then
        echo -e "  \033[0;32mPASS\033[0m"
    else
        echo -e "  \033[0;31mFAIL (See log)\033[0m"
        FAILED=1
    fi
}

require_pid_alive() {
    local pid="$1"
    if [ -z "$pid" ] || ! kill -0 "$pid" 2>/dev/null; then
        return 1
    fi
    return 0
}

# --- 1. Infrastructure ---
echo -n "[Step 1] Rosbridge + Proxies Startup... "
start_node_bg "$LOG_BASE/rosbridge.log" ros2 launch rosbridge_server rosbridge_websocket_launch.xml
PID_BRIDGE=$RET_PID
# Give rosbridge a moment to bind the socket
sleep 3
start_node_bg "$LOG_BASE/proxy_pub.log" rosrustext_lifecycle_proxy --target-node lesson_06_lifecycle_publisher --node-name lesson_06_lifecycle_publisher
PID_PRX_PUB=$RET_PID
start_node_bg "$LOG_BASE/proxy_sub.log" rosrustext_lifecycle_proxy --target-node lesson_06_lifecycle_subscriber --node-name lesson_06_lifecycle_subscriber
PID_PRX_SUB=$RET_PID
sleep 5

RET=0
require_pid_alive "$PID_BRIDGE" || RET=1
require_pid_alive "$PID_PRX_PUB" || RET=1
require_pid_alive "$PID_PRX_SUB" || RET=1

# Prove the proxy is actually serving lifecycle, not just alive.
ros2 lifecycle get /lesson_06_lifecycle_publisher > "$LOG_BASE/get_state_pub.log" 2>&1
RET_GET_PUB=$?
ros2 lifecycle get /lesson_06_lifecycle_subscriber > "$LOG_BASE/get_state_sub.log" 2>&1
RET_GET_SUB=$?
RET=$((RET + RET_GET_PUB + RET_GET_SUB))

check_ret $RET

# --- 2. Lesson Nodes (Direct Binary) ---
echo -n "[Step 2] Binary Startup (Unconfigured)... "
PUB_BIN="install/lesson_06_lifecycle_roslibrust/lib/lesson_06_lifecycle_roslibrust/lesson_06_lifecycle_publisher"
SUB_BIN="install/lesson_06_lifecycle_roslibrust/lib/lesson_06_lifecycle_roslibrust/lesson_06_lifecycle_subscriber"

if [ ! -x "$PUB_BIN" ] || [ ! -x "$SUB_BIN" ]; then
  echo -e "  \033[0;31mFAIL (missing binaries; build first)\033[0m"
  exit 1
fi

start_node_bg "$LOG_BASE/binary_pub.log" "$PUB_BIN" --ros-args --params-file src/4_interfaces/lesson_interfaces/config/topics_config.yaml --params-file src/4_interfaces/lesson_interfaces/config/qos_config.yaml
PID_PUB=$RET_PID
start_node_bg "$LOG_BASE/binary_sub.log" "$SUB_BIN" --ros-args --params-file src/4_interfaces/lesson_interfaces/config/topics_config.yaml --params-file src/4_interfaces/lesson_interfaces/config/qos_config.yaml
PID_SUB=$RET_PID
sleep 5

RET=0
require_pid_alive "$PID_PUB" || RET=1
require_pid_alive "$PID_SUB" || RET=1
check_ret $RET

# --- 3. Configuration ---
echo -n "[Step 3] Lifecycle Configure... "
ros2 lifecycle set /lesson_06_lifecycle_publisher configure > "$LOG_BASE/set_configure_pub.log" 2>&1
RET_PUB=$?
ros2 lifecycle set /lesson_06_lifecycle_subscriber configure > "$LOG_BASE/set_configure_sub.log" 2>&1
RET_SUB=$?
RET=$((RET_PUB + RET_SUB))
check_ret $RET

# --- 4. Activation ---
echo -n "[Step 4] Lifecycle Activate... "
ros2 lifecycle set /lesson_06_lifecycle_subscriber activate > "$LOG_BASE/set_activate_sub.log" 2>&1
RET_SUB=$?
ros2 lifecycle set /lesson_06_lifecycle_publisher activate > "$LOG_BASE/set_activate_pub.log" 2>&1
RET_PUB=$?
RET=$((RET_PUB + RET_SUB))
check_ret $RET

# --- 5. Data Flow Verification ---
echo -n "[Step 5] Data Flow Verification (Active)... "
# Try up to 3 times for data to start flowing
RET=1
for i in {1..3}; do
  MSG=$(ros2 topic echo "$TELEMETRY_TOPIC" --once --timeout 10 2>/dev/null)
  if [[ -n "$MSG" ]]; then RET=0; break; fi
  sleep 1
done
check_ret $RET

# --- 6. Deactivation ---
echo -n "[Step 6] Lifecycle Deactivate... "
ros2 lifecycle set /lesson_06_lifecycle_publisher deactivate > "$LOG_BASE/set_deactivate_pub.log" 2>&1
RET=$?
check_ret $RET

# --- 7. Inactive Verification ---
echo -n "[Step 7] Inactive Verification (No Data)... "
MSG=$(ros2 topic echo "$TELEMETRY_TOPIC" --once --timeout 5 2>/dev/null)
if [[ -z "$MSG" ]]; then RET=0; else RET=1; fi
check_ret $RET

# --- 8. Re-activation ---
echo -n "[Step 8] Lifecycle Re-activate... "
ros2 lifecycle set /lesson_06_lifecycle_publisher activate > "$LOG_BASE/set_reactivate_pub.log" 2>&1
RET=$?
check_ret $RET

# --- 9. Final Shutdown ---
echo -n "[Step 9] Lifecycle Shutdown... "
ros2 lifecycle set /lesson_06_lifecycle_publisher shutdown > "$LOG_BASE/set_shutdown_pub.log" 2>&1
RET_PUB=$?
ros2 lifecycle set /lesson_06_lifecycle_subscriber shutdown > "$LOG_BASE/set_shutdown_sub.log" 2>&1
RET_SUB=$?
RET_ORCH=$((RET_PUB + RET_SUB))
check_ret $RET_ORCH

# --- Cleanup ---
echo -n "Shutting down nodes... "
stop_node $PID_SUB
stop_node $PID_PUB
stop_node $PID_PRX_SUB
stop_node $PID_PRX_PUB
stop_node $PID_BRIDGE
echo "Done."

echo "=================================================="
echo "SUMMARY: roslibrust Lesson 06"
echo "=================================================="

# Post-Check
if ! assert_clean_graph; then
    echo -e "\033[0;31mCLEANUP VERIFICATION FAILED\033[0m"
    exit 1
fi

if [ $FAILED -eq 0 ] && [ $RET_ORCH -eq 0 ]; then
    echo -e "\033[0;32mALL TESTS PASSED\033[0m"
    exit 0
else
    echo -e "\033[0;31mTESTS FAILED\033[0m"
    exit 1
fi
