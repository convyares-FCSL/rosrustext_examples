#!/bin/bash
set +e
source "$(dirname "$0")/../env_setup.sh"
source "$(dirname "$0")/../../scripts/05_utils/test_helpers.sh"

sanitize_environment
# Source Workspace
FILES="$(dirname "$0")/../../install/setup.bash"
if [ -f "$FILES" ]; then source "$FILES"; else echo "Error: install/setup.bash not found. Build first."; exit 1; fi

LOG_BASE="log/tests/python"
mkdir -p "$LOG_BASE"
rm -f "$LOG_BASE"/*.log
export PYTHONUNBUFFERED=1

setup_traps
ensure_clean_start

echo "=================================================="
echo "TEST SUITE: Python (Lessons 1-6)"
echo "=================================================="

# Helper for Pass/Fail
check_ret() {
    if [ $1 -eq 0 ]; then echo -e "  \033[0;32mPASS\033[0m"; else echo -e "  \033[0;31mFAIL (See log)\033[0m"; return 1; fi
}

# Helper: Wait for a service to appear
wait_for_service() {
    local service=$1
    local attempts=10
    for ((i=0; i<attempts; i++)); do
        if ros2 service list | grep -q "$service"; then
             return 0
        fi
        sleep 1
    done
    return 1
}

# Helper: Wait for a node to start (via param service)
wait_for_node_params() {
    local node=$1
    local attempts=10
    for ((i=0; i<attempts; i++)); do
        if ros2 service list | grep -q "$node/get_parameters"; then
             return 0
        fi
        sleep 1
    done
    return 1
}

# Helper: Wait for a topic to appear
wait_for_topic() {
    local topic=$1
    local attempts=10
    for ((i=0; i<attempts; i++)); do
        if ros2 topic list | grep -q "$topic"; then
             return 0
        fi
        sleep 1
    done
    return 1
}


# --- Lesson 01: Node ---
echo -n "[L01] Node Startup... "
start_node_bg "$LOG_BASE/l01.log" ros2 run lesson_01_node_py lesson_01_node
PID=$RET_PID
sleep 2
stop_node $PID
check_ret 0
R1=$?

# --- Lesson 02: Publisher ---
echo -n "[L02] Publisher Topic... "
start_node_bg "$LOG_BASE/l02.log" ros2 run lesson_02_publisher_py lesson_02_publisher
PID=$RET_PID
sleep 2
if ros2 topic list | grep -q "/tutorial/chatter"; then RET=0; else RET=1; fi
stop_node $PID
check_ret $RET
R2=$?

# --- Lesson 03: Subscriber ---
echo -n "[L03] Subscriber Data... "
RET=1
# Start Sub
start_node_bg "$LOG_BASE/l03.log" ros2 run lesson_03_subscriber_py lesson_03_subscriber
PID_SUB=$RET_PID

# Wait for subscription
for ((i=0; i<10; i++)); do
    if grep -q "Subscribing" "$LOG_BASE/l03.log"; then
         break
    fi
    sleep 1
done

# Inject Message (Mirroring Manual Workflow)
for ((i=0; i<3; i++)); do
    ros2 topic pub /tutorial/chatter lesson_interfaces/msg/MsgCount "{count: 1}" --once > /dev/null 2>&1
    sleep 0.5
done

# Verify Receipt
for ((i=0; i<10; i++)); do
    if grep -q "Received" "$LOG_BASE/l03.log"; then
         RET=0
         break
    fi
    sleep 1
done

stop_node $PID_SUB
check_ret $RET
R3=$?

# --- Lesson 04: Service ---
echo -n "[L04] Service Logic... "
start_node_bg "$LOG_BASE/l04.log" ros2 run lesson_04_service_py service_server
PID_SRV=$RET_PID
RESP=""
if wait_for_service "/tutorial/compute_stats"; then
    for ((i=0; i<5; i++)); do
        RESP=$(timeout 20 ros2 service call /tutorial/compute_stats lesson_interfaces/srv/ComputeStats "{data: [1.0, 2.0, 3.0]}" 2>"$LOG_BASE/l04_client.log")
        if [[ "$RESP" == *"sum=6"* ]]; then break; fi
        sleep 1
    done
else
    RESP="Timeout"
fi
stop_node $PID_SRV
if [[ "$RESP" == *"sum=6"* ]]; then RET=0; else RET=1; fi
check_ret $RET
R4=$?

# --- Lesson 05: Parameters ---
echo -n "[L05] Parameter Read... "
start_node_bg "$LOG_BASE/l05.log" ros2 run lesson_05_parameters_py lesson_05_publisher
PID=$RET_PID
VAL=""
if wait_for_node_params "/lesson_05_publisher"; then
    for ((i=0; i<5; i++)); do
        VAL=$(ros2 param get /lesson_05_publisher timer_period_s 2>"$LOG_BASE/l05_client.log" || echo "Fail")
        if [[ "$VAL" == *"1.0"* ]]; then break; fi
        sleep 1
    done
else
    VAL="Timeout"
fi
stop_node $PID
if [[ "$VAL" == *"1.0"* ]]; then RET=0; else RET=1; fi
check_ret $RET
R5=$?

# --- Lesson 06: Orchestration ---
echo -n "[L06] Lifecycle Orch... "
start_node_bg "$LOG_BASE/l06_pub.log" ros2 run lesson_06_lifecycle_py lesson_06_lifecycle_publisher
PID_PUB=$RET_PID
start_node_bg "$LOG_BASE/l06_sub.log" ros2 run lesson_06_lifecycle_py lesson_06_lifecycle_subscriber
PID_SUB=$RET_PID
start_node_bg "$LOG_BASE/l06_mgr.log" ros2 run lesson_06_orchestration lifecycle_manager
PID_MGR=$RET_PID

# Wait for Active
sleep 5
STATE_PUB=$(ros2 lifecycle get /lesson_06_lifecycle_publisher 2>/dev/null | tr '[:upper:]' '[:lower:]' || echo "unknown")
STATE_SUB=$(ros2 lifecycle get /lesson_06_lifecycle_subscriber 2>/dev/null | tr '[:upper:]' '[:lower:]' || echo "unknown")

if [[ "$STATE_PUB" == *"active"* && "$STATE_SUB" == *"active"* ]]; then
    # Clean Shutdown Check
    pkill -INT -P $PID_MGR 2>/dev/null
    kill -INT $PID_MGR 2>/dev/null
    wait $PID_MGR 2>/dev/null || true
    sleep 2
    if grep -q "context is invalid" "$LOG_BASE/l06_mgr.log"; then RET=1; else RET=0; fi
else
    RET=1
fi

# Cleanup
stop_node $PID_PUB
stop_node $PID_SUB
stop_node $PID_MGR

check_ret $RET
R6=$?

echo "=================================================="
echo "SUMMARY: Python"
echo "=================================================="

# Post-Check
if ! assert_clean_graph; then
    echo -e "\033[0;31mCLEANUP VERIFICATION FAILED\033[0m"
    exit 1
fi

FAILURES=$((R1 + R2 + R3 + R4 + R5 + R6))
if [ $FAILURES -eq 0 ]; then
    echo -e "\033[0;32mALL TESTS PASSED\033[0m"
    exit 0
else
    echo -e "\033[0;31m$FAILURES TESTS FAILED\033[0m"
    exit 1
fi
