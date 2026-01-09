#!/bin/bash
set -e

SCRIPT_DIR=$(dirname "$0")
source "$SCRIPT_DIR/env_setup.sh"
sanitize_environment



echo "=========================================="
echo " 1. Building Interfaces"
echo "=========================================="
"$SCRIPT_DIR/01_setup/build_interfaces.sh" || { echo -e "\n\033[0;31mInterfaces Build : FAILED\033[0m"; exit 1; }

echo -e "\n------------------------------------------"
echo -e "\033[0;32mInterfaces Build : SUCCESS\033[0m"
echo -e "------------------------------------------\n"

echo "=========================================="
echo " 2. Building Bootstrap & Utils (All Languages)"
echo "=========================================="
"$SCRIPT_DIR/02_bootstrap/build_python.sh"    || { echo -e "\n\033[0;31mPython Bootstrap : FAILED\033[0m"; exit 1; }
"$SCRIPT_DIR/02_bootstrap/build_cpp.sh"       || { echo -e "\n\033[0;31mC++ Bootstrap : FAILED\033[0m"; exit 1; }
"$SCRIPT_DIR/02_bootstrap/build_rclrs.sh"     || { echo -e "\n\033[0;31mrclrs Bootstrap : FAILED\033[0m"; exit 1; }
"$SCRIPT_DIR/02_bootstrap/build_rcllibrust.sh" || { echo -e "\n\033[0;31mrcllibrust Bootstrap : FAILED\033[0m"; exit 1; }

echo -e "\n------------------------------------------"
echo -e "\033[0;32mPython Bootstrap    : SUCCESS\033[0m"
echo -e "\033[0;32mC++ Bootstrap       : SUCCESS\033[0m"
echo -e "\033[0;32mrclrs Bootstrap     : SUCCESS\033[0m"
echo -e "\033[0;32mrcllibrust Bootstrap: SUCCESS\033[0m"
echo -e "------------------------------------------\n"

echo "=========================================="
echo " 3. Building All Lessons (All Languages)"
echo "=========================================="
"$SCRIPT_DIR/03_lessons/build_python.sh"       || { echo -e "\n\033[0;31mPython Lessons : FAILED\033[0m"; exit 1; }
"$SCRIPT_DIR/03_lessons/build_cpp.sh"          || { echo -e "\n\033[0;31mC++ Lessons : FAILED\033[0m"; exit 1; }
"$SCRIPT_DIR/03_lessons/build_rclrs.sh"        || { echo -e "\n\033[0;31mrclrs Lessons : FAILED\033[0m"; exit 1; }
"$SCRIPT_DIR/03_lessons/build_rcllibrust.sh"   || { echo -e "\n\033[0;31mrcllibrust Lessons : FAILED\033[0m"; exit 1; }

echo -e "\n------------------------------------------"
echo -e "\033[0;32mPython Lessons      : SUCCESS\033[0m"
echo -e "\033[0;32mC++ Lessons         : SUCCESS\033[0m"
echo -e "\033[0;32mrclrs Lessons       : SUCCESS\033[0m"
echo -e "\033[0;32mrcllibrust Lessons  : SUCCESS\033[0m"
echo -e "------------------------------------------\n"

echo "=========================================="
echo -e "\033[0;32m WORKSPACE BUILD COMPLETE \033[0m"
echo "=========================================="
