#!/bin/bash
set -e
SCRIPT_DIR=$(dirname "$0")
source "$SCRIPT_DIR/../env_setup.sh"
sanitize_environment


echo "--> Building rcllibrust (Bridge Rust) Lessons..."

echo "    Building lesson_01_node..."
cd src/3_rust/2_rcllibrust/lesson_01_node
cargo build
cd - > /dev/null

echo "    Building lesson_02_publisher..."
cd src/3_rust/2_rcllibrust/lesson_02_publisher
cargo build
cd - > /dev/null

echo "    Building lesson_03_subscriber..."
cd src/3_rust/2_rcllibrust/lesson_03_subscriber
cargo build
cd - > /dev/null

echo "rcllibrust Lessons built."
