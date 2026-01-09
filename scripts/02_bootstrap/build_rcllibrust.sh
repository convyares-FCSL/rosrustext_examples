#!/bin/bash
set -e
SCRIPT_DIR=$(dirname "$0")
source "$SCRIPT_DIR/../env_setup.sh"
sanitize_environment


echo "--> Building rcllibrust (Bridge Rust) Bootstrap..."

echo "    Building utils_roslibrust..."
cd src/3_rust/2_rcllibrust/utils_roslibrust
cargo build
cd - > /dev/null

echo "    Building lesson_00_bootstrap..."
cd src/3_rust/2_rcllibrust/lesson_00_bootstrap
cargo build
cd - > /dev/null

echo "rcllibrust Bootstrap built."
