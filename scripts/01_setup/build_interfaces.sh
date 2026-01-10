#!/bin/bash
set -e
SCRIPT_DIR=$(dirname "$0")
# SCRIPT_DIR is scripts/01_setup, so we need to go up one level to find env_setup.sh
source "$SCRIPT_DIR/../env_setup.sh"
sanitize_environment

echo "--> Cleaning previous interface builds to force regeneration..."
rm -rf "$SCRIPT_DIR/../../build/lesson_interfaces" "$SCRIPT_DIR/../../install/lesson_interfaces"
rm -rf "$SCRIPT_DIR/../../build/rosidl_generator_rs" "$SCRIPT_DIR/../../install/rosidl_generator_rs"

echo "--> Building Interfaces..."
colcon build --packages-select rosidl_generator_rs builtin_interfaces service_msgs action_msgs unique_identifier_msgs lesson_interfaces

echo "--> Validating generated Cargo.toml..."
CARGO_TOML="$SCRIPT_DIR/../../install/lesson_interfaces/share/lesson_interfaces/rust/Cargo.toml"
if [ -f "$CARGO_TOML" ]; then
    if grep -q 'path =' "$CARGO_TOML"; then
         echo "Validation Success: Found path dependencies."
         grep 'path =' "$CARGO_TOML"
    else
         echo "Validation Failed: No path dependencies found (result might be using crates.io '*')."
         cat "$CARGO_TOML"
         exit 1
    fi
else
    echo "Validation Failed: Generated Cargo.toml not found!"
    exit 1
fi
echo "Interfaces built."
