#!/bin/bash
set -e
SCRIPT_DIR=$(dirname "$0")
# SCRIPT_DIR is scripts/01_setup, so we need to go up one level to find env_setup.sh
source "$SCRIPT_DIR/../env_setup.sh"
sanitize_environment

echo "--> Patching rosidl_generator_rs template to pin dependencies..."
TEMPLATE_FILE="$SCRIPT_DIR/../../src/4_interfaces/rosidl_rust/rosidl_generator_rs/resource/Cargo.toml.em"

# Use python inside the template to conditionally pin versions
# Original: @(dep) = "*"
# Goal: @(dep) = @("0.0.0" if dep in ["builtin_interfaces", "service_msgs"] else "*")
# sed logic removed as template was manually patched
# sed -i "s/@(dep) = \"*\"/@(dep) = @('\"0.0.0\"' if dep in ['builtin_interfaces', 'service_msgs'] else '\"*\"')/" "$TEMPLATE_FILE"

echo "--> Cleaning previous interface builds to force regeneration..."
rm -rf "$SCRIPT_DIR/../../build/lesson_interfaces" "$SCRIPT_DIR/../../install/lesson_interfaces"
rm -rf "$SCRIPT_DIR/../../build/rosidl_generator_rs" "$SCRIPT_DIR/../../install/rosidl_generator_rs"

echo "--> Building Interfaces..."
colcon build --packages-select rosidl_generator_rs lesson_interfaces

echo "--> Validating generated Cargo.toml..."
CARGO_TOML="$SCRIPT_DIR/../../install/lesson_interfaces/share/lesson_interfaces/rust/Cargo.toml"
if [ -f "$CARGO_TOML" ]; then
        # Validation for path dependencies or pinned versions
        # We expect paths now, so just ensuring the lines exist for these deps is enough, 
        # or check they are NOT "*"
        if grep -E '(builtin_interfaces|service_msgs) = "\*"' "$CARGO_TOML"; then
             echo "Validation Failed: Found unpinned dependencies!"
             grep -E '(builtin_interfaces|service_msgs)' "$CARGO_TOML"
             exit 1
        fi
        echo "Validation Success: Dependencies pinned/pathed."
        grep -E '(builtin_interfaces|service_msgs)' "$CARGO_TOML"
else
    echo "Validation Failed: Generated Cargo.toml not found!"
    exit 1
fi
echo "Interfaces built."
