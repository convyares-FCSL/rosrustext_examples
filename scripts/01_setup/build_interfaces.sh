#!/bin/bash
set -e
SCRIPT_DIR=$(dirname "$0")
# SCRIPT_DIR is scripts/01_setup, so we need to go up one level to find env_setup.sh
source "$SCRIPT_DIR/../env_setup.sh"
sanitize_environment

ROS_DISTRO="${ROS_DISTRO:-humble}"
INTERFACES_DIR=$(realpath "$SCRIPT_DIR/../../src/4_interfaces")
RCL_OVERLAY_DIR="$INTERFACES_DIR/rcl_interfaces"
RCL_REMOTE_URL="https://github.com/ros2/rcl_interfaces.git"

select_rcl_branch() {
    local distro="$1"
    if git ls-remote --heads "$RCL_REMOTE_URL" "$distro" >/dev/null 2>&1; then
        echo "$distro"
    elif git ls-remote --heads "$RCL_REMOTE_URL" main >/dev/null 2>&1; then
        echo "main"
    elif git ls-remote --heads "$RCL_REMOTE_URL" master >/dev/null 2>&1; then
        echo "master"
    else
        echo ""
    fi
}

prepare_rcl_overlay() {
    local branch="$1"
    echo "--> Ensuring rcl_interfaces overlay matches ROS_DISTRO='$ROS_DISTRO' (branch='$branch')"
    if [ -d "$RCL_OVERLAY_DIR/.git" ]; then
        git -C "$RCL_OVERLAY_DIR" fetch --depth=1 origin "$branch" >/dev/null 2>&1 || true
        if git -C "$RCL_OVERLAY_DIR" rev-parse --verify "$branch" >/dev/null 2>&1; then
            git -C "$RCL_OVERLAY_DIR" checkout "$branch"
        else
            git -C "$RCL_OVERLAY_DIR" checkout -B "$branch" "origin/$branch"
        fi
        git -C "$RCL_OVERLAY_DIR" reset --hard "origin/$branch"
    elif [ -d "$RCL_OVERLAY_DIR" ]; then
        echo "--> rcl_interfaces overlay already exists without git metadata; assuming the vendored copy matches '$branch'."
    else
        echo "--> Cloning rcl_interfaces overlay (branch '$branch')."
        git clone --depth=1 --branch "$branch" "$RCL_REMOTE_URL" "$RCL_OVERLAY_DIR"
    fi
}

RCL_BRANCH=$(select_rcl_branch "$ROS_DISTRO")
if [ -z "$RCL_BRANCH" ]; then
    echo "Failed to determine a branch for $RCL_REMOTE_URL; checked ROS_DISTRO='$ROS_DISTRO', main, and master."
    exit 1
fi

prepare_rcl_overlay "$RCL_BRANCH"

echo "--> Cleaning previous interface builds to force regeneration..."
rm -rf "$SCRIPT_DIR/../../build/lesson_interfaces" "$SCRIPT_DIR/../../install/lesson_interfaces"
rm -rf "$SCRIPT_DIR/../../build/rcl_interfaces" "$SCRIPT_DIR/../../install/rcl_interfaces"
rm -rf "$SCRIPT_DIR/../../build/rosidl_generator_rs" "$SCRIPT_DIR/../../install/rosidl_generator_rs"

INTERFACE_PACKAGES=(
    rosidl_generator_rs
    builtin_interfaces
    service_msgs
    action_msgs
    unique_identifier_msgs
    rcl_interfaces
    lesson_interfaces
)

echo "--> Building Interfaces (${INTERFACE_PACKAGES[*]})"
colcon build --packages-select "${INTERFACE_PACKAGES[@]}"

validate_generated_cargo() {
    local label="$1"
    local cargo_path="$2"
    local hint="$3"

    if [ ! -f "$cargo_path" ]; then
        echo "Validation Failed: $label Cargo.toml missing at $cargo_path"
        echo "Hint: $hint"
        exit 1
    fi

    if grep -q 'path =' "$cargo_path"; then
        echo "Validation Success: $label crate references path dependencies."
        grep 'path =' "$cargo_path"
    else
        echo "Validation Failed: $label Cargo.toml does not declare path dependencies (might be resolving crates.io)."
        echo "Hint: $hint"
        cat "$cargo_path"
        exit 1
    fi
}

# We overlay the rcl_interfaces core package so ROS 2 clients link against locally generated Rust crates
# instead of crates.io, hence we must have the generated Cargo.toml ready for Cargo to resolve path deps.
validate_generated_cargo \
    "lesson_interfaces" \
    "$SCRIPT_DIR/../../install/lesson_interfaces/share/lesson_interfaces/rust/Cargo.toml" \
    "Ensure lesson_interfaces/package.xml depends on rosidl_generator_rs and that the package list includes lesson_interfaces."

validate_generated_cargo \
    "rcl_interfaces" \
    "$SCRIPT_DIR/../../install/rcl_interfaces/share/rcl_interfaces/rust/Cargo.toml" \
    "Ensure rcl_interfaces/package.xml adds rosidl_generator_rs, the overlay was built (colcon includes rcl_interfaces), and the overlay branch matches ROS_DISTRO."

echo "Interfaces built."
