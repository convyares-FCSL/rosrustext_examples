# Interfaces and Tools

This directory contains shared ROS 2 interfaces and the tools required to generate bindings for them.

## Packages

- **`lesson_interfaces`**: Common messages, services, and actions for the tutorial.
- **`rosidl_rust`**: Contains `rosidl_generator_rs`, the tool required to generate Rust definitions from `.msg` files.
    - Note: This generator config is **patched** by `scripts/01_setup/build_interfaces.sh` to pin dependencies `builtin_interfaces` and `service_msgs` to local dummy packages to avoid yanked/missing crates on crates.io.
- **`src/4_interfaces/vendor_dummy`**: Contains minimal dummy crates (`builtin_interfaces`, `service_msgs`) used to satisfy local build dependencies for `lesson_interfaces`.

## Usage in Rust

To use `lesson_interfaces` in a Rust node:

1.  **Enable Generator**: Ensure `lesson_interfaces/package.xml` has:
    ```xml
    <build_depend>rosidl_generator_rs</build_depend>
    ```
2.  **Dependencies**: In your node's `Cargo.toml`, point to the generated crate:
    ```toml
    [dependencies]
    lesson_interfaces = { path = "../../../../install/lesson_interfaces/share/lesson_interfaces/rust" }
    ```
