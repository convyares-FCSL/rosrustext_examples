# Interfaces and Tools

This directory contains shared ROS 2 interfaces and the tools required to generate bindings for them.

## Packages

- **`lesson_interfaces`**: Common messages, services, and actions for the tutorial.
- **`rosidl_rust`**: Contains `rosidl_generator_rs`, the tool required to generate Rust definitions from `.msg` files.
    - Note: This generator config is modified to use relative path dependencies.
- **`rcl_interfaces`** & **`unique_identifier_msgs`**: Local clones of core ROS 2 interface packages. These are built locally with `rosidl_generator_rs` enabled to provide necessary Rust dependencies (`builtin_interfaces`, `service_msgs`, etc.) that are missing from the system installation.

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
