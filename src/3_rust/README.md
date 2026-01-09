# Rust Lessons

This directory contains ROS 2 lessons implemented in Rust, split into two distinct tracks:

## 1. `1_rclrs` (Native Rust Node)

These lessons use [rclrs](https://github.com/ros2-rust/ros2_rust), the native Rust client library for ROS 2.
This is the **primary track** for writing standard ROS 2 nodes in Rust.

*   **Build System**: `colcon` (via `ament_cargo`) or `cargo`.
*   **Run**: `ros2 run <package> <executable>`
*   **Behavior**: Behaves like a C++ `rclcpp` or Python `rclpy` node.

### **Important: Custom Messages**

To use custom messages (from `lesson_interfaces`) in `rclrs`:

1.  **Generator**: The interface package MUST depend on `rosidl_generator_rs` in its `package.xml`.
2.  **Dependency**: Your Rust node must depend on the **generated crate** in the installation directory, not the source directory.

```toml
[dependencies]
lesson_interfaces = { path = "../../../../install/lesson_interfaces/share/lesson_interfaces/rust" }
```

## 2. `2_rcllibrust` (Client Library)

These lessons use `rcllibrust`, which is primarily a client/bridge library.

*   **Build System**: `cargo` ONLY.
*   **Ignored by Colcon**: These packages contain `COLCON_IGNORE` to prevent build failures in standard workflows.
*   **Run**: `cargo run --bin <name>`
