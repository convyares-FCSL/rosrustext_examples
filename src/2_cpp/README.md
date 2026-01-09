# C++ Lessons (`rclcpp`)

These lessons use `rclcpp`, the standard C++ client library for ROS 2.
In this workspace, the C++ implementation is treated as the **Reference Implementation**. If behavior differs between languages, the C++ behavior is considered "correct" by definition of ROS 2 standards.

---

## Developer Workflow

### 1. Build Command
Unlike Python, C++ code must be compiled. Changes to `.cpp` or `.hpp` files require a rebuild.

```bash
cd ~/ros2_ws_tutorial
colcon build --packages-select lesson_00_bootstrap_cpp

```

### 2. Why use `--symlink-install` with C++?

Even though C++ binaries are compiled, we still recommend using `--symlink-install`:

```bash
colcon build --symlink-install --packages-select lesson_00_bootstrap_cpp

```

* **Binaries**: Must still be rebuilt to update.
* **Config Files**: Launch files (`.py`, `.xml`), parameters (`.yaml`), and interface definitions will update immediately without a rebuild. This saves time during the tuning phase.

### 3. Environment

Ensure your environment is sourced in your `~/.bashrc`:

```bash
source /opt/ros/jazzy/setup.bash
if [ -f ~/ros2_ws_tutorial/install/setup.bash ]; then
  source ~/ros2_ws_tutorial/install/setup.bash
fi

```

---

## Architectural Patterns

These lessons enforce strict C++ software engineering standards to ensure scalability.

### Header/Source Separation

We do not write inline classes in `.cpp` files.

* **Headers (`.hpp`)**: Define the interface.
* **Source (`.cpp`)**: Define the implementation.
* **Benefit**: Allows the node to be compiled as a shared library later (for "Composable Node" containers) without refactoring.

### Smart Pointers

We avoid raw pointers (`new`/`delete`).

* **`std::shared_ptr`**: Used for the Node and Timers (shared ownership with the Executor).
* **`std::unique_ptr`**: Used for internal resources that do not need to be shared.

### Robust Entry Point

The `main` function wraps execution in a `try-catch` block.

```cpp
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    // Prevents "Core Dump" on known errors like parameter validation failure
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Error: %s", e.what());
    return EXIT_FAILURE;
  }
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}

```

---

## Lessons

1. **Lesson 00**: Bootstrap (Header/Source split).
2. **Lesson 01**: Event Loop (Timers & `std::bind`).
3. **Lesson 02**: Publishers (Typed Interfaces & `utils_cpp`).