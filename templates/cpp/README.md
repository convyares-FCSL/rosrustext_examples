# C++ Template (`rclcpp`)

This is a minimal `ament_cmake` package template that enforces **header/source separation** and the **composition pattern**.

## Directory Structure

```text
.
├── include/lesson_cpp_template/
│   ├── lesson_node.hpp       # Node Class Definition
│   ├── qos.hpp               # QoS Configuration Constants
│   ├── services.hpp          # Service Name Constants
│   └── topics.hpp            # Topic Name Constants
├── src/
│   └── lesson_node.cpp       # Implementation + Main Entry Point
├── CMakeLists.txt
└── package.xml

```

## How to Use

1. **Copy the Template**:
Copy this folder into your `src/` directory (e.g., `src/2_cpp/lesson_03_my_lesson`).
2. **Rename the Package**:
* **`package.xml`**: Update `<name>` to your new package name.
* **`CMakeLists.txt`**: Update `project(lesson_cpp_template)` to your new package name.


3. **Update References**:
* **`include/`**: Rename the folder `include/lesson_cpp_template` to `include/your_package_name`.
* **`CMakeLists.txt`**: Update the `install(DIRECTORY include/ ...)` path if necessary.


4. **Edit the Node**:
* Define your node logic in `src/lesson_node.cpp`.
* Add shared constants to `topics.hpp`, `qos.hpp`, and `services.hpp` instead of hardcoding strings.
