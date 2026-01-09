# Python Lessons (`rclpy`)

These lessons use `rclpy`, the standard Python client library for ROS 2.
Python nodes in this workspace prioritize **readability**, **orchestration**, and **rapid iteration**.

---

## Developer Workflow

### 1. Build with Symlinks (Recommended)
When developing Python packages, always use the `--symlink-install` flag. This creates a symbolic link from the install directory back to your source code.
* **Benefit**: Edits to `.py` files take effect immediately. You do **not** need to rebuild the workspace to see changes.

```bash
cd ~/ros2_ws_tutorial
colcon build --symlink-install --packages-select lesson_00_bootstrap_py
```

### 2. Automatic Sourcing

To avoid running `source` commands in every new terminal, add the following to your shell configuration (`~/.bashrc` or `~/.zshrc`).

**Note**: Adjust the path if your workspace is named differently.

```bash
# Source the ROS 2 system installation
source /opt/ros/jazzy/setup.bash

# Source the local workspace overlay (if built)
if [ -f ~/ros2_ws_tutorial/install/setup.bash ]; then
  source ~/ros2_ws_tutorial/install/setup.bash
fi
```

---

## Architectural Patterns

While Python allows for simple scripts, these lessons enforce **Production Patterns** to match the C++ and Rust tracks.

### The Class-Based Node

We avoid global variables. All nodes inherit from `rclpy.node.Node`.

```python
class MyNode(Node):
    def __init__(self):
        super().__init__("my_node_name")
        # Logic goes here
```

### The `main` Entry Point

We use a robust `try-except-finally` block to ensure resources (like DDS handles) are cleaned up, preventing "zombie nodes" that require a system reboot to clear.

```python
def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MyNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
```

---

## Lessons
1. **Lesson 00**: Bootstrap & Lifecycle  
   The `main` pattern and clean shutdown semantics.

2. **Lesson 01**: Parameters & Timers  
   Event-loop execution and safe state mutation.

3. **Lesson 02**: Publishers & Custom Interfaces  
   Composition, shared configuration (`utils_py`), and message definitions.

4. **Lesson 03**: Subscribers & System Verification  
   QoS compatibility, cross-language validation, logic injection for callbacks, and stream reset tolerance.
