# Lesson 00 Breakdown: The Python Node Pattern

## Architecture: The Class-Based Node

In ROS 2 Python (`rclpy`), the standard design pattern is to create a class that inherits from `Node`. This allows your specific logic (timers, publishers, etc.) to live inside `self`, keeping the global namespace clean.



1.  **Inheritance**: `class BootstrapNode(Node)` gives your class all the standard ROS 2 capabilities (logging, time, parameters).
2.  **Encapsulation**: All node-specific data is stored in `self`.
3.  **Lifecycle**: The `main` function handles the "boilerplate" of starting and stopping the system safely.

## Code Walkthrough

### 1. The Class Definition
```python
class BootstrapNode(Node):
    def __init__(self):
        # Initialize the node with the name "lesson_00_bootstrap"
        super().__init__("lesson_00_bootstrap")

        # Log an informational message
        self.get_logger().info("Lesson 00 bootstrap node started...")

```

* **`super().__init__(name)`**: This is mandatory. It registers your node with the ROS 2 core. The string name you pass here is how the node will appear in tools like `ros2 node list`.
* **`self.get_logger()`**: This accesses the built-in ROS logger. Unlike `print()`, this output includes timestamps, the node name, and severity levels (INFO, WARN, ERROR), and it is published to `/rosout`.

### 2. The Main Entry Point (Deep Dive)

The `main` function is designed to be robust. It ensures that even if the node crashes, ROS cleans up resources properly.

```python
def main(args=None):
    # 1. Initialize ROS 2 Middleware
    # This sets up the communication layer (DDS).
    rclpy.init(args=args)
    
    node = None

    # 2. The Execution Block
    try:
        node = BootstrapNode()

        # 3. The Spin
        # 'spin' means "process events". 
        # Usually, we use rclpy.spin(node) to run forever.
        # For this bootstrap lesson, we run ONCE with 0 timeout to exit immediately.
        rclpy.spin_once(node, timeout_sec=0.0)

    except Exception as exc:
        # 4. Error Handling
        # If construction fails, we catch it here.
        # We use a static logger because 'node' might be None if creation failed.
        logger = rclpy.logging.get_logger("lesson_00_bootstrap")
        logger.error(f"Exception in main: {exc}")

    finally:
        # 5. Cleanup (Crucial!)
        # This block runs whether the code succeeded OR failed.
        if node is not None:
            node.destroy_node() # Frees internal handles
        
        if rclpy.ok():
            rclpy.shutdown()    # Shuts down the DDS context

```

**Why the `try-finally` block?**

* **Zombie Prevention**: If a script crashes without calling `shutdown()`, the node might remain listed in ROS 2 discovery, confusing other tools until you reboot the terminal.
* **Resource Leaks**: In complex nodes (e.g., those using cameras or serial ports), the `finally` block is where you ensure hardware connections are closed.

### 3. The Script Guard

```python
if __name__ == "__main__":
    main()

```

* This standard Python idiom ensures `main()` only runs if you execute the file directly. It allows other scripts to `import` this file (e.g., for unit testing) without accidentally starting the node.