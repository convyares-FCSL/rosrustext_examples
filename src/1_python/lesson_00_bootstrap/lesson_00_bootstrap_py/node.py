#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# Define a simple ROS2 node
class BootstrapNode(Node):
    def __init__(self):
        # Initialize the node with the name "lesson_00_bootstrap"
        super().__init__("lesson_00_bootstrap")

        # Log an informational message indicating the node has started
        self.get_logger().info("Lesson 00 bootstrap node started...")

# Main function to initialize and run the node
def main(args=None):
    # Initialize the ROS2 Python client library with command-line arguments
    rclpy.init(args=args)
    
    # Initialize the node variable
    node = None

    # Use a try-except-finally block to manage node lifecycle and exceptions
    try:
        # Create the node
        node = BootstrapNode()

        # `spin_once()` blocks indefinitely by default; use a 0 timeout so this lesson exits.
        rclpy.spin_once(node, timeout_sec=0.0)

    except Exception as exc:
        # Log any exceptions that occur
        logger = rclpy.logging.get_logger("lesson_00_bootstrap")
        logger.error(f"Exception in main: {exc}")

    finally:
        # Clean up and shutdown
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

# Run the main function when this script is executed
if __name__ == "__main__":
    main()
