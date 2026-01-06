#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class BootstrapNode(Node):
    def __init__(self):
        super().__init__("lesson_00_bootstrap")
        self.get_logger().info("Lesson 00 bootstrap node started.")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BootstrapNode()
        rclpy.spin(node)
    except Exception as exc:
        logger = rclpy.logging.get_logger("lesson_00_bootstrap")
        logger.error(f"Exception in main: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
