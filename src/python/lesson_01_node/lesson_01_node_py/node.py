#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class LessonNode(Node):
    def __init__(self):
        super().__init__("lesson_01_node")
        self.declare_parameter("timer_period_s", 0.5)
        period = self.get_parameter("timer_period_s").get_parameter_value().double_value

        self._count = 0
        self._timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(f"Lesson 01 node started (period: {period:.2f}s).")

    def _on_timer(self):
        self.get_logger().info(f"Tick {self._count}")
        self._count += 1


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LessonNode()
        rclpy.spin(node)
    except Exception as exc:
        logger = rclpy.logging.get_logger("lesson_01_node")
        logger.error(f"Exception in main: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
