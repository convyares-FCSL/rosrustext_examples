#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from utils_py import qos
from utils_py import topics


class PublisherNode(Node):
    def __init__(self):
        super().__init__("lesson_02_publisher")
        self.declare_parameter("message_prefix", "Hello")
        self.declare_parameter("publish_period_s", 0.5)

        self._prefix = self.get_parameter("message_prefix").get_parameter_value().string_value
        period = self.get_parameter("publish_period_s").get_parameter_value().double_value

        self._count = 0
        self._topic = topics.from_params(self, "chatter", "/tutorial/chatter")
        self._publisher = self.create_publisher(
            String, 
            self._topic, 
            qos.from_parameters(self)
        )
        self._timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"Lesson 02 publisher started (topic: {self._topic}, period: {period:.2f}s)."
        )

    def _on_timer(self):
        msg = String()
        msg.data = f"{self._prefix} {self._count}"
        self._publisher.publish(msg)
        self._count += 1


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PublisherNode()
        rclpy.spin(node)
    except Exception as exc:
        logger = rclpy.logging.get_logger("lesson_02_publisher")
        logger.error(f"Exception in main: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
