#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from utils_py import qos
from utils_py import topics


class SubscriberNode(Node):
    def __init__(self):
        super().__init__("lesson_03_subscriber")
        self._topic = topics.from_params(self, "chatter", "/tutorial/chatter")
        self._subscription = self.create_subscription(
            String, 
            self._topic, 
            self._on_message, 
            qos.from_parameters(self)
        )
        self.get_logger().info(f"Lesson 03 subscriber started (topic: {self._topic}).")

    def _on_message(self, msg: String):
        self.get_logger().info(f"Heard: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SubscriberNode()
        rclpy.spin(node)
    except Exception as exc:
        logger = rclpy.logging.get_logger("lesson_03_subscriber")
        logger.error(f"Exception in main: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
