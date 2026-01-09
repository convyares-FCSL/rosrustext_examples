#!/usr/bin/env python3
import rclpy
from typing import List, Optional
from rclpy.node import Node

# Local imports
from utils_py import topics, qos
from lesson_interfaces.msg import MsgCount

# Lesson 02 Node: Publisher
class Lesson02Node(Node):
    # Constructor
    def __init__(self):
        super().__init__("lesson_02_node")

        # 1. Parameter: timer_period_s (float seconds)
        self.declare_parameter("timer_period_s", 1.0)

        # 2. Create Publisher
        self._create_publisher()

        # 3. Create Timer
        self._create_timer_from_param()

        # 4. Log
        self.get_logger().info(f"Lesson 02 node started. Publishing to '{self.topic_name}' every {self.period}s")

    # Helper to create publisher
    def _create_publisher(self) -> None:
        # Load topic name 
        self.topic_name = topics.chatter(self)

        # Load QoS profile 
        self.qos_profile = qos.telemetry(self)

        # Create publisher (msg_type, topic_name, qos_profile)
        self._publisher = self.create_publisher(MsgCount, self.topic_name, self.qos_profile)

    # Helper to create/reset the timer if the parameter changes
    def _create_timer_from_param(self) -> None:
        self._count = 0
        self._timer = None

        self.period = float(self.get_parameter("timer_period_s").value)
        if self.period <= 0.0:          
            self.get_logger().warn(f"timer_period_s={self.period} is invalid; using 1.0s")
            self.period = 1.0
        
        if self._timer is not None:
            self._timer.cancel()

        self._timer = self.create_timer(self.period, self._tick)

    # Timer callback: Publish message and increment count
    def _tick(self) -> None:
        # Publish message
        msg = MsgCount()
        msg.count = self._count
        self._publisher.publish(msg)

        # Increment tick counter
        self._count += 1

# Main entry point
def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = None

    try:
        node = Lesson02Node()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as exc:
        logger = rclpy.logging.get_logger("lesson_02_node")
        logger.error(f"Exception in main: {exc}")

    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

# Entry point
if __name__ == "__main__":
    main()