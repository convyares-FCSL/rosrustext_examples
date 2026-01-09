#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from typing import Optional

# Helpers for shared configuration (uncomment when needed)
# from lesson_py_template import qos, topics, services

# << FILL IN HERE >>: Define the default node name
NODE_NAME = "template_node"

class MyNode(Node):
    def __init__(self):
        super().__init__(NODE_NAME)

        # 1. Setup Parameters
        # We declare and read parameters first so we can use them for initialization.
        self._init_parameters()

        # 2. Setup State
        # << FILL IN HERE >>: Initialize member variables (business logic state)
        # self._count: int = 0

        # 3. Setup Resources
        # << FILL IN HERE >>: Create Publishers and Subscribers using shared helpers
        # self._pub = self.create_publisher(MsgType, topics.INPUT_TOPIC, qos.default_qos)

        # << FILL IN HERE >>: Create Timers
        # self._timer = self.create_timer(self._period_s, self._on_timer)

        self.get_logger().info(f"Node '{NODE_NAME}' initialized.")

    def _init_parameters(self) -> None:
        """
        Declares ROS parameters and sets local member variables.
        """
        # << FILL IN HERE >>: Declare parameters with default values
        # self.declare_parameter("timer_period_s", 1.0)

        # << FILL IN HERE >>: Read parameters into member variables
        # self._period_s = self.get_parameter("timer_period_s").value
        pass

    # << FILL IN HERE >>: Define Callbacks
    # def _on_timer(self) -> None:
    #     self._count += 1
    #     self.get_logger().info(f"Tick {self._count}")


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = None
    
    try:
        node = MyNode()
        # Run the event loop (blocking)
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass # Clean exit on Ctrl+C
        
    except Exception as exc:
        # Log fatal errors explicitly
        logger = node.get_logger() if node else rclpy.logging.get_logger(NODE_NAME)
        logger.error(f"Fatal error: {exc}")

    finally:
        # Proper resource cleanup
        if node:
            node.destroy_node