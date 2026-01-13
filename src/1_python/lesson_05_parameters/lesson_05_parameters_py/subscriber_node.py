#!/usr/bin/env python3
"""
Lesson 05 Subscriber Node.
Demonstrates Logic/Component/Node separation.
"""
from __future__ import annotations

from typing import List, Optional

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node

from lesson_interfaces.msg import MsgCount
from utils_py import qos, topics

from lesson_05_parameters_py.logic import TelemetryStreamValidator, StreamEvent


class TelemetrySubscriber:
    """
    Encapsulates subscription logic and resource ownership.
    """

    def __init__(self, node: Node) -> None:
        # 1. Instantiate Pure Logic
        self._logic = TelemetryStreamValidator()

        # 2. Setup ROS Resources
        topic = topics.telemetry(node)
        profile = qos.telemetry(node)
        
        self._sub = node.create_subscription(MsgCount, topic, self._on_message, profile)
        self._logger = node.get_logger()
        self._logger.info(f"TelemetrySubscriber listening on '{topic}'")

    def update_config(self, reset_max_value: int) -> None:
        """Push config to logic."""
        self._logic.set_reset_max_value(reset_max_value)

    def _on_message(self, msg: MsgCount) -> None:
        """Translate ROS Msg -> Logic Decision -> Side Effect."""
        decision = self._logic.on_count(msg.count)

        if decision.event == StreamEvent.RESET:
            self._logger.warn(decision.message)
        elif decision.event == StreamEvent.OUT_OF_ORDER:
            self._logger.error(decision.message)
        else:
            self._logger.info(decision.message)


class Lesson05SubscriberNode(Node):
    """
    Manages parameters and component lifecycle.
    """

    def __init__(self) -> None:
        super().__init__("lesson_05_subscriber")

        # 1. Setup
        self._declare_parameters()
        self._telemetry = TelemetrySubscriber(self)

        # 2. Initial Config
        initial_val = self.get_parameter("subscriber.reset_max_value").value
        self._telemetry.update_config(int(initial_val))

        # 3. Callbacks
        self.add_on_set_parameters_callback(self._on_param_update)

    def _declare_parameters(self) -> None:
        self.declare_parameter("subscriber.reset_max_value", 10)

    def _on_param_update(self, parameters: List[rclpy.Parameter]) -> SetParametersResult:
        for p in parameters:
            if p.name == "subscriber.reset_max_value":
                try:
                    self._telemetry.update_config(int(p.value))
                    self.get_logger().info(f"Updated reset tolerance to {p.value}")
                except ValueError:
                    return SetParametersResult(successful=False)
        
        return SetParametersResult(successful=True)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[Lesson05SubscriberNode] = None

    try:
        node = Lesson05SubscriberNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        logger = rclpy.logging.get_logger("lesson_05_subscriber")
        logger.error(f"Unhandled exception: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()