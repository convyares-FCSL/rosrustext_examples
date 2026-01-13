#!/usr/bin/env python3
"""
Lesson 05 Publisher Node.
Demonstrates component composition and centralized configuration.
"""
from __future__ import annotations

from typing import List, Optional

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node

from lesson_interfaces.msg import MsgCount
from utils_py import qos, topics

from lesson_05_parameters_py.utils import validate_timer_period_s
from lesson_05_parameters_py.logic import TelemetryGenerator


class TelemetryPublisher:
    """
    Encapsulates publishing logic and resource ownership.
    """

    def __init__(self, node: Node) -> None:
        # 1. Instantiate Pure Logic
        self._logic = TelemetryGenerator()

        # 2. Setup ROS Resources
        topic_name = topics.telemetry(node)
        qos_profile = qos.telemetry(node)

        self._handle = node.create_publisher(MsgCount, topic_name, qos_profile)
        node.get_logger().info(f"TelemetryPublisher initialized on '{topic_name}'")

    def publish(self) -> None:
        """Construct and send the next message."""
        val = self._logic.next_value()
        
        msg = MsgCount()
        msg.count = val
        self._handle.publish(msg)


class Lesson05PublisherNode(Node):
    """
    Manages the application lifecycle, parameters, and execution scheduling.
    """

    def __init__(self) -> None:
        super().__init__("lesson_05_publisher")

        self._timer = None
        self._timer_period_s = 1.0

        # 1. Setup
        self._declare_parameters()
        self._telemetry = TelemetryPublisher(self)

        # 2. Execution (Start Timer)
        initial_period = self.get_parameter("timer_period_s").value
        self._create_or_update_timer(initial_period)

        # 3. Callbacks
        self.add_on_set_parameters_callback(self._on_param_update)

        self.get_logger().info("Lesson 05 publisher started.")

    def _declare_parameters(self) -> None:
        self.declare_parameter("timer_period_s", 1.0)

    def _on_param_update(self, parameters: List[rclpy.Parameter]) -> SetParametersResult:
        for parameter in parameters:
            if parameter.name == "timer_period_s":
                validation = validate_timer_period_s(parameter.value)
                if not validation.ok:
                     return SetParametersResult(successful=False, reason=validation.reason)
                
                self._create_or_update_timer(validation.value)

        return SetParametersResult(successful=True)

    def _create_or_update_timer(self, period_s: float) -> None:
        """
        Idempotent timer creation. Only destroys/recreates if period changes.
        """
        # Optimization: No-op if frequency matches
        if self._timer is not None and self._timer_period_s == period_s:
            return

        if self._timer is not None:
            self._timer.cancel()
            self._timer.destroy()

        self._timer_period_s = period_s
        self._timer = self.create_timer(self._timer_period_s, self._telemetry.publish)
        
        self.get_logger().info(f"Timer set to {self._timer_period_s}s")


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[Lesson05PublisherNode] = None

    try:
        node = Lesson05PublisherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        logger = rclpy.logging.get_logger("lesson_05_publisher")
        logger.error(f"Unhandled exception: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()