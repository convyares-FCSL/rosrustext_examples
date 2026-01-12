#!/usr/bin/env python3
from __future__ import annotations

from typing import List, Optional

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node

from lesson_interfaces.msg import MsgCount
from utils_py import qos, topics, _get_or_declare

from lesson_05_parameters_py.utils import validate_timer_period_s

class TelemetryPublisherCore:
    """
    Pure publishing behaviour (no ROS resources).
    Owns counter state and produces the next message payload.
    """

    __slots__ = ("_count",)

    def __init__(self) -> None:
        self._count = 0

    def next_message(self) -> MsgCount:
        message = MsgCount()
        message.count = self._count
        self._count += 1
        return message


class Lesson05PublisherNode(Node):
    """
    ROS wrapper:
      - reads configuration (topic/QoS via workspace utils)
      - manages timer lifecycle
      - supports runtime parameter updates
    """

    def __init__(self) -> None:
        super().__init__("lesson_05_publisher")

        self._publisher = None
        self._timer = None
        self._timer_period_s = 1.0
        self._topic_name = ""

        self._core = TelemetryPublisherCore()

        self._declare_parameters()
        self._setup_publisher()
        self._apply_initial_configuration()
        self._setup_parameter_callbacks()

        self.get_logger().info(
            f"Lesson 05 publisher started. Publishing to '{self._topic_name}' every {self._timer_period_s}s"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("timer_period_s", 1.0)

    def _setup_publisher(self) -> None:
        self._topic_name = topics.telemetry(self)
        qos_profile = qos.telemetry(self)
        self._publisher = self.create_publisher(MsgCount, self._topic_name, qos_profile)

    def _apply_initial_configuration(self) -> None:
        raw_period = self.get_parameter("timer_period_s").value
        validation = validate_timer_period_s(raw_period)

        if not validation.ok:
            self.get_logger().warn(f"Invalid timer_period_s='{raw_period}', using 1.0s")
            self._create_or_update_timer(1.0)
            return

        self._create_or_update_timer(validation.value)

    def _setup_parameter_callbacks(self) -> None:
        self.add_on_set_parameters_callback(self._on_parameters)

    def _on_parameters(self, parameters) -> SetParametersResult:
        for parameter in parameters:
            if parameter.name != "timer_period_s":
                continue

            validation = validate_timer_period_s(parameter.value)
            if not validation.ok:
                return SetParametersResult(successful=False, reason=validation.reason)

            self._create_or_update_timer(validation.value)
            self.get_logger().info(f"Updated timer_period_s -> {self._timer_period_s}s")

        return SetParametersResult(successful=True)

    def _create_or_update_timer(self, timer_period_s: float) -> None: 
        self._timer_period_s = float(timer_period_s)

        if self._timer is not None:
            self._timer.cancel()

        self._timer = self.create_timer(self._timer_period_s, self._tick)

    def _tick(self) -> None:
        message = self._core.next_message()
        self._publisher.publish(message)


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
        logger.error(f"Exception in main: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
