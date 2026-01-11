#!/usr/bin/env python3
from __future__ import annotations

from typing import List, Optional

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node

from lesson_interfaces.msg import MsgCount
from utils_py import qos, topics

from lesson_05_parameters_py.logic import StreamEvent, TelemetryStreamValidator
from lesson_05_parameters_py.utils import validate_reset_max_value


class TelemetryListener:
    """
    ROS callback adapter: converts ROS message into domain-level validation decisions.
    """

    __slots__ = ("_logger", "_validator")

    def __init__(self, logger, *, reset_max_value: int = 1) -> None:
        self._logger = logger
        self._validator = TelemetryStreamValidator(reset_max_value=reset_max_value)

    def set_reset_max_value(self, value: int) -> None:
        self._validator.set_reset_max_value(value)

    def __call__(self, msg: MsgCount) -> None:
        decision = self._validator.on_count(int(msg.count))

        if decision.event == StreamEvent.RESET:
            self._logger.warn(decision.message)
            return

        if decision.event == StreamEvent.OUT_OF_ORDER:
            self._logger.warn(decision.message)
            return

        self._logger.info(decision.message)


class Lesson05SubscriberNode(Node):
    def __init__(self) -> None:
        super().__init__("lesson_05_subscriber")

        self._topic_name = ""
        self._subscription = None

        self._declare_parameters()
        self._setup_subscription()
        self._setup_parameter_callbacks()

        self.get_logger().info(f"Lesson 05 subscriber started. Subscribing to '{self._topic_name}'")

    def _declare_parameters(self) -> None:
        self.declare_parameter("reset_max_value", 1)

    def _setup_subscription(self) -> None:
        self._topic_name = topics.telemetry(self)
        qos_profile = qos.telemetry(self)

        raw_reset_max = self.get_parameter("reset_max_value").value
        validation = validate_reset_max_value(raw_reset_max)
        reset_max_value = validation.value if validation.ok else 1

        self._listener = TelemetryListener(self.get_logger(), reset_max_value=reset_max_value)

        self._subscription = self.create_subscription(
            MsgCount,
            self._topic_name,
            self._listener,
            qos_profile,
        )

    def _setup_parameter_callbacks(self) -> None:
        self.add_on_set_parameters_callback(self._on_parameters)

    def _on_parameters(self, parameters) -> SetParametersResult:
        for parameter in parameters:
            if parameter.name != "reset_max_value":
                continue

            validation = validate_reset_max_value(parameter.value)
            if not validation.ok:
                return SetParametersResult(successful=False, reason=validation.reason)

            self._listener.set_reset_max_value(validation.value)
            self.get_logger().info(f"Updated reset_max_value -> {validation.value}")

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
        logger.error(f"Exception in main: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
