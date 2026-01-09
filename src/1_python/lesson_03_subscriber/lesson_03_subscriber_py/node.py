#!/usr/bin/env python3
import rclpy
from typing import List, Optional
from rclpy.node import Node

from utils_py import topics, qos
from lesson_interfaces.msg import MsgCount


class ChatterListener:
    """Pure message-handling logic (no ROS resources)."""

    __slots__ = ("_logger", "_expected", "_initialized", "_reset_max_value")

    def __init__(self, logger, reset_max_value: int = 1):
        # If the stream counter ever drops to <= reset_max_value, treat it as a publisher reset.
        # This makes the node robust to:
        # - restarting a publisher
        # - manual CLI injections (e.g., publishing count:=1 after count:=100)
        self._logger = logger
        self._expected = 0
        self._initialized = False
        self._reset_max_value = int(reset_max_value)

    def _init_from(self, msg_count: int, initial: bool = False, reset: bool = False) -> None:
        self._expected = int(msg_count) + 1
        self._initialized = True
        if initial:
            self._logger.info(f"Received (initial): {msg_count}")
        elif reset:
            self._logger.warn(f"Detected counter reset. Re-syncing at: {msg_count}")
        else:
            self._logger.info(f"Received: {msg_count}")

    def __call__(self, msg: MsgCount) -> None:
        c = int(msg.count)

        # Late joiners: first observed message defines the baseline.
        if not self._initialized:
            self._init_from(c, initial=True)
            return

        # Reset detection: publisher restart / manual injection.
        # If we've already been running and suddenly see a very small count, treat as reset.
        if c <= self._reset_max_value and c < self._expected:
            self._init_from(c, reset=True)
            return

        # Out-of-order / stale detection.
        if c < self._expected:
            self._logger.warn(f"Out-of-order/invalid: {c} < {self._expected}")
            return

        self._logger.info(f"Received: {c}")
        self._expected = c + 1


class Lesson03Node(Node):
    """ROS resource container (owns subscriptions, timers, parameters)."""

    def __init__(self) -> None:
        super().__init__("lesson_03_node")

        # Configuration is externalized (YAML via utils_py), not hardcoded.
        self.topic_name = topics.chatter(self)
        self.qos_profile = qos.telemetry(self)

        # Logic is injected; the ROS callback is just a router.
        self._listener = ChatterListener(self.get_logger(), reset_max_value=1)

        self._subscriber = self.create_subscription(
            MsgCount,
            self.topic_name,
            self._listener,
            self.qos_profile,
        )

        self.get_logger().info(
            f"Lesson 03 node started. Subscribing to '{self.topic_name}'"
        )


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[Lesson03Node] = None

    try:
        node = Lesson03Node()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as exc:
        logger = rclpy.logging.get_logger("lesson_03_node")
        logger.error(f"Exception in main: {exc}")

    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
