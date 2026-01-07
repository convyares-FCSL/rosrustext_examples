#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# Lesson 01 Node: Timer + Logging
class Lesson01Node(Node):
    def __init__(self):
        super().__init__("lesson_01_node")

        # Parameter: timer_period_s (float seconds)
        self.declare_parameter("timer_period_s", 1.0)

        # Internal state
        self._tick = 0
        self._timer = None

        # Create timer using the current parameter value
        self._start_timer_from_param()

        self.get_logger().info("Lesson 01 node started (timer + logging). Ctrl+C to exit.")

    # Create or recreate the timer based on the current parameter value
    def _start_timer_from_param(self) -> None:
        period = float(self.get_parameter("timer_period_s").value)

        # Basic sanity clamp: prevent 0 or negative periods
        if period <= 0.0:
            self.get_logger().warn(f"timer_period_s={period} is invalid; using 1.0s")
            period = 1.0

        # If we ever recreate the timer, cancel the old one
        if self._timer is not None:
            self._timer.cancel()

        self._timer = self.create_timer(period, self._on_timer)
        self.get_logger().info(f"Timer running with period={period:.3f}s")

    # Timer callback: increment tick and log
    def _on_timer(self) -> None:
        self._tick += 1
        self.get_logger().info(f"tick {self._tick}")


# Main entry point
def main(args=None) -> None:
    rclpy.init(args=args)
    node = None

    try:
        node = Lesson01Node()
        rclpy.spin(node)

    # Handle Ctrl+C cleanly
    except KeyboardInterrupt:
        pass

    except Exception as exc:
        logger = rclpy.logging.get_logger("lesson_01_node")
        logger.error(f"Exception in main: {exc}")

    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()