"""
Action Server Telemetry Publisher.

Provides lifecycle-gated telemetry data.
"""

from typing import Optional

import rclpy
from lesson_interfaces.msg import MsgCount
from utils_py import qos, topics, LifecycleNode
from ..logic import TelemetryGenerator


class TelemetryPublisher:
    """
    Lifecycle-aware Telemetry Component.
    
    Implements manual data gating to emulate LifecyclePublisher behavior
    using standard ROS 2 publishers.
    """

    def __init__(self) -> None:
        self._logic = TelemetryGenerator()
        # Use standard Publisher, not LifecyclePublisher
        self._handle: Optional[rclpy.publisher.Publisher] = None 
        self._topic_name = ""
        # The Manual Gate
        self._is_enabled = False

    def configure(self, parent_node: LifecycleNode) -> None:
        """Create the publisher resources (Inactive state)."""
        self._topic_name = topics.telemetry(parent_node)
        qos_profile = qos.telemetry(parent_node)
        
        # FIX: Use standard create_publisher
        self._handle = parent_node.create_publisher(
            MsgCount, 
            self._topic_name, 
            qos_profile=qos_profile
        )
        parent_node.get_logger().info(f"Publisher configured on '{self._topic_name}'")

    def activate(self) -> None:
        """Enable data flow (Active state)."""
        self._is_enabled = True
        # Standard publishers don't have .on_activate(), we just flip the flag.

    def deactivate(self) -> None:
        """Disable data flow (Inactive state)."""
        self._is_enabled = False

    def cleanup(self, parent_node: LifecycleNode) -> None:
        """Destroy resources (Unconfigured state)."""
        if self._handle:
            parent_node.destroy_publisher(self._handle)
            self._handle = None
        self._is_enabled = False

    def publish(self) -> None:
        """Produce and send data (if Active)."""
        # 1. Check the Manual Gate
        if not self._is_enabled or self._handle is None:
            return

        # 2. Generate and Publish
        val = self._logic.next_value()
        msg = MsgCount()
        msg.count = val
        self._handle.publish(msg)