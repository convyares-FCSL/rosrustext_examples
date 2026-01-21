"""
Action Server Telemetry Publisher.

Provides lifecycle-gated telemetry data using a timer.
"""
from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.callback_groups import CallbackGroup
from rclpy.timer import Timer

from lesson_interfaces.msg import MsgCount
from utils_py import qos, topics, LifecycleNode
from .. import logic

class TelemetryPublisher:
    """
    Lifecycle-aware Telemetry Component.
    
    Implements manual data gating to emulate LifecyclePublisher behavior
    using standard ROS 2 publishers and timers.
    """

    def __init__(self) -> None:
        self._logic = logic.TelemetryGenerator()
        self._handle: Optional[rclpy.publisher.Publisher] = None 
        self._timer: Optional[Timer] = None
        self._topic_name = ""
        self._logger = None # NEW: Store logger reference
        self._is_enabled = False

    def configure(self, parent_node: LifecycleNode, callback_group: CallbackGroup, period: float) -> None:
        """Create the publisher resources and timer (Inactive state)."""
        self._logger = parent_node.get_logger()
        self._topic_name = topics.telemetry(parent_node)
        qos_profile = qos.telemetry(parent_node)
        
        self._handle = parent_node.create_publisher(
            MsgCount, 
            self._topic_name, 
            qos_profile=qos_profile
        )
        
        # Create timer using the specified callback group
        self._timer = parent_node.create_timer(
            period, 
            self._on_timer, 
            callback_group=callback_group
        )
        
        parent_node.get_logger().info(f"Publisher configured on '{self._topic_name}' with period {period}s")

    def activate(self) -> None:
        """Enable data flow (Active state)."""
        self._is_enabled = True
        if self._logger:
            self._logger.info("Telemetry Publisher Activated")

    def deactivate(self) -> None:
        """Disable data flow (Inactive state)."""
        self._is_enabled = False
        if self._logger:
            self._logger.info("Telemetry Publisher Deactivated")

    def cleanup(self, parent_node: LifecycleNode) -> None:
        """Destroy resources (Unconfigured state)."""
        if self._handle:
            parent_node.destroy_publisher(self._handle)
            self._handle = None
        
        if self._timer:
            parent_node.destroy_timer(self._timer)
            self._timer = None
            
        self._is_enabled = False

    def set_period(self, val: float) -> None:
        """Update timer period."""
        if self._timer:
            self._timer.timer_period_ns = int(val * 1e9)

    def _on_timer(self) -> None:
        """Produce and send data (if Active)."""
        if not self._is_enabled or self._handle is None:
            return

        val = self._logic.next_value()
        msg = MsgCount()
        msg.count = val
        self._handle.publish(msg)
