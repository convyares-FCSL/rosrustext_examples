"""
Action Server Component (Python).

Implements the Fibonacci action server logic and lifecycle hooks.
"""
from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import CallbackGroup
from rclpy.lifecycle import LifecycleNode

from lesson_interfaces.action import Fibonacci
from ..logic import FibonacciGenerator
from utils_py import topics
import time

class ActionServerComponent:
    """
    Action Server Component.
    """

    def __init__(self) -> None:
        self._action_server: Optional[ActionServer] = None
        self._node: Optional[LifecycleNode] = None
        self._generator = FibonacciGenerator()
        self._enabled = False

    def configure(self, parent_node: LifecycleNode, callback_group: CallbackGroup) -> None:
        """Create resources."""
        self._node = parent_node
        # WARNING: This implementation uses a blocking execution model on the
        # main thread. Long-running goals will block other node operations
        # (like parameters and lifecycle) unless a MultiThreadedExecutor is used.
        self._action_server = ActionServer(
            parent_node,
            Fibonacci,
            topics.fibonacci(parent_node),
            self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=callback_group
        )
        action_name = topics.fibonacci(parent_node)
        parent_node.get_logger().info(f"Action Server configured on '{action_name}'")

    def activate(self) -> None:
        """Enable processing."""
        self._enabled = True
        if self._node:
            self._node.get_logger().info("Action Server Activated")

    def deactivate(self) -> None:
        """Disable processing."""
        self._enabled = False
        if self._node:
            self._node.get_logger().info("Action Server Deactivated")

    def cleanup(self, parent_node: LifecycleNode) -> None:
        """Destroy resources."""
        if self._action_server:
            self._action_server.destroy()
            self._action_server = None
        self._enabled = False

    def _goal_callback(self, goal_request):
        """Accept or reject goal."""
        if not self._enabled:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        """Accept or reject cancel."""
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        """Execute the goal."""
        goal_id = goal_handle.goal_id.uuid
        self._node.get_logger().info(f'Executing goal {goal_id}...')
        
        order = goal_handle.request.order
        feedback_msg = Fibonacci.Feedback()
        sequence = []
        
        # Simulate long running task
        for i in range(order):
            if not self._enabled:
                # Abort if deactivated mid-run (optional robustness)
                self._node.get_logger().warn(f'Goal {goal_id} aborted: Node deactivated')
                goal_handle.abort()
                return Fibonacci.Result(sequence=sequence)

            if goal_handle.is_cancel_requested:
                self._node.get_logger().info(f'Goal {goal_id} canceled')
                goal_handle.canceled()
                return Fibonacci.Result(sequence=sequence)

            time.sleep(1.0) 
            
            val = self._generator.step(sequence)
            sequence.append(val)
            
            feedback_msg.partial_sequence = sequence
            goal_handle.publish_feedback(feedback_msg)
            self._node.get_logger().info(f'Feedback for {goal_id}: {sequence}')

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = sequence
        self._node.get_logger().info(f'Goal {goal_id} succeeded')
        return result
        