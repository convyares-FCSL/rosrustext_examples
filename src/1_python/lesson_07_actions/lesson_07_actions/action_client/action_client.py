"""
Fibonacci Action Client (Python).

Interacts with the Fibonacci Action Server to send goals and monitor progress.
"""
from __future__ import annotations

import time
from typing import List, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from utils_py import topics

from lesson_interfaces.action import Fibonacci

class ActionClientNode(Node):
    """
    Action Client Node (Demo).
    
    Demonstrates sending goals, handling feedback, and requesting cancellation.
    """

    def __init__(self) -> None:
        super().__init__("lesson_07_action_client")
        action_name = topics.fibonacci(self)
        self.get_logger().info(f"Connecting to action server: {action_name}")
        self._action_client = ActionClient(self, Fibonacci, action_name)

    def send_goal(self, order: int) -> None:
        self.get_logger().info(f"Waiting for action server 'fibonacci'...")
        if not self._action_client.wait_for_server(timeout_sec=60.0):
            self.get_logger().error("Action server not available")
            return



        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self.get_logger().info(f"Sending goal: order={order}")

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        # We can spin until goal accepted
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted!')

        # Wait for result
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)

        result = get_result_future.result().result
        status = get_result_future.result().status
        self.get_logger().info(f'Result Received: {result.sequence} (Status: {status})')
        
    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Received feedback: {feedback_msg.feedback.partial_sequence}')

    def demo_cancel(self):
        """Send goal, wait, then cancel."""
        self.get_logger().info("--- Starting Cancellation Demo ---")
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Server not available for cancel demo")
            return

        goal_msg = Fibonacci.Goal()
        goal_msg.order = 10
        
        self.get_logger().info("Sending goal (to be canceled)...")
        send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected, cannot demonstrate cancel")
            return

        self.get_logger().info("Goal accepted. Waiting 3 seconds allowing some feedback...")
        
        # We need to spin a bit to process feedback while waiting
        # In this synchronous client demo, we just iterate spin_once
        start_time = time.time()
        while time.time() - start_time < 3.0:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info("Canceling goal...")
        cancel_future = goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future)
        
        cancel_response = cancel_future.result()
        if len(cancel_response.goals_canceling) > 0:
             self.get_logger().info("Cancel request accepted")
        else:
             self.get_logger().warn("Cancel request did not return any cancelling goals (already done?)")

        # Wait for final result (should be CANCELED)
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        
        # Status 5 is CANCELED, 6 is ABORTED, 4 is SUCCEEDED
        # rclpy.action.client.GoalStatus isn't always easy to import, result.status works
        status = get_result_future.result().status
        result = get_result_future.result().result
        self.get_logger().info(f"Final Result after cancel: Sequence={result.sequence}, Status={status}")


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    
    node = ActionClientNode()
    
    # 1. Standard Success Case
    print("\n=== CASE 1: SUCCESS ===")
    node.send_goal(5)
    
    # 2. Cancellation Case
    print("\n=== CASE 2: CANCELLATION ===")
    node.demo_cancel()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()