import os
import time
import unittest
import rclpy
import pytest
import launch
import launch_ros.actions
import launch_testing.actions
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node as LaunchNode

from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import Transition
from lesson_interfaces.action import Fibonacci
from rclpy.action import ActionClient

# 1. DEFINE THE SYSTEM UNDER TEST
@pytest.mark.rostest
def generate_test_description():
    dut_node = LaunchNode(
        package='lesson_08_executors',
        executable='lesson_08_executors',
        name='lesson_08_executors',
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'info'],
    )

    return LaunchDescription([
        dut_node,
        TimerAction(period=2.0, actions=[launch_testing.actions.ReadyToTest()]),
    ])

# 2. DEFINE THE TEST CLIENT
class TestActionBlocking(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('action_blocking_tester')
        self.client_get_state = self.node.create_client(GetState, '/lesson_08_executors/get_state')
        self.client_change_state = self.node.create_client(ChangeState, '/lesson_08_executors/change_state')
        self._action_client = ActionClient(self.node, Fibonacci, '/lesson_08_executors/fibonacci')

    def tearDown(self):
        self.node.destroy_node()

    def _change_state(self, transition_id):
        self.assertTrue(self.client_change_state.wait_for_service(timeout_sec=5.0))
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = self.client_change_state.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        res = future.result()
        self.assertTrue(res.success, f"State transition {transition_id} failed!")
        return res

    def test_blocking_behavior(self):
        """
        Verify that executing an action DOES NOT BLOCK the lifecycle services.
        """
        # 1. Configure and Activate
        self._change_state(Transition.TRANSITION_CONFIGURE)
        self._change_state(Transition.TRANSITION_ACTIVATE)

        # 2. Send Long-Running Goal
        self.assertTrue(self._action_client.wait_for_server(timeout_sec=5.0), "Action Server not found")
        goal_msg = Fibonacci.Goal()
        goal_msg.order = 5  # 5 seconds of blocking
        
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, send_goal_future)
        goal_handle = send_goal_future.result()
        self.assertTrue(goal_handle.accepted)

        # 3. IMMEDIATELY try to get state
        # The Action Server is now busy sleeping in the loop.
        self.assertTrue(self.client_get_state.wait_for_service(timeout_sec=1.0))
        request = GetState.Request()
        
        future = self.client_get_state.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)
        
        if future.done():
            # SUCCESS: The service responded immediately because the action is running 
            # on a separate callback group/thread.
            self.assertTrue(future.result().current_state, "GetState should return valid state")
        else:
            self.fail("Lifecycle GetState service timed out! The node IS blocking, but it should NOT be.")
            
        # 4. Cleanup: Wait for action to finish so we tear down cleanly
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
