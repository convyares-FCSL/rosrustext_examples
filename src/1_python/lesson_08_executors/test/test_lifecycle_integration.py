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

# 1. DEFINE THE SYSTEM UNDER TEST
@pytest.mark.rostest
def generate_test_description():
    dut_node = LaunchNode(
        package='lesson_08_executors',
        executable='lesson_08_executors',
        name='lesson_08_executors',
        parameters=[{'timer_period_s': 0.1}],
        # CRITICAL FIX 1: Force output to screen to catch ImportErrors
        output='screen',
        # CRITICAL FIX 2: Emulate TTY to prevent buffering delays
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'info'],
    )

    return LaunchDescription([
        dut_node,
        # CRITICAL FIX 3: Wait 2.0s before starting the test logic
        # This ensures the node process is fully spawned and DDS is ready.
        TimerAction(period=2.0, actions=[launch_testing.actions.ReadyToTest()]),
    ])

# 2. DEFINE THE TEST CLIENT
class TestLifecycleIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('lifecycle_integration_tester')
        self.client_get_state = self.node.create_client(GetState, '/lesson_08_executors/get_state')
        self.client_change_state = self.node.create_client(ChangeState, '/lesson_08_executors/change_state')

    def tearDown(self):
        self.node.destroy_node()

    def _wait_for_service(self, client):
        # 10s timeout to accomodate slow CI environments
        self.assertTrue(client.wait_for_service(timeout_sec=10.0), f"Lifecycle service {client.srv_name} not available")

    def _get_state(self):
        self._wait_for_service(self.client_get_state)
        request = GetState.Request()
        future = self.client_get_state.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        return future.result().current_state

    def _change_state(self, transition_id):
        self._wait_for_service(self.client_change_state)
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = self.client_change_state.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        return future.result()

    def test_lifecycle_sequence(self):
        """
        Verify the Full Lifecycle: Unconfigured -> Inactive -> Active -> Inactive -> Unconfigured
        """
        # 1. Check Initial State
        state = self._get_state()
        self.assertEqual(state.label, 'unconfigured', "Node must start Unconfigured")

        # 2. Configure (Unconfigured -> Inactive)
        res = self._change_state(Transition.TRANSITION_CONFIGURE)
        self.assertTrue(res.success, "Configure failed")
        
        state = self._get_state()
        self.assertEqual(state.label, 'inactive', "State should be Inactive after configure")

        # 3. Activate (Inactive -> Active)
        res = self._change_state(Transition.TRANSITION_ACTIVATE)
        self.assertTrue(res.success, "Activate failed")

        state = self._get_state()
        self.assertEqual(state.label, 'active', "State should be Active after activate")

        # 4. Deactivate (Active -> Inactive)
        res = self._change_state(Transition.TRANSITION_DEACTIVATE)
        self.assertTrue(res.success, "Deactivate failed")

        state = self._get_state()
        self.assertEqual(state.label, 'inactive', "State should be Inactive after deactivate")

        # 5. Cleanup (Inactive -> Unconfigured)
        res = self._change_state(Transition.TRANSITION_CLEANUP)
        self.assertTrue(res.success, "Cleanup failed")

        state = self._get_state()
        self.assertEqual(state.label, 'unconfigured', "State should be Unconfigured after cleanup")