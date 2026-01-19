
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
    # Target the Rust executable
    dut_node = LaunchNode(
        package='lesson_06_lifecycle_rclrs',
        executable='lesson_06_lifecycle_publisher',
        name='lesson_06_lifecycle_publisher',
        # Set a fast timer for testing
        parameters=[{'timer_period_s': 0.1}],
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'info'],
    )

    return LaunchDescription([
        dut_node,
        # Wait 2.0s for the node to spawn and DDS to settle
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
        # Create a hidden node to act as the Service Client
        self.node = rclpy.create_node('integration_test_client')
        self.target_name = '/lesson_06_lifecycle_publisher'

    def tearDown(self):
        self.node.destroy_node()

    def _get_state(self):
        """Helper to call /get_state service"""
        client = self.node.create_client(GetState, f'{self.target_name}/get_state')
        if not client.wait_for_service(timeout_sec=5.0):
            self.fail(f"Service {self.target_name}/get_state not available")
            
        req = GetState.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
        return future.result().current_state

    def _change_state(self, transition_id):
        """Helper to call /change_state service"""
        client = self.node.create_client(ChangeState, f'{self.target_name}/change_state')
        if not client.wait_for_service(timeout_sec=5.0):
            self.fail(f"Service {self.target_name}/change_state not available")
            
        req = ChangeState.Request()
        req.transition.id = transition_id
        
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
        return future.result()

    def test_lifecycle_sequence(self):
        """
        Verify the Full Lifecycle: Unconfigured -> Inactive -> Active -> Inactive -> Unconfigured
        """
        try:
            # 1. Check Initial State
            state = self._get_state()
            self.assertEqual(state.label.lower(), 'unconfigured', "Node must start Unconfigured")

            # 2. Configure (Unconfigured -> Inactive)
            res = self._change_state(Transition.TRANSITION_CONFIGURE)
            self.assertTrue(res.success, "Configure failed")
            
            state = self._get_state()
            self.assertEqual(state.label.lower(), 'inactive', "State should be Inactive after configure")

            # 3. Activate (Inactive -> Active)
            res = self._change_state(Transition.TRANSITION_ACTIVATE)
            self.assertTrue(res.success, "Activate failed")

            state = self._get_state()
            self.assertEqual(state.label.lower(), 'active', "State should be Active after activate")

            # 4. Deactivate (Active -> Inactive)
            res = self._change_state(Transition.TRANSITION_DEACTIVATE)
            self.assertTrue(res.success, "Deactivate failed")

            state = self._get_state()
            self.assertEqual(state.label.lower(), 'inactive', "State should be Inactive after deactivate")

            # 5. Cleanup (Inactive -> Unconfigured)
            res = self._change_state(Transition.TRANSITION_CLEANUP)
            self.assertTrue(res.success, "Cleanup failed")

            state = self._get_state()
            self.assertEqual(state.label.lower(), 'unconfigured', "State should be Unconfigured after cleanup")

            print("\n\033[92mINTEGRATION TEST PASS\033[0m")
        except Exception:
            print("\n\033[91mINTEGRATION TEST FAIL\033[0m")
            raise
