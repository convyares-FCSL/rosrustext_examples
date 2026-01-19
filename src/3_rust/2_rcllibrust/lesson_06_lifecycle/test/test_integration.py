
import os
import time
import unittest
import rclpy
import pytest
import launch
import launch_ros.actions
import launch_testing.actions
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node as LaunchNode
from ament_index_python.packages import get_package_share_directory

from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import Transition


# Launch Description Generation
@pytest.mark.rostest
def generate_test_description():
    # Start Rosbridge
    rosbridge_launch_dir = os.path.join(
        get_package_share_directory('rosbridge_server'), 'launch')
    rosbridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
             os.path.join(rosbridge_launch_dir, 'rosbridge_websocket_launch.xml')
        )
    )

    # Start Lifecycle Proxy
    from launch.actions import ExecuteProcess
    proxy_proc = ExecuteProcess(
        cmd=['rosrustext_lifecycle_proxy', 
             '--rosbridge-url', 'ws://localhost:9090',
             '--target-node', 'lesson_06_lifecycle_publisher'],
        output='screen'
    )

    # Target the Rust executable (Publisher)
    dut_node = LaunchNode(
        package='lesson_06_lifecycle_roslibrust',
        executable='lesson_06_lifecycle_publisher',
        name='lesson_06_lifecycle_publisher',
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'info'],
    )

    return LaunchDescription([
        rosbridge_launch,
        proxy_proc,
        dut_node,
        # Wait for rosbridge -> proxy -> node connection and discovery
        TimerAction(period=5.0, actions=[launch_testing.actions.ReadyToTest()]),
    ])

# Test Client Definition
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

        except Exception as e:
            self.fail(f"Lifecycle sequence failed: {e}")
