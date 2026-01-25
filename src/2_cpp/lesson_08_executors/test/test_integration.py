import time
import unittest

import pytest
import rclpy
import launch
import launch_testing.actions
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node as LaunchNode

from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import Transition

from rclpy.action import ActionClient
from lesson_interfaces.action import Fibonacci


@pytest.mark.rostest
def generate_test_description():
    dut_node = LaunchNode(
        package="lesson_08_executors_cpp",
        executable="lesson_08_actions_server",
        name="lesson_08_action_server",
        parameters=[{"timer_period_s": 0.1}],
        output="screen",
        emulate_tty=True,
        arguments=["--ros-args", "--log-level", "info"],
    )

    return LaunchDescription(
        [
            dut_node,
            TimerAction(period=2.0, actions=[launch_testing.actions.ReadyToTest()]),
        ]
    )


class TestActionsIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("integration_test_client")
        self.target_name = "/lesson_08_action_server"

    def tearDown(self):
        self.node.destroy_node()

    # -------------------------------------------------------------------------
    # Helpers
    # -------------------------------------------------------------------------

    def _spin_for(self, seconds, step=0.05):
        start = time.time()
        while (time.time() - start) < seconds:
            rclpy.spin_once(self.node, timeout_sec=step)

    def _get_state(self, timeout_sec=2.0):
        client = self.node.create_client(GetState, f"{self.target_name}/get_state")
        if not client.wait_for_service(timeout_sec=5.0):
            self.fail(f"Service {self.target_name}/get_state not available")

        req = GetState.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)

        if not future.done():
            self.fail(f"Timed out calling {self.target_name}/get_state")

        return future.result().current_state

    def _change_state(self, transition_id, timeout_sec=2.0):
        client = self.node.create_client(ChangeState, f"{self.target_name}/change_state")
        if not client.wait_for_service(timeout_sec=5.0):
            self.fail(f"Service {self.target_name}/change_state not available")

        req = ChangeState.Request()
        req.transition.id = transition_id

        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)

        if not future.done():
            self.fail(f"Timed out calling {self.target_name}/change_state")

        return future.result()

    def _configure_and_activate(self):
        state = self._get_state()
        self.assertEqual(state.label, "unconfigured", "Node must start unconfigured")

        res = self._change_state(Transition.TRANSITION_CONFIGURE)
        self.assertTrue(res.success, "Configure failed")
        state = self._get_state()
        self.assertEqual(state.label, "inactive", "State should be inactive after configure")

        res = self._change_state(Transition.TRANSITION_ACTIVATE)
        self.assertTrue(res.success, "Activate failed")
        state = self._get_state()
        self.assertEqual(state.label, "active", "State should be active after activate")

    def _deactivate_and_cleanup(self):
        res = self._change_state(Transition.TRANSITION_DEACTIVATE)
        self.assertTrue(res.success, "Deactivate failed")
        state = self._get_state()
        self.assertEqual(state.label, "inactive", "State should be inactive after deactivate")

        res = self._change_state(Transition.TRANSITION_CLEANUP)
        self.assertTrue(res.success, "Cleanup failed")
        state = self._get_state()
        self.assertEqual(state.label, "unconfigured", "State should be unconfigured after cleanup")

    def _action_client(self):
        return ActionClient(self.node, Fibonacci, "/tutorial/fibonacci")

    # -------------------------------------------------------------------------
    # Tests
    # -------------------------------------------------------------------------

    def test_lifecycle_sequence(self):
        state = self._get_state()
        self.assertEqual(state.label, "unconfigured", "Node must start unconfigured")

        res = self._change_state(Transition.TRANSITION_CONFIGURE)
        self.assertTrue(res.success, "Configure failed")
        state = self._get_state()
        self.assertEqual(state.label, "inactive", "State should be inactive after configure")

        res = self._change_state(Transition.TRANSITION_ACTIVATE)
        self.assertTrue(res.success, "Activate failed")
        state = self._get_state()
        self.assertEqual(state.label, "active", "State should be active after activate")

        res = self._change_state(Transition.TRANSITION_DEACTIVATE)
        self.assertTrue(res.success, "Deactivate failed")
        state = self._get_state()
        self.assertEqual(state.label, "inactive", "State should be inactive after deactivate")

        res = self._change_state(Transition.TRANSITION_CLEANUP)
        self.assertTrue(res.success, "Cleanup failed")
        state = self._get_state()
        self.assertEqual(state.label, "unconfigured", "State should be unconfigured after cleanup")

    def test_actions_success_cancel_and_recovery(self):
        self._configure_and_activate()

        client = self._action_client()
        self.assertTrue(client.wait_for_server(timeout_sec=10.0), "Action server not available")

        # ---- CASE 1: SUCCESS
        feedback_count = 0
        last_feedback = []

        def on_feedback(msg):
            nonlocal feedback_count, last_feedback
            feedback_count += 1
            last_feedback = list(msg.feedback.partial_sequence)

        goal = Fibonacci.Goal()
        goal.order = 5

        send_future = client.send_goal_async(goal, feedback_callback=on_feedback)
        rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=5.0)
        self.assertTrue(send_future.done(), "Timed out sending goal")

        goal_handle = send_future.result()
        self.assertTrue(goal_handle.accepted, "Goal was rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=15.0)
        self.assertTrue(result_future.done(), "Timed out waiting for result")

        wrapped = result_future.result()
        self.assertIsNotNone(wrapped, "No result wrapper returned")
        self.assertEqual(wrapped.status, 4, "Expected SUCCEEDED (status=4)")
        self.assertEqual(list(wrapped.result.sequence), [0, 1, 1, 2, 3])

        self.assertGreaterEqual(feedback_count, 1, "Expected feedback messages")
        self.assertGreaterEqual(len(last_feedback), 1, "Expected non-empty feedback sequence")

        # ---- CASE 2: CANCELLATION + AVAILABILITY RECOVERY TEST
        goal2 = Fibonacci.Goal()
        goal2.order = 10

        send_future2 = client.send_goal_async(goal2, feedback_callback=on_feedback)
        rclpy.spin_until_future_complete(self.node, send_future2, timeout_sec=5.0)
        self.assertTrue(send_future2.done(), "Timed out sending cancel-demo goal")

        goal_handle2 = send_future2.result()
        self.assertTrue(goal_handle2.accepted, "Cancel-demo goal was rejected")

        self._spin_for(2.0)

        get_state_client = self.node.create_client(GetState, f"{self.target_name}/get_state")
        self.assertTrue(get_state_client.wait_for_service(timeout_sec=5.0), "get_state not available")

        # --- RECOVERY CHECK ---
        # The service MUST respond immediately, even while the action is running/blocking a thread.
        # We loop to be sure, but even the first call should succeed quickly.
        observed_sluggish = False
        for _ in range(3):
            probe_future = get_state_client.call_async(GetState.Request())
            # Short timeout. If it times out, we are starved. Use 0.5s to be safe but should be instant.
            rclpy.spin_until_future_complete(self.node, probe_future, timeout_sec=0.5)
            if not probe_future.done():
                observed_sluggish = True
                break
            self._spin_for(0.1)

        self.assertFalse(
            observed_sluggish,
            "Service get_state TIMED OUT during action execution! Starvation persists.",
        )

        cancel_future = goal_handle2.cancel_goal_async()
        rclpy.spin_until_future_complete(self.node, cancel_future, timeout_sec=10.0)
        self.assertTrue(cancel_future.done(), "Timed out waiting for cancel response")

        cancel_resp = cancel_future.result()
        self.assertIsNotNone(cancel_resp, "No cancel response returned")

        result_future2 = goal_handle2.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future2, timeout_sec=20.0)
        self.assertTrue(result_future2.done(), "Timed out waiting for final result after cancel")

        wrapped2 = result_future2.result()
        self.assertIsNotNone(wrapped2, "No result wrapper returned for canceled goal")
        self.assertEqual(wrapped2.status, 5, "Expected CANCELED (status=5)")
        self.assertGreaterEqual(len(wrapped2.result.sequence), 1, "Expected partial sequence on cancel")
        self.assertLess(len(wrapped2.result.sequence), 10, "Expected sequence shorter than requested order")

        self._spin_for(0.5)
        state_after = self._get_state(timeout_sec=2.0)
        self.assertEqual(state_after.label, "active", "Node should remain active after cancel demo")

        self._deactivate_and_cleanup()
