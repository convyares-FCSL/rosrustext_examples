#!/usr/bin/env python3
"""
Lesson 06 Lifecycle Publisher (Python).

Demonstrates a Managed Node where the Node orchestrates the component's state.
"""
from __future__ import annotations

from typing import List, Optional

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.lifecycle import TransitionCallbackReturn
from lifecycle_msgs.msg import State
from rclpy.timer import Timer

from lesson_interfaces.msg import MsgCount
from utils_py import qos, topics, LifecycleNode

from lesson_05_parameters_py.utils import validate_timer_period_s
from lesson_05_parameters_py.logic import TelemetryGenerator


class TelemetryPublisher:
    """
    Lifecycle-aware Component.
    
    NOTE: Because we are using a Python Shim for LifecycleNode, we cannot use
    'create_lifecycle_publisher'. We must use a standard publisher and 
    manually gate the data flow, just like the Subscriber.
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


class LifecyclePublisherNode(LifecycleNode):
    """
    The Lifecycle Authority.
    """

    def __init__(self) -> None:
        super().__init__("lesson_06_lifecycle_publisher")
        
        self._telemetry = TelemetryPublisher()
        self._timer: Optional[Timer] = None
        self._timer_period_s = 1.0

        self.get_logger().info("Node initialized (Unconfigured). Waiting for manager...")

    # --- Lifecycle State Machine Callbacks ---

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Transition: Unconfigured -> Inactive"""
        self.get_logger().info(f"Configuring from {state.label}...")
        
        try:
            self._declare_parameters()
            self.add_on_set_parameters_callback(self._on_param_update)

            self._telemetry.configure(self)
            
            initial_period = self.get_parameter("timer_period_s").value
            self._create_timer_paused(initial_period)
            
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Configuration failed: {e}")
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Transition: Inactive -> Active"""
        self.get_logger().info(f"Activating from {state.label}...")
        
        self._telemetry.activate()
        if self._timer:
            self._timer.reset()
            
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Transition: Active -> Inactive"""
        self.get_logger().info(f"Deactivating from {state.label}...")
        
        self._telemetry.deactivate()
        if self._timer:
            self._timer.cancel()
            
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Transition: Inactive -> Unconfigured"""
        self.get_logger().info(f"Cleaning up from {state.label}...")
        
        if self._timer:
            self._timer.destroy()
            self._timer = None
            
        self._telemetry.cleanup(self)
        self.undeclare_parameter("timer_period_s")
        
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Transition: Any -> Finalized"""
        self.get_logger().info(f"Shutting down from {state.label}...")
        
        if self._timer:
            self._timer.destroy()
        self._telemetry.cleanup(self)
        
        return TransitionCallbackReturn.SUCCESS

    # --- Helper Functions ---

    def _declare_parameters(self) -> None:
        self.declare_parameter("timer_period_s", 1.0)

    def _on_param_update(self, parameters: List[rclpy.Parameter]) -> SetParametersResult:
        for parameter in parameters:
            if parameter.name == "timer_period_s":
                validation = validate_timer_period_s(parameter.value)
                if not validation.ok:
                     return SetParametersResult(successful=False, reason=validation.reason)
                
                self._update_timer_period(validation.value)

        return SetParametersResult(successful=True)

    def _create_timer_paused(self, period_s: float) -> None:
        self._timer_period_s = period_s
        self._timer = self.create_timer(self._timer_period_s, self._telemetry.publish)
        self._timer.cancel()  # Start paused

    def _update_timer_period(self, period_s: float) -> None:
        if self._timer is None:
            return
            
        if self._timer_period_s == period_s:
            return

        is_running = not self._timer.is_canceled()
        
        self._timer.destroy()
        self._timer_period_s = period_s
        self._timer = self.create_timer(self._timer_period_s, self._telemetry.publish)
        
        if not is_running:
            self._timer.cancel()
            
        self.get_logger().info(f"Timer updated to {period_s}s")


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = LifecyclePublisherNode()
    
    # Use a MultiThreadedExecutor to prevent deadlocks during 
    # service-based state transitions
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        node.get_logger().error(f"Unhandled exception: {exc}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()