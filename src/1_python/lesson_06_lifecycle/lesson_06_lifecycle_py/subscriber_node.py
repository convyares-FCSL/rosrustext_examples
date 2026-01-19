#!/usr/bin/env python3
"""
Lesson 06 Lifecycle Subscriber (Python).

Demonstrates a Managed Node where the Subscriber component manually gates
data flow based on the Lifecycle state.
"""
from __future__ import annotations

from typing import List, Optional

import rclpy
from rcl_interfaces.msg import SetParametersResult

from rclpy.lifecycle import TransitionCallbackReturn
from lifecycle_msgs.msg import State

from lesson_interfaces.msg import MsgCount
from utils_py import qos, topics, LifecycleNode # FIXED: Import our Shim

from lesson_05_parameters_py.logic import TelemetryStreamValidator, StreamEvent


class TelemetrySubscriber:
    """
    Lifecycle-aware Component.
    """

    def __init__(self) -> None:
        self._logic = TelemetryStreamValidator()
        self._sub = None
        self._logger = None
        self._is_enabled = False

    # FIXED: Type hint updated to use our Shim class
    def configure(self, parent_node: LifecycleNode) -> None:
        """Create the subscription resources (Inactive state)."""
        self._logger = parent_node.get_logger()
        
        topic = topics.telemetry(parent_node)
        profile = qos.telemetry(parent_node)
        
        self._sub = parent_node.create_subscription(
            MsgCount, 
            topic, 
            self._on_message, 
            profile
        )
        self._logger.info(f"Subscriber configured on '{topic}'")

    def activate(self) -> None:
        """Open the gate (Active state)."""
        self._is_enabled = True
        if self._logger:
            self._logger.info("Subscriber Activated")

    def deactivate(self) -> None:
        """Close the gate (Inactive state)."""
        self._is_enabled = False
        if self._logger:
            self._logger.info("Subscriber Deactivated")

    def cleanup(self, parent_node: LifecycleNode) -> None:
        """Destroy resources (Unconfigured state)."""
        if self._sub:
            parent_node.destroy_subscription(self._sub)
            self._sub = None
        self._is_enabled = False
        self._logger = None

    def update_config(self, reset_max_value: int) -> None:
        self._logic.set_reset_max_value(reset_max_value)

    def _on_message(self, msg: MsgCount) -> None:
        """The Translation Layer with Lifecycle Gate."""
        if not self._is_enabled:
            return

        decision = self._logic.on_count(msg.count)

        if self._logger:
            if decision.event == StreamEvent.RESET:
                self._logger.warn(decision.message)
            elif decision.event == StreamEvent.OUT_OF_ORDER:
                self._logger.error(decision.message)
            else:
                self._logger.info(decision.message)


# FIXED: Inherit from 'LifecycleNode', not 'NodeLifecycle'
class LifecycleSubscriberNode(LifecycleNode):
    """
    The Lifecycle Authority.
    """

    def __init__(self) -> None:
        super().__init__("lesson_06_lifecycle_subscriber")
        
        self._telemetry = TelemetrySubscriber()
        self.get_logger().info("Node initialized (Unconfigured). Waiting for manager...")

    # --- Lifecycle State Machine Callbacks ---

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Transition: Unconfigured -> Inactive"""
        self.get_logger().info(f"Configuring from {state.label}...")
        
        try:
            # 1. Declare Parameters
            self._declare_parameters()
            self.add_on_set_parameters_callback(self._on_param_update)

            # 2. Configure Component
            self._telemetry.configure(self)
            
            # 3. Apply Initial Config
            initial_val = self.get_parameter("subscriber.reset_max_value").value
            self._telemetry.update_config(int(initial_val))
            
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Configuration failed: {e}")
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Transition: Inactive -> Active"""
        self.get_logger().info(f"Activating from {state.label}...")
        self._telemetry.activate()
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Transition: Active -> Inactive"""
        self.get_logger().info(f"Deactivating from {state.label}...")
        self._telemetry.deactivate()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Transition: Inactive -> Unconfigured"""
        self.get_logger().info(f"Cleaning up from {state.label}...")
        
        self._telemetry.cleanup(self)
        self.undeclare_parameter("subscriber.reset_max_value")
        
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Transition: Any -> Finalized"""
        self.get_logger().info(f"Shutting down from {state.label}...")
        self._telemetry.cleanup(self)
        return TransitionCallbackReturn.SUCCESS

    # --- Helper Functions ---

    def _declare_parameters(self) -> None:
        """Declare all parameters. Can be extensive."""
        self.declare_parameter("subscriber.reset_max_value", 10)

    def _on_param_update(self, parameters: List[rclpy.Parameter]) -> SetParametersResult:
        for p in parameters:
            if p.name == "subscriber.reset_max_value":
                try:
                    self._telemetry.update_config(int(p.value))
                    self.get_logger().info(f"Updated reset tolerance to {p.value}")
                except ValueError:
                    return SetParametersResult(successful=False)
        return SetParametersResult(successful=True)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = LifecycleSubscriberNode()
    
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
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()