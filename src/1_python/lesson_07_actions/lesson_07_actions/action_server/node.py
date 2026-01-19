"""
Action Server Node (Python).

Orchestrates the Lifecycle Publisher and Action Server components.
"""
from __future__ import annotations

from typing import List, Optional

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.lifecycle import TransitionCallbackReturn
from lifecycle_msgs.msg import State
from rclpy.timer import Timer

from utils_py import qos, topics, LifecycleNode

from . import publisher
from . import action_server

class ActionServerNode(LifecycleNode):    
    """
    Lifecycle Authority Node.
    
    Manages the lifecycle state transitions and coordinates the
    Telemtry Publisher and Fibonacci Action Server components.
    """

    def __init__(self) -> None:
        super().__init__("lesson_07_action_server")
        
        self._telemetry = publisher.TelemetryPublisher()
        self._action_server = action_server.ActionServerComponent()
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
            self._action_server.configure(self)
            
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
        self._action_server.activate()
        if self._timer:
            self._timer.reset()
            
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Transition: Active -> Inactive"""
        self.get_logger().info(f"Deactivating from {state.label}...")
        
        self._telemetry.deactivate()
        self._action_server.deactivate()
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
        self._action_server.cleanup(self)
        self.undeclare_parameter("timer_period_s")
        
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """Transition: Any -> Finalized"""
        self.get_logger().info(f"Shutting down from {state.label}...")
        
        if self._timer:
            self._timer.destroy()
        self._telemetry.cleanup(self)
        self._action_server.cleanup(self)
        
        return TransitionCallbackReturn.SUCCESS

    # --- Helper Functions ---

    def _declare_parameters(self) -> None:
        self.declare_parameter("timer_period_s", 1.0)

    def _on_param_update(self, parameters: List[rclpy.Parameter]) -> SetParametersResult:
        for parameter in parameters:
            if parameter.name == "timer_period_s":
                val = parameter.value
                if val <= 0.0:
                     return SetParametersResult(successful=False, reason="Period must be > 0.0")
                
                self._update_timer_period(val)
                
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
    node = ActionServerNode()
    
    # Use a SingleThreadedExecutor to demonstrate blocking behavior 
    # during long-running callbacks (Lesson 07)
    executor = rclpy.executors.SingleThreadedExecutor()
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