"""
Action Server Node (Python).

Lifecycle-managed orchestration node that wires together:
- TelemetryPublisher component (periodic topic publication)
- ActionServerComponent (Fibonacci action server)

This node owns:
- Lifecycle transitions (configure/activate/deactivate/cleanup/shutdown)
- Parameter declaration and validation
- Callback group allocation and executor strategy

It does NOT own periodic scheduling resources for telemetry; those are owned by the
TelemetryPublisher component.
"""

from __future__ import annotations

from typing import List, Optional

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.lifecycle import TransitionCallbackReturn
from lifecycle_msgs.msg import State

from utils_py import LifecycleNode

from . import action_server
from . import publisher


class ActionServerNode(LifecycleNode):
    """
    Lifecycle Authority Node.

    Coordinates lifecycle state transitions and delegates resource ownership to
    components. The node remains the single source of truth for:
    - when resources are created/destroyed (configure/cleanup)
    - when work is allowed to run (activate/deactivate)
    - how callbacks are scheduled (callback groups + executor)
    """

    def __init__(self) -> None:
        super().__init__("lesson_08_action_server")

        self._telemetry = publisher.TelemetryPublisher()
        self._action_server = action_server.ActionServerComponent()

        # Callback groups are created during configure() so they only exist
        # when the node is configured.
        self._cbg_pub = None
        self._cbg_action = None

        self.get_logger().info("Node initialized (Unconfigured). Waiting for manager...")

    # --- Lifecycle State Machine Callbacks ---

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """
        Transition: Unconfigured -> Inactive

        Allocate resources, declare parameters, and wire callback groups.
        No periodic publishing occurs until activate().
        """
        self.get_logger().info(f"Configuring from {state.label}...")

        try:
            # Callback group allocation:
            # - Telemetry timer callbacks are isolated from action execution.
            # - Action server callbacks are allowed to overlap (reentrant) to
            #   avoid blocking goal/cancel/result plumbing under load.
            self._cbg_pub = MutuallyExclusiveCallbackGroup()
            self._cbg_action = ReentrantCallbackGroup()

            self._declare_parameters()
            self.add_on_set_parameters_callback(self._on_param_update)

            initial_period = float(self.get_parameter("timer_period_s").value)

            # Component resource allocation (Inactive state).
            self._telemetry.configure(self, self._cbg_pub, initial_period)
            self._action_server.configure(self, self._cbg_action)

            return TransitionCallbackReturn.SUCCESS
        except Exception as exc:
            self.get_logger().error(f"Configuration failed: {exc}")
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """
        Transition: Inactive -> Active

        Enable work. Components are responsible for unpausing timers / opening
        gates when activated.
        """
        self.get_logger().info(f"Activating from {state.label}...")

        self._telemetry.activate()
        self._action_server.activate()

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """
        Transition: Active -> Inactive

        Disable work. Components are responsible for pausing timers / closing
        gates when deactivated.
        """
        self.get_logger().info(f"Deactivating from {state.label}...")

        self._telemetry.deactivate()
        self._action_server.deactivate()

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """
        Transition: Inactive -> Unconfigured

        Destroy all resources and return to a clean slate.
        """
        self.get_logger().info(f"Cleaning up from {state.label}...")

        self._telemetry.cleanup(self)
        self._action_server.cleanup(self)

        # Parameters belong to the configured state; remove on cleanup.
        self.undeclare_parameter("timer_period_s")

        self._cbg_pub = None
        self._cbg_action = None

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """
        Transition: Any -> Finalized

        Best-effort cleanup. This path should be safe even if configure was only
        partially completed.
        """
        self.get_logger().info(f"Shutting down from {state.label}...")

        try:
            self._telemetry.cleanup(self)
        except Exception:
            pass

        try:
            self._action_server.cleanup(self)
        except Exception:
            pass

        return TransitionCallbackReturn.SUCCESS

    # --- Parameter Handling ---

    def _declare_parameters(self) -> None:
        # Keep defaults explicit; tests and launch files can override.
        self.declare_parameter("timer_period_s", 1.0)

    def _on_param_update(self, parameters: List[rclpy.Parameter]) -> SetParametersResult:
        """
        Parameter update callback.

        Validates and applies runtime configuration. This callback must be fast;
        it runs on the executor and should not block.
        """
        for parameter in parameters:
            if parameter.name != "timer_period_s":
                continue

            try:
                val = float(parameter.value)
            except Exception:
                return SetParametersResult(successful=False, reason="timer_period_s must be a number")

            if val <= 0.0:
                return SetParametersResult(successful=False, reason="timer_period_s must be > 0.0")

            # Delegate timer rebuild to the component that owns the timer.
            self._telemetry.set_period(val)

        return SetParametersResult(successful=True)


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = ActionServerNode()

    # MultiThreadedExecutor enables concurrent progress across callback groups.
    # The number of threads should be >= number of groups that must make
    # independent progress under load.
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=3)
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
