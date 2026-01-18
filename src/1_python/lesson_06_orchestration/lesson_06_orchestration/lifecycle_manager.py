#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import Transition
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import signal

# Lifecycle State IDs
STATE_UNCONFIGURED = 1
STATE_INACTIVE = 2
STATE_ACTIVE = 3
STATE_FINALIZED = 4

class LifecycleManager(Node):
    """
    Orchestrates the lifecycle of managed nodes using a two-phase startup strategy.
    
    Startup Strategy:
    1. Configuration Barrier: All nodes must reach the Inactive state before any are activated.
    2. Ordered Activation: Subscribers are activated first, followed by Publishers.
    
    Shutdown Strategy:
    - Best-effort reverse-order teardown (Active -> Inactive -> Finalized).
    """

    def __init__(self):
        super().__init__("lifecycle_manager")

        # --- Parameter Declaration ---
        descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)
        self.declare_parameter("node_names", 
                             ['/lesson_06_lifecycle_publisher', '/lesson_06_lifecycle_subscriber'], 
                             descriptor)
        self.declare_parameter("autostart", True)
        self.declare_parameter("timeout_s", 10.0)
        self.declare_parameter("continue_on_error", False)
        self.declare_parameter("shutdown_on_exit", True)

        # --- Parameter Caching ---
        self._autostart = self.get_parameter("autostart").value
        self._timeout_s = self.get_parameter("timeout_s").value
        self._continue_on_error = self.get_parameter("continue_on_error").value
        self._shutdown_on_exit = self.get_parameter("shutdown_on_exit").value

        # --- Target Resolution ---
        self._target_nodes = []
        raw_names = self.get_parameter("node_names").value
        if raw_names:
            for name in raw_names:
                # Ensure fully qualified names
                self._target_nodes.append(name if name.startswith("/") else f"/{name}")
        
        self.get_logger().info(f"Lifecycle Manager initialized. Targets: {self._target_nodes}")

        # --- Service Clients ---
        self._lifecycle_clients = {}
        for node_name in self._target_nodes:
            self._lifecycle_clients[node_name] = {
                "get_state": self.create_client(GetState, f"{node_name}/get_state"),
                "change_state": self.create_client(ChangeState, f"{node_name}/change_state"),
            }

        # --- Runtime State ---
        self._shutdown_requested = False

    def request_shutdown(self):
        """Sets the flag to trigger shutdown on the next event loop iteration."""
        self._shutdown_requested = True

    def orchestrate_startup(self):
        """
        Executes the two-phase startup sequence.
        
        Returns:
            bool: True if the sequence completed successfully, False otherwise.
        """
        self.get_logger().info("Starting Orchestration: Configuration Phase (Barrier)")
        
        # --- Phase 1: Configuration Barrier ---
        barrier_passed = True
        
        for node_name in self._target_nodes:
            if not self._wait_for_services(node_name):
                self.get_logger().error(f"Service lookup failed: {node_name}")
                if not self._continue_on_error:
                    return False
                barrier_passed = False
                continue

            current_state = self._get_lifecycle_state(node_name)
            if current_state is None:
                barrier_passed = False
                continue

            # Enforce Invariant: Unconfigured -> Inactive
            if current_state.id == STATE_UNCONFIGURED:
                if not self._change_lifecycle_state(node_name, Transition.TRANSITION_CONFIGURE, "Configure"):
                    self.get_logger().error(f"Failed to Configure {node_name}")
                    if not self._continue_on_error:
                        return False
                    barrier_passed = False
                    continue
                
                # Check resulting state
                new_state = self._get_lifecycle_state(node_name)
                if new_state.id != STATE_INACTIVE:
                    self.get_logger().error(f"{node_name} failed to reach Inactive state (Current: {new_state.label})")
                    barrier_passed = False

            elif current_state.id == STATE_INACTIVE:
                self.get_logger().info(f"{node_name} is already Inactive.")

            elif current_state.id == STATE_ACTIVE:
                self.get_logger().error(f"{node_name} is already ACTIVE. This violates the startup barrier.")
                if not self._continue_on_error:
                    return False
                barrier_passed = False

            else:
                 self.get_logger().error(f"{node_name} is in terminal state {current_state.label}.")
                 barrier_passed = False

        if not barrier_passed:
            self.get_logger().error("Configuration Barrier Failed: Not all nodes reached Inactive. Aborting Activation.")
            return False
            
        self.get_logger().info("Barrier Phase Complete. Starting Ordered Activation.")

        # --- Phase 2: Ordered Activation ---
        # Rule: Subscribers (priority 0) -> Others (1) -> Publishers (2)
        def target_sort_key(name):
            lower_name = name.lower()
            if "subscriber" in lower_name: return 0
            if "publisher" in lower_name: return 2
            return 1
        
        ordered_targets = sorted(self._target_nodes, key=target_sort_key)
        
        for node_name in ordered_targets:
            self.get_logger().info(f"Activating {node_name}...")
            if not self._change_lifecycle_state(node_name, Transition.TRANSITION_ACTIVATE, "Activate"):
                 self.get_logger().error(f"Activation failed for {node_name}")
                 if not self._continue_on_error:
                     return False
            
            # Optional verification could go here, but transition success implies intended state.

        self.get_logger().info("System Activated Successfully.")
        return True

    def orchestrate_shutdown(self):
        """
        Executes best-effort shutdown sequence in reverse order.
        """
        self.get_logger().info("Starting Shutdown Sequence...")
        
        # Shutdown reverse of definition order (or strictly reverse of activation if tracked, but simplifed here)
        targets = list(reversed(self._target_nodes))

        for node_name in targets:
            state = self._get_lifecycle_state(node_name)
            if state is None:
                continue

            # Active -> Inactive
            if state.id == STATE_ACTIVE:
                self.get_logger().info(f"Deactivating {node_name}...")
                if self._change_lifecycle_state(node_name, Transition.TRANSITION_DEACTIVATE, "Deactivate"):
                    state = self._get_lifecycle_state(node_name)

            # Inactive -> Shutdown
            if state and state.id == STATE_INACTIVE:
                self.get_logger().info(f"Shutting down {node_name}...")
                self._change_lifecycle_state(node_name, Transition.TRANSITION_INACTIVE_SHUTDOWN, "Shutdown")
            
            # Unconfigured -> Shutdown
            elif state and state.id == STATE_UNCONFIGURED:
                 self.get_logger().info(f"Shutting down {node_name}...")
                 self._change_lifecycle_state(node_name, Transition.TRANSITION_UNCONFIGURED_SHUTDOWN, "Shutdown")

        self.get_logger().info("Shutdown Sequence Completed.")


    def _wait_for_services(self, node_name):
        client_get = self._lifecycle_clients[node_name]["get_state"]
        client_change = self._lifecycle_clients[node_name]["change_state"]
        
        if not client_get.wait_for_service(timeout_sec=self._timeout_s):
            return False
        if not client_change.wait_for_service(timeout_sec=self._timeout_s):
            return False
        return True

    def _get_lifecycle_state(self, node_name):
        client = self._lifecycle_clients[node_name]["get_state"]
        request = GetState.Request()
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self._timeout_s)
        
        if future.result() is not None:
             return future.result().current_state
        else:
             self.get_logger().error(f"Failed to get state for {node_name}")
             return None

    def _change_lifecycle_state(self, node_name, transition_id, label="Unknown"):
        client = self._lifecycle_clients[node_name]["change_state"]
        request = ChangeState.Request()
        request.transition.id = transition_id
        request.transition.label = label.lower()

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self._timeout_s)
        
        if future.result() is not None:
            if future.result().success:
                return True
            else:
                self.get_logger().warning(f"Transition {label} rejected by {node_name}")
                return False
        else:
            self.get_logger().error(f"Failed to call change_state for {node_name}")
            return False

def main(args=None):
    rclpy.init(args=args)
    manager = LifecycleManager()
    
    # --- Signal Handling ---
    def signal_handler(sig, frame):
        manager.request_shutdown()
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # --- Startup ---
    if manager._autostart:
        try:
            manager.orchestrate_startup()
        except KeyboardInterrupt:
            manager.request_shutdown()

    # --- Main Loop (Spin Once) ---
    try:
        while rclpy.ok():
            if manager._shutdown_requested:
                break
            rclpy.spin_once(manager, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass # Signal handler already caught it or loop exit prompted it
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        # --- Shutdown ---
        if manager._shutdown_on_exit and manager._shutdown_requested:
             manager.orchestrate_shutdown()

        manager.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
