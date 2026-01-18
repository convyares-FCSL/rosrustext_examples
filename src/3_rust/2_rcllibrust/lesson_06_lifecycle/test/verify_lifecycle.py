#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition
from sensor_msgs.msg import Joy
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
import time
import sys

# Constants matching lesson
PUB_NODE = "/lesson_06_lifecycle_publisher"
SUB_NODE = "/lesson_06_lifecycle_subscriber"
SERVICE_TIMEOUT = 2.0

class LifecycleVerifier(Node):
    def __init__(self):
        super().__init__('lifecycle_verifier')

    def call_change_state(self, node_name, transition_id, label):
        cli = self.create_client(ChangeState, f'{node_name}/change_state')
        if not cli.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
            self.get_logger().error(f'Service {node_name}/change_state not available')
            return False
        
        req = ChangeState.Request()
        req.transition.id = transition_id
        req.transition.label = label
        
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().success

    def call_get_state(self, node_name):
        cli = self.create_client(GetState, f'{node_name}/get_state')
        if not cli.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
            return None
        req = GetState.Request()
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().current_state

    def call_set_param(self, node_name, param_name, float_val):
        cli = self.create_client(SetParameters, f'{node_name}/set_parameters')
        if not cli.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
            return False
        
        req = SetParameters.Request()
        p = Parameter()
        p.name = param_name
        p.value.type = ParameterType.PARAMETER_DOUBLE
        p.value.double_value = float_val
        req.parameters.append(p)
        
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        return res.results[0].successful

def main(args=None):
    rclpy.init(args=args)
    verifier = LifecycleVerifier()
    
    print("--- Verifying Lesson 06 Lifecycle (Roslibrust) ---")
    print("Ensure rosbridge_server is running and cargo run commands are active.")
    
    # 1. Check Initial State
    state = verifier.call_get_state(PUB_NODE)
    if state is None:
        print(f"FAIL: Could not reach {PUB_NODE}. Is it running?")
        sys.exit(1)
    print(f"Publisher State: {state.label} ({state.id})")
    assert state.id == 1 or state.id == 2 # Unconfigured or Inactive (if restart)

    # 2. Configure
    print("Transitioning to Inactive...")
    if not verifier.call_change_state(PUB_NODE, 1, 'configure'):
        print("FAIL: Configure Publisher failed")
    if not verifier.call_change_state(SUB_NODE, 1, 'configure'):
        print("FAIL: Configure Subscriber failed")
        
    state = verifier.call_get_state(PUB_NODE)
    assert state.id == 2 # Inactive
    print("State: Inactive")

    # 3. Activate
    print("Transitioning to Active...")
    verifier.call_change_state(PUB_NODE, 3, 'activate')
    verifier.call_change_state(SUB_NODE, 3, 'activate')
    state = verifier.call_get_state(PUB_NODE)
    assert state.id == 3 # Active
    print("State: Active")

    # 4. Set Parameters
    print("Setting timer period...")
    if verifier.call_set_param(PUB_NODE, "timer_period_s", 0.1):
        print("Parameter set successfully.")
    else:
        print("FAIL: Parameter set failed")

    # 5. Deactivate
    print("Transitioning to Inactive...")
    verifier.call_change_state(PUB_NODE, 4, 'deactivate')
    state = verifier.call_get_state(PUB_NODE)
    assert state.id == 2
    print("State: Inactive")

    # 6. Cleanup
    print("Cleaning up...")
    verifier.call_change_state(PUB_NODE, 2, 'cleanup')
    verifier.call_change_state(SUB_NODE, 2, 'cleanup')
    state = verifier.call_get_state(PUB_NODE)
    assert state.id == 1
    print("State: Unconfigured")
    
    print("--- Verification Check Passed ---")
    verifier.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
