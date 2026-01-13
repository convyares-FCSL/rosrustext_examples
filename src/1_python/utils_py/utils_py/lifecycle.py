"""
Missing Lifecycle Node implementation for rclpy.
Provides standard /get_state and /change_state services.
"""
import rclpy
from rclpy.node import Node
from rclpy.lifecycle import TransitionCallbackReturn

# CRITICAL: We use the message definition directly.
from lifecycle_msgs.msg import State as StateMsg
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState


class LifecycleNode(Node):
    """
    Base class that implements the ROS 2 Lifecycle State Machine services.
    Inherit from this instead of rclpy.node.Node for managed nodes.
    """

    def __init__(self, node_name: str, **kwargs) -> None:
        super().__init__(node_name, **kwargs)
        
        # Initial State: Unconfigured
        self._state = StateMsg(
            id=StateMsg.PRIMARY_STATE_UNCONFIGURED, 
            label="unconfigured"
        )
        
        # Standard Lifecycle Services
        self._srv_get_state = self.create_service(
            GetState, 
            f"{node_name}/get_state", 
            self._handle_get_state
        )
        self._srv_change_state = self.create_service(
            ChangeState, 
            f"{node_name}/change_state", 
            self._handle_change_state
        )

    @property
    def current_state(self) -> StateMsg:
        return self._state

    def _handle_get_state(self, request, response):
        response.current_state = self._state
        return response

    def _handle_change_state(self, request, response):
        transition = request.transition
        response.success = False

        # --- Transition Logic ---
        
        # Configure: Unconfigured -> Inactive
        if (transition.id == Transition.TRANSITION_CONFIGURE and 
            self._state.id == StateMsg.PRIMARY_STATE_UNCONFIGURED):
            
            if self.on_configure(self._state) == TransitionCallbackReturn.SUCCESS:
                self._state = StateMsg(id=StateMsg.PRIMARY_STATE_INACTIVE, label="inactive")
                response.success = True

        # Activate: Inactive -> Active
        elif (transition.id == Transition.TRANSITION_ACTIVATE and 
              self._state.id == StateMsg.PRIMARY_STATE_INACTIVE):
            
            if self.on_activate(self._state) == TransitionCallbackReturn.SUCCESS:
                self._state = StateMsg(id=StateMsg.PRIMARY_STATE_ACTIVE, label="active")
                response.success = True

        # Deactivate: Active -> Inactive
        elif (transition.id == Transition.TRANSITION_DEACTIVATE and 
              self._state.id == StateMsg.PRIMARY_STATE_ACTIVE):
            
            if self.on_deactivate(self._state) == TransitionCallbackReturn.SUCCESS:
                self._state = StateMsg(id=StateMsg.PRIMARY_STATE_INACTIVE, label="inactive")
                response.success = True

        # Cleanup: Inactive -> Unconfigured
        elif (transition.id == Transition.TRANSITION_CLEANUP and 
              self._state.id == StateMsg.PRIMARY_STATE_INACTIVE):
            
            if self.on_cleanup(self._state) == TransitionCallbackReturn.SUCCESS:
                self._state = StateMsg(id=StateMsg.PRIMARY_STATE_UNCONFIGURED, label="unconfigured")
                response.success = True

        # Shutdown: Any -> Finalized
        elif transition.id == Transition.TRANSITION_SHUTDOWN:
            if self.on_shutdown(self._state) == TransitionCallbackReturn.SUCCESS:
                self._state = StateMsg(id=StateMsg.PRIMARY_STATE_FINALIZED, label="finalized")
                response.success = True

        return response

    # --- Virtual Methods (Override these) ---
    def on_configure(self, state: StateMsg) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: StateMsg) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: StateMsg) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: StateMsg) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: StateMsg) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS