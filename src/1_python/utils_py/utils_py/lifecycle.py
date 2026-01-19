"""
Missing Lifecycle Node implementation for rclpy.
Provides standard /get_state and /change_state services.
"""
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup

from rclpy.node import Node
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.qos import QoSProfile

# CRITICAL: We use the message definition directly.
from lifecycle_msgs.msg import State as StateMsg
from lifecycle_msgs.msg import TransitionEvent
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.msg import TransitionDescription
from lifecycle_msgs.srv import ChangeState, GetState, GetAvailableTransitions

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
        
        # Use ReentrantCallbackGroup to allow lifecycle requests to overlap
        # with other node activity (preventing deadlocks with MultiThreadedExecutor).
        self._lifecycle_cb_group = ReentrantCallbackGroup()

        # Lifecycle Event Publisher
        # Necessary for 'ros2 lifecycle set' CLI to confirm transition completion.
        self._pub_lifecycle_event = self.create_publisher(
            TransitionEvent, 
            "~/transition_event", 
            10,
            callback_group=self._lifecycle_cb_group
        )

        # Standard Lifecycle Services
        self._srv_get_state = self.create_service(
            GetState, 
            "~/get_state",
            self._handle_get_state,
            callback_group=self._lifecycle_cb_group
        )
        self._srv_change_state = self.create_service(
            ChangeState, 
            "~/change_state",
            self._handle_change_state,
            callback_group=self._lifecycle_cb_group
        )
        self._srv_get_available_transitions = self.create_service(
            GetAvailableTransitions, 
            "~/get_available_transitions",
            self._handle_get_available_transitions,
            callback_group=self._lifecycle_cb_group
        )




    @property
    def current_state(self) -> StateMsg:
        return self._state

    def _handle_get_state(self, request, response):
        response.current_state = self._state
        return response

    def _handle_get_available_transitions(self, request, response):
        response.available_transitions = []
        
        # Helper to add transition
        def add(ids, label):
            t = TransitionDescription()
            t.transition.id = ids
            t.transition.label = label
            t.start_state = self._state
            # Goal state is implicit/complex in this view, strictly we define the transition
            response.available_transitions.append(t)

        if self._state.id == StateMsg.PRIMARY_STATE_UNCONFIGURED:
            add(Transition.TRANSITION_CONFIGURE, "configure")
            add(Transition.TRANSITION_UNCONFIGURED_SHUTDOWN, "shutdown")
            
        elif self._state.id == StateMsg.PRIMARY_STATE_INACTIVE:
            add(Transition.TRANSITION_CLEANUP, "cleanup")
            add(Transition.TRANSITION_ACTIVATE, "activate")
            add(Transition.TRANSITION_INACTIVE_SHUTDOWN, "shutdown")
            
        elif self._state.id == StateMsg.PRIMARY_STATE_ACTIVE:
            add(Transition.TRANSITION_DEACTIVATE, "deactivate")
            add(Transition.TRANSITION_ACTIVE_SHUTDOWN, "shutdown")
            
        return response

    def _handle_change_state(self, request, response):
        transition = request.transition
        response.success = False
        
        start_state = self._state

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
        elif (transition.id == Transition.TRANSITION_UNCONFIGURED_SHUTDOWN or
              transition.id == Transition.TRANSITION_INACTIVE_SHUTDOWN or
              transition.id == Transition.TRANSITION_ACTIVE_SHUTDOWN):
            
            if self.on_shutdown(self._state) == TransitionCallbackReturn.SUCCESS:
                self._state = StateMsg(id=StateMsg.PRIMARY_STATE_FINALIZED, label="finalized")
                response.success = True

        if response.success:
            self._publish_transition_event(transition, start_state, self._state)

        return response

    def _publish_transition_event(self, transition: Transition, start_state: StateMsg, goal_state: StateMsg):
        event = TransitionEvent()
        event.timestamp = self.get_clock().now().nanoseconds
        event.transition = transition
        event.start_state = start_state
        event.goal_state = goal_state
        self._pub_lifecycle_event.publish(event)

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