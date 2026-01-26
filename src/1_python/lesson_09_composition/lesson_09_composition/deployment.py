import os
import rclpy
from rclpy.executors import MultiThreadedExecutor

# Import usage of existing node classes (no modification)
from lesson_06_lifecycle_py.publisher_node import LifecyclePublisherNode
from lesson_06_lifecycle_py.subscriber_node import LifecycleSubscriberNode
from lesson_08_executors.action_server.node import ActionServerNode

def main(args=None):
    rclpy.init(args=args)
    
    # Lesson 09 Intent: Shared executor = shared fate
    executor = MultiThreadedExecutor()
    
    # Instantiate nodes
    pub_node = LifecyclePublisherNode()
    sub_node = LifecycleSubscriberNode()
    action_node = ActionServerNode()
    
    # Compose
    executor.add_node(pub_node)
    executor.add_node(sub_node)
    executor.add_node(action_node)
    
    # Print real PID
    print(f"Starting Composed Deployment (PID: {os.getpid()})")
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        # Explicitly destroy nodes to clean up
        pub_node.destroy_node()
        sub_node.destroy_node()
        action_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
