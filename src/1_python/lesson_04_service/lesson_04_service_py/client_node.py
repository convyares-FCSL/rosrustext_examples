#!/usr/bin/env python3
import rclpy
from typing import List, Optional
from rclpy.node import Node

from lesson_interfaces.srv import ComputeStats
from utils_py import services


class ClientNode(Node):
    """
    ROS resource container for the ComputeStats client.
    """

    def __init__(self) -> None:
        super().__init__('lesson_04_service_client')

        self.service_name = services.compute_stats(self)

        self._client = self.create_client(ComputeStats, self.service_name)

        # Wait for service availability
        while not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Service "{self.service_name}" not available, waiting...')
        
        self.get_logger().info(f'Service "{self.service_name}" available.')

    def send_request(self, data: List[float]) -> None:
        """
        Asynchronously sends a request.
        """
        req = ComputeStats.Request()
        req.data = data

        self.get_logger().info(f"Sending request: {data}")
        
        # Async call
        self._future = self._client.call_async(req)
        self._future.add_done_callback(self.response_callback)

    def response_callback(self, future) -> None:
        """
        Handles the async response.
        """
        try:
            response = future.result()
            self.get_logger().info(
                f"Result -> Sum: {response.sum:.2f}, Avg: {response.average:.2f}, Status: '{response.status}'"
            )
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
        
        # In this simple example, we shutdown after one response
        # to allow the process to exit cleanly.
        raise KeyboardInterrupt 


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[ClientNode] = None

    try:
        node = ClientNode()
        
        # Example data to send
        # In a real app, this might come from CLI args or another topic.
        sample_data = [10.5, 20.2, 30.7]
        node.send_request(sample_data)

        rclpy.spin(node)

    except KeyboardInterrupt:
        pass # Clean exit trigger

    except Exception as exc:
        logger = rclpy.logging.get_logger("lesson_04_service_client")
        logger.error(f"Exception in main: {exc}")

    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()