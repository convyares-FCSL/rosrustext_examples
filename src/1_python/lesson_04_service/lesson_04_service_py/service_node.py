#!/usr/bin/env python3
import rclpy
from typing import List, Optional
from rclpy.node import Node

from lesson_interfaces.srv import ComputeStats
from utils_py import services
from lesson_04_service_py.stats_logic import StatsLogic


class ServiceNode(Node):
    """
    ROS resource container for the ComputeStats service.
    
    Responsibilities:
    1. Own the ROS Service Server handle.
    2. Deserialise ROS requests.
    3. Delegate processing to the pure-logic 'StatsLogic' class.
    4. Serialise ROS responses.
    """

    def __init__(self) -> None:
        super().__init__('lesson_04_service_server')

        self.service_name = services.compute_stats(self)

        self._srv = self.create_service(
            ComputeStats, 
            self.service_name, 
            self.compute_stats_callback
        )
        
        self.get_logger().info(f'Service "{self.service_name}" is ready.')

    def compute_stats_callback(self, request: ComputeStats.Request, response: ComputeStats.Response) -> ComputeStats.Response:
        """
        Callback acts as the 'Adapter' layer between ROS types and Python types.
        """
        self.get_logger().info(f"Incoming request with {len(request.data)} samples.")

        # 1. Delegate Business Logic (Pure Python)
        # We pass the raw data list; StatsLogic returns a pure tuple.
        # This allows StatsLogic to be unit-tested without 'request' or 'response' objects.
        total, avg, status = StatsLogic.compute(request.data)

        # 2. Populate Response (ROS Adapter)
        response.sum = float(total)
        response.average = float(avg)
        response.status = status

        return response


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[ServiceNode] = None

    try:
        node = ServiceNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as exc:
        logger = rclpy.logging.get_logger("lesson_04_service_server")
        logger.error(f"Exception in main: {exc}")

    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()