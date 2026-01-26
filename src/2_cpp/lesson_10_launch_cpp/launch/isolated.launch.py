import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # 1. Locate configuration
    interfaces_share = get_package_share_directory('lesson_interfaces')
    topics_config = os.path.join(interfaces_share, 'config', 'topics_config.yaml')
    qos_config = os.path.join(interfaces_share, 'config', 'qos_config.yaml')

    config_args = [
        {'params_file': topics_config},
        {'params_file': qos_config},
    ]

    # CLI args for Executable (Worker Plane)
    # The executable wrapper takes --params-file directly via --ros-args
    worker_cli_args = [
        '--ros-args',
        '--params-file', topics_config,
        '--params-file', qos_config,
    ]

    # 2. Domain 1: Control Plane (Composed)
    # Lifecycle Publisher + Subscriber in one container.
    
    lifecycle_pub = ComposableNode(
        package='lesson_06_lifecycle_cpp',
        plugin='lesson_06_lifecycle_cpp::LifecyclePublisherNode',
        name='lesson_06_lifecycle_publisher',
        parameters=config_args
    )

    lifecycle_sub = ComposableNode(
        package='lesson_06_lifecycle_cpp',
        plugin='lesson_06_lifecycle_cpp::LifecycleSubscriberNode',
        name='lesson_06_lifecycle_subscriber',
        parameters=config_args
    )

    control_plane_container = ComposableNodeContainer(
        name='control_plane_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            lifecycle_pub,
            lifecycle_sub
        ],
        output='screen'
    )

    # 3. Domain 2: Worker Plane (Isolated)
    # Action Server running as a standalone executable.
    # This creates a hard process boundary.
    
    action_worker = Node(
        package='lesson_08_executors_cpp',
        executable='lesson_08_actions_server',
        name='lesson_08_action_server',
        output='screen',
        arguments=worker_cli_args
    )

    return LaunchDescription([
        control_plane_container,
        action_worker
    ])
