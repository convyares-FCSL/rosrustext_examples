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

    # 2. Define Components
    # Same components as Lesson 09, loading legacy nodes into one container.
    
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

    action_server = ComposableNode(
        package='lesson_08_executors_cpp',
        plugin='lesson_08_executors_cpp::ActionServerNode',
        name='lesson_08_action_server',
        parameters=config_args
    )

    # 3. Profile A Container: Shared Fate
    # All nodes run in this single process.
    container = ComposableNodeContainer(
        name='lesson_10_composed_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            lifecycle_pub,
            lifecycle_sub,
            action_server
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )

    return LaunchDescription([
        container
    ])
