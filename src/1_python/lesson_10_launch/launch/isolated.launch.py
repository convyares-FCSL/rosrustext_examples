import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Locate configuration
    # We use the same configuration as Profile A to ensure fairness.
    interfaces_share = get_package_share_directory('lesson_interfaces')
    topics_config = os.path.join(interfaces_share, 'config', 'topics_config.yaml')
    qos_config = os.path.join(interfaces_share, 'config', 'qos_config.yaml')

    # Common args
    config_args = [
        '--ros-args',
        '--params-file', topics_config,
        '--params-file', qos_config,
    ]

    # 2. Define Topology: Profile B (Fault-Lined / Selectively Isolated)
    #
    # Constraint: Must use existing node artifacts (entry points).
    # Strategy: Deploy each major component as a separate OS process.
    
    # Domain 1: Control Plane (Lifecycle Nodes)
    # Ideally these could share a container, but for this lesson using 
    # simple separate processes (Node actions) is the canonical way to 
    # demonstrate "isolated deployment" vs "composed deployment".
    
    lifecycle_pub = Node(
        package='lesson_06_lifecycle_py',
        executable='lesson_06_lifecycle_publisher',
        output='screen',
        arguments=config_args
    )

    lifecycle_sub = Node(
        package='lesson_06_lifecycle_py',
        executable='lesson_06_lifecycle_subscriber',
        output='screen',
        arguments=config_args
    )

    # Domain 2: Worker Plane (Action Server)
    # This is the "Fault Isolation Boundary".
    # By running in a distinct process, its GIL and Executor starvation 
    # cannot directly halt the Control Plane processes.
    
    action_server = Node(
        package='lesson_08_executors',
        executable='lesson_08_action_server',
        output='screen',
        arguments=config_args
    )

    return LaunchDescription([
        lifecycle_pub,
        lifecycle_sub,
        action_server
    ])
