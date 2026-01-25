import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Locate configuration
    interfaces_share = get_package_share_directory('lesson_interfaces')
    topics_config = os.path.join(interfaces_share, 'config', 'topics_config.yaml')
    qos_config = os.path.join(interfaces_share, 'config', 'qos_config.yaml')

    # 2. Define the composed deployment node
    # Note: We pass params-file args to the executable process.
    composed_node = Node(
        package='lesson_09_composition',
        executable='lesson_09_composed_deployment',
        # name='composed_deployment', # REMOVED: Avoid ambiguity / let ROS generate or default
        output='screen',
        arguments=[
            '--ros-args',
            '--params-file', topics_config,
            '--params-file', qos_config,
        ],
    )

    return LaunchDescription([
        composed_node
    ])
