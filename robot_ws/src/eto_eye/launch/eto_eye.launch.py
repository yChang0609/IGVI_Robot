from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='eto_eye',
            executable='detector_node',
            name='detector_node',
            output='screen'
        ),
        Node(
            package='eto_eye',
            executable='semantic_memory_node',
            name='semantic_memory_node',
            output='screen'
        )
    ])
