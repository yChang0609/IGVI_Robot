import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    merge_radius = float(os.environ.get('SEMANTIC_MERGE_RADIUS_M', '0.4'))
    target_filter_config = '/configs/target_filter.yaml'
    semantic_params = [{'merge_radius_m': merge_radius}]
    if os.path.exists(target_filter_config):
        semantic_params.insert(0, target_filter_config)
    if os.environ.get('SEMANTIC_XIONG_MEMORY_MIN_HITS'):
        semantic_params.append({
            'xiong_memory_min_hits': int(os.environ['SEMANTIC_XIONG_MEMORY_MIN_HITS'])
        })

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
            output='screen',
            parameters=semantic_params,
        ),
    ])
