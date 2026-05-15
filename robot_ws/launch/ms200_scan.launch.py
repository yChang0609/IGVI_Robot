#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    scan_topic = LaunchConfiguration('scan_topic')
    angle_min = LaunchConfiguration('angle_min')
    angle_max = LaunchConfiguration('angle_max')
    range_max = LaunchConfiguration('range_max')

    ordlidar_node = Node(
        package='oradar_lidar',
        executable='oradar_scan',      # 改這裡
        name='MS200',                  # 改這裡
        output='screen',
        parameters=[
            {'device_model': 'MS200'},
            {'frame_id': 'laser'},
            {'scan_topic': scan_topic},
            {'port_name': '/dev/oradar'},
            {'baudrate': 230400},
            {'angle_min': angle_min},
            {'angle_max': angle_max},
            {'range_min': 0.05},
            {'range_max': range_max},
            {'clockwise': False},
            {'motor_speed': 10}
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('scan_topic', default_value='/scan_tmp'),
        DeclareLaunchArgument('angle_min', default_value='0.1'),
        DeclareLaunchArgument('angle_max', default_value='350.9'),
        DeclareLaunchArgument('range_max', default_value='12.0'),
        ordlidar_node
    ])
