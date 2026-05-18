from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory  # 加入這一行
from ament_index_python.packages import get_package_share_directory  # 加入這一行
import os

def generate_launch_description():
    # 找到 oradar_lidar package 內的 launch 檔案
    ms200_scan_launch = os.path.join(
        get_package_share_directory('oradar_lidar'),
        'launch', 'ms200_scan.launch.py'
    )

    lidar_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ms200_scan_launch),
        launch_arguments={
            # Camera obstructs the front ±80°; only the back 200° arc gives
            # usable returns. Lidar 0° = base_link +X (forward), CCW.
            # Window 80°→280° = the rear arc centered on 180° (straight back).
            'angle_min': '80.0',
            'angle_max': '280.0',
            'range_max': '12.0',
            'scan_topic': '/scan_tmp'
        }.items()
    )

    # 設定 laser 到 base_link 的靜態 TF
    # tf2_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_tf_pub_laser',
    #     # 參數依序：X(前後), Y(左右), Z(高), Roll, Pitch, Yaw, 父座標, 子座標
    #     arguments=['0.034', '0.0', '0.288', '0', '0', '0', 'base_link', 'laser'],
    # )

    return LaunchDescription([
        lidar_driver,
        # tf2_node,
    ])
