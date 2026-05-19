"""Consolidated LiDAR stack: driver + NaN filter + timestamp re-stamper + the
base_link→laser static TF. All in one container/launch. Topic flow:

    oradar driver  →  /scan_tmp   (raw scan, may include NaNs)
    NaN filter     →  /scan_clean
    transformer    →  /scan       (timestamp re-stamped to current clock)

Also publishes the laser static TF so this stack works without requiring
kros_car's robot_state_publisher. When kros_car also runs, both will publish
the same transform — TF2 handles that fine for matching values.
"""
import os
import yaml
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _laser_offset_from_xacro(configs_dir: str = '/configs') -> dict:
    """Parse the base_to_laser joint translation out of kros_car.xacro so the
    static TF stays in sync with the URDF without hand-duplicating numbers.
    Falls back to defaults if parsing fails for any reason.
    """
    defaults = {'x': 0.021, 'y': 0.0, 'z': 0.278}
    path = os.path.join(configs_dir, 'kros_car.xacro')
    try:
        with open(path) as f:
            text = f.read()
    except OSError:
        return defaults
    # Crude regex: find <joint name="base_to_laser" ... <origin xyz="X Y Z" />
    import re
    m = re.search(
        r'base_to_laser.*?<origin\s+xyz="([^"]+)"',
        text, flags=re.DOTALL,
    )
    if not m:
        return defaults
    parts = m.group(1).split()
    if len(parts) != 3:
        return defaults
    try:
        return {'x': float(parts[0]), 'y': float(parts[1]), 'z': float(parts[2])}
    except ValueError:
        return defaults


def generate_launch_description():
    ms200_scan_launch = os.path.join(
        get_package_share_directory('oradar_lidar'),
        'launch', 'ms200_scan.launch.py',
    )
    laser = _laser_offset_from_xacro()

    # 1. MS200 driver. Camera obstructs the front ±80°, so the driver only
    #    emits the back 200° arc (80°→280°) — saves CPU downstream and stops
    #    ICP / SLAM matching against a known-bad slice of the scan.
    lidar_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ms200_scan_launch),
        launch_arguments={
            'angle_min':  '80.0',
            'angle_max':  '280.0',
            'range_max':  '12.0',
            'scan_topic': '/scan_tmp',
        }.items(),
    )

    # 2. NaN filter. Some MS200 frames carry NaN ranges that crash downstream
    #    matchers; this strips them. Remapped to publish to /scan_clean so the
    #    transformer below can pick up the cleaned scan and re-stamp it.
    #    (Without the remap the filter and transformer both publish to /scan,
    #    causing a two-publisher race.)
    nan_filter = Node(
        package='lidar_pkg',
        executable='lidar_nan_value_filter_node',
        name='lidar_nan_filter',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        remappings=[
            ('/scan_tmp', '/scan_tmp'),
            ('/scan',     '/scan_clean'),
        ],
    )

    # 3. Timestamp re-stamper. The MS200 stamps with its internal clock which
    #    drifts vs ROS clock and breaks approx_sync downstream. Consumes the
    #    NaN-cleaned scan from step 2 and republishes as /scan with the current
    #    ROS clock stamp — the topic downstream nodes (SLAM, ICP, Nav2) read.
    transformer = ExecuteProcess(
        cmd=['python3', '/lidar_transformer_node.py'],
        output='screen',
        name='lidar_transformer',
        additional_env={'IGVI_SCAN_IN': '/scan_clean'},
        respawn=True,
        respawn_delay=2.0,
    )

    # base_link → laser static TF — keeps SLAM working when kros_car isn't up.
    # Reads xyz from the URDF so a single edit there propagates here too.
    base_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser',
        respawn=True,
        respawn_delay=2.0,
        arguments=[
            '--x', str(laser['x']),
            '--y', str(laser['y']),
            '--z', str(laser['z']),
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link', '--child-frame-id', 'laser',
        ],
    )

    return LaunchDescription([
        lidar_driver,
        nan_filter,
        transformer,
        base_to_laser_tf,
    ])
