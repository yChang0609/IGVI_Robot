import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    configs_dir = '/configs'

    calib_path = os.path.join(configs_dir, 'calibration.yaml')
    with open(calib_path, 'r') as f:
        calib = yaml.safe_load(f) or {}
    cam = calib.get('camera_extrinsics', {})

    delete_db_on_start = LaunchConfiguration('delete_db_on_start')
    database_path      = LaunchConfiguration('database_path')

    declared_arguments = [
        DeclareLaunchArgument(
            'delete_db_on_start',
            default_value='false',
            description='Delete the RTAB-Map database on startup (fresh map).',
        ),
        DeclareLaunchArgument(
            'database_path',
            default_value='/root/.ros/rtabmap.db',
            description='Path to the RTAB-Map database file inside the container.',
        ),
    ]

    # ── 1. Madgwick IMU filter ────────────────────────────────────────────────
    # Converts calibrated Kinect IMU (accel + gyro, no mag) → orientation estimate.
    # /imu/filtered is consumed by RTAB-Map for gravity-aligned loop closure.
    # rgbd_odometry subscribes to filtered orientation for IMU initialization.
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[os.path.join(configs_dir, 'imu_filter_kinect.yaml')],
        remappings=[
            ('imu/data_raw', '/imu/calibrated'),
            ('imu/data',     '/imu/filtered'),
        ],
    )

    # ── 3. Static TF: base_link → camera_base ────────────────────────────────
    # Locates the Kinect body in the robot frame.  The Kinect driver then
    # publishes camera_base → camera_color_left, camera_imu_frame, etc.,
    # completing the full TF chain needed for gravity alignment and map fusion.
    base_to_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_camera_base',
        arguments=[
            '--x',     str(cam.get('x', 0.0)),
            '--y',     str(cam.get('y', 0.0)),
            '--z',     str(cam.get('z', 0.0)),
            '--roll',  str(cam.get('roll', 0.0)),
            '--pitch', str(cam.get('pitch', 0.0)),
            '--yaw',   str(cam.get('yaw', 0.0)),
            '--frame-id', 'base_link', '--child-frame-id', 'camera_base',
        ],
    )

    # ── 4. RGBD Odometry ─────────────────────────────────────────────────────
    # Visual feature tracking on RGB + depth.  Works with a narrow-FoV depth
    # camera; does not need a 360° LiDAR or a geometrically complex scan.
    rgbd_odom_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        output='screen',
        parameters=[{
            'frame_id':           'base_link',
            'odom_frame_id':      'odom',
            # No longer the TF authority: the robot_localization EKF owns
            # odom->base_link (low latency). Visual odom is kept only as an
            # input for RTAB-Map / future fusion, on /odom_visual.
            'publish_tf':         False,
            'wait_imu_to_init':   True,
            'Reg/Force3DoF':      'true',
            'Vis/EstimationType': '0',
            'Vis/MinInliers':     '5',
            'Vis/MaxDepth':       '3.5',
            'Vis/MinDepth':       '0.5',
        }],
        remappings=[
            ('rgb/image',       '/rgb/image_raw'),
            ('rgb/camera_info', '/rgb/camera_info'),
            ('depth/image',     '/depth_to_rgb/image_raw'),
            ('imu',             '/imu/filtered'),
            ('odom',            '/odom_visual'),
        ],
    )

    # ── Low-latency state estimator ──────────────────────────────────────────
    # Fuses wheel odometry (/base_controller/odom) + Kinect IMU yaw rate and
    # publishes odom->base_link at 50 Hz. This replaces the laggy RGBD visual
    # odometry on the real-time path; see configs/ekf_wheel_imu.yaml.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(configs_dir, 'ekf_wheel_imu.yaml')],
    )

    # ── 5. RTAB-Map SLAM (RGBD mode) ─────────────────────────────────────────
    # Builds and maintains a persistent 2-D occupancy grid + 3-D point-cloud map
    # from Kinect RGBD frames.  Loop closure uses bag-of-words visual place
    # recognition so revisited areas are correctly merged without LiDAR.
    slam_common_params = {
        'database_path':              database_path,
        'frame_id':                   'base_link',
        'subscribe_depth':            True,
        'subscribe_rgb':              True,
        'subscribe_scan':             False,  # depth camera handles mapping; scan only used for odometry
        'approx_sync':                True,
        'Mem/IncrementalMemory':      'true',
        'Reg/Strategy':               '0',   # 0 = Visual (depth camera)
        'Reg/Force3DoF':              'true',
        'RGBD/NeighborLinkRefining':  'true',
        # Occupancy grid from depth camera
        'Grid/Sensor':                '1',   # 1 = depth camera
        'Grid/CellSize':              '0.05',
        'Grid/MaxGroundAngle':        '45',
        'Grid/MaxObstacleHeight':     '2.0',
        'Grid/MinGroundHeight':       '-0.1',
        'Mem/UseOdomGravity':         'true',
        'Optimizer/GravitySigma':     '0.25',
        'RTAB-Map/TimeThr':           '0',
        'RTAB-Map/DetectionRate':     '5.0',  # 5 Hz — RTAB-Map needs ~100-200ms/frame, 15Hz causes queuing lag
        'Mem/STMSize':                '30',
        'RGBD/LinearUpdate':          '0.1',    # new node only after 10 cm movement
        'RGBD/AngularUpdate':         '0.05',  # new node only after ~3° rotation
    }

    slam_remaps = [
        ('rgb/image',       '/rgb/image_raw'),
        ('rgb/camera_info', '/rgb/camera_info'),
        ('depth/image',     '/depth_to_rgb/image_raw'),
        # RTAB-Map runs on the fused EKF odom and publishes only map->odom.
        ('odom',            '/odometry/filtered'),
    ]

    rtabmap_keep = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[slam_common_params],
        remappings=slam_remaps,
        condition=UnlessCondition(delete_db_on_start),
    )

    rtabmap_fresh = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[slam_common_params],
        remappings=slam_remaps,
        arguments=['--delete_db_on_start'],
        condition=IfCondition(delete_db_on_start),
    )

    return LaunchDescription(declared_arguments + [
        imu_filter_node,
        base_to_camera_tf,
        ekf_node,
        rgbd_odom_node,
        rtabmap_keep,
        rtabmap_fresh,
    ])
