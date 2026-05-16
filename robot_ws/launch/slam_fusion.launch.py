import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    configs_dir = '/configs'

    delete_db_on_start = LaunchConfiguration('delete_db_on_start')
    database_path      = LaunchConfiguration('database_path')
    cam_tx = LaunchConfiguration('camera_mount_x')
    cam_ty = LaunchConfiguration('camera_mount_y')
    cam_tz = LaunchConfiguration('camera_mount_z')
    cam_rr = LaunchConfiguration('camera_mount_roll')
    cam_rp = LaunchConfiguration('camera_mount_pitch')
    cam_ry = LaunchConfiguration('camera_mount_yaw')

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
        # Camera mount position relative to base_link.
        # Run ./scripts/show_base_link.sh and open Foxglove to visualise base_link,
        # then measure the Kinect camera_base origin from it and set these values.
        DeclareLaunchArgument('camera_mount_x', default_value='0.0',
                              description='Kinect camera_base X from base_link (m, forward+)'),
        DeclareLaunchArgument('camera_mount_y', default_value='0.0',
                              description='Kinect camera_base Y from base_link (m, left+)'),
        DeclareLaunchArgument('camera_mount_z', default_value='0.0',
                              description='Kinect camera_base Z from base_link (m, up+)'),
        DeclareLaunchArgument('camera_mount_roll',  default_value='0.0'),
        DeclareLaunchArgument('camera_mount_pitch', default_value='0.0'),
        DeclareLaunchArgument('camera_mount_yaw',   default_value='0.0'),
    ]

    # ── 1. Madgwick IMU filter ────────────────────────────────────────────────
    # Converts raw Kinect /imu (accel + gyro, no mag) → orientation estimate.
    # /imu/filtered is consumed by RTAB-Map for gravity-aligned loop closure.
    # rgbd_odometry subscribes to raw /imu for IMU pre-integration.
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[os.path.join(configs_dir, 'imu_filter_kinect.yaml')],
        remappings=[
            ('imu/data_raw', '/imu'),
            ('imu/data',     '/imu/filtered'),
        ],
    )

    # ── 2. Static TF: base_link → camera_base ────────────────────────────────
    # Locates the Kinect body in the robot frame.  The Kinect driver then
    # publishes camera_base → camera_color_left, camera_imu_frame, etc.,
    # completing the full TF chain needed for gravity alignment and map fusion.
    base_to_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_camera_base',
        arguments=[
            '--x',     cam_tx, '--y',     cam_ty, '--z',   cam_tz,
            '--roll',  cam_rr, '--pitch', cam_rp, '--yaw', cam_ry,
            '--frame-id', 'base_link', '--child-frame-id', 'camera_base',
        ],
    )

    # ── 3. RGBD Odometry ─────────────────────────────────────────────────────
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

    # ── 4. RTAB-Map SLAM (RGBD mode) ─────────────────────────────────────────
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
