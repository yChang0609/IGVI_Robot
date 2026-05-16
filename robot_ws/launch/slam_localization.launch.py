import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    configs_dir = '/configs'

    database_path = LaunchConfiguration('database_path')
    cam_tx = LaunchConfiguration('camera_mount_x')
    cam_ty = LaunchConfiguration('camera_mount_y')
    cam_tz = LaunchConfiguration('camera_mount_z')
    cam_rr = LaunchConfiguration('camera_mount_roll')
    cam_rp = LaunchConfiguration('camera_mount_pitch')
    cam_ry = LaunchConfiguration('camera_mount_yaw')

    declared_arguments = [
        DeclareLaunchArgument(
            'database_path',
            default_value='/root/.ros/map.db',
            description='Path to the RTAB-Map database file inside the container.',
        ),
        DeclareLaunchArgument('camera_mount_x', default_value='0.0'),
        DeclareLaunchArgument('camera_mount_y', default_value='0.0'),
        DeclareLaunchArgument('camera_mount_z', default_value='0.0'),
        DeclareLaunchArgument('camera_mount_roll',  default_value='0.0'),
        DeclareLaunchArgument('camera_mount_pitch', default_value='0.0'),
        DeclareLaunchArgument('camera_mount_yaw',   default_value='0.0'),
    ]

    # Same IMU filter as mapping
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

    # Same camera mount TF as mapping
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

    # Same odometry as mapping — tracks robot motion against the loaded map
    rgbd_odom_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        output='screen',
        parameters=[{
            'frame_id':           'base_link',
            'odom_frame_id':      'odom',
            # EKF (wheel + IMU) owns odom->base_link; visual odom is published
            # on /odom_visual only, not the TF authority.
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

    # Low-latency state estimator: wheel odom + Kinect IMU yaw rate -> 50 Hz
    # odom->base_link. See configs/ekf_wheel_imu.yaml.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(configs_dir, 'ekf_wheel_imu.yaml')],
    )

    # RTAB-Map in localization mode:
    #   Mem/IncrementalMemory=False  → never adds new nodes, map is frozen
    #   Mem/InitWMWithAllNodes=True  → loads entire saved database at startup
    rtabmap_loc_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[{
            'database_path':             database_path,
            'frame_id':                  'base_link',
            'subscribe_depth':           True,
            'subscribe_rgb':             True,
            'subscribe_scan':            False,
            'approx_sync':               True,
            'Mem/IncrementalMemory':     'False',
            'Mem/InitWMWithAllNodes':    'True',
            'Reg/Strategy':              '0',
            'Reg/Force3DoF':             'true',
            'RGBD/NeighborLinkRefining': 'true',
            'Grid/Sensor':               '1',
            'Grid/CellSize':             '0.05',
            'Grid/MaxGroundAngle':       '45',
            'Grid/MaxObstacleHeight':    '2.0',
            'Grid/MinGroundHeight':      '-0.1',
            'Mem/UseOdomGravity':        'true',
            'Optimizer/GravitySigma':    '0.25',
        }],
        remappings=[
            ('rgb/image',       '/rgb/image_raw'),
            ('rgb/camera_info', '/rgb/camera_info'),
            ('depth/image',     '/depth_to_rgb/image_raw'),
            # RTAB-Map localizes on the fused EKF odom, publishes only map->odom.
            ('odom',            '/odometry/filtered'),
        ],
    )

    return LaunchDescription(declared_arguments + [
        imu_filter_node,
        base_to_camera_tf,
        ekf_node,
        rgbd_odom_node,
        rtabmap_loc_node,
    ])
