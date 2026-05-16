import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    configs_dir = '/configs'

    # Load camera extrinsics from calibration.yaml (managed by host UI)
    calib_path = os.path.join(configs_dir, 'calibration.yaml')
    with open(calib_path, 'r') as f:
        calib = yaml.safe_load(f) or {}
    cam = calib.get('camera_extrinsics', {})
    imu = calib.get('imu', {})
    gyro_bias = [
        float(imu.get('gyro_bias_x', 0.0)),
        float(imu.get('gyro_bias_y', 0.0)),
        float(imu.get('gyro_bias_z', 0.0)),
    ]

    database_path = LaunchConfiguration('database_path')

    declared_arguments = [
        DeclareLaunchArgument(
            'database_path',
            default_value='/root/.ros/map.db',
            description='Path to the RTAB-Map database file inside the container.',
        ),
    ]

    # Same online IMU bias calibration as mapping.
    imu_calibrator_node = Node(
        package='igvi_imu',
        executable='imu_bias_calibrator',
        name='imu_bias_calibrator',
        output='screen',
        parameters=[
            os.path.join(configs_dir, 'imu_calibrator_kinect.yaml'),
            {'gyro_bias': gyro_bias},
        ],
        remappings=[
            ('imu/in', '/imu'),
            ('imu/out', '/imu/calibrated'),
            ('imu/calibration_state', '/imu/calibration_state'),
        ],
    )

    # Same IMU filter as mapping.
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

    # Camera mount TF from calibration.yaml
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
        imu_calibrator_node,
        imu_filter_node,
        base_to_camera_tf,
        ekf_node,
        rgbd_odom_node,
        rtabmap_loc_node,
    ])
