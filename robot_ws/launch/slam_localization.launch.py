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

    database_path = LaunchConfiguration('database_path')

    declared_arguments = [
        DeclareLaunchArgument(
            'database_path',
            default_value='/root/.ros/map.db',
            description='Path to the RTAB-Map database file inside the container.',
        ),
    ]

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
            ('rgb/image',       '/rgb/image_slam'),
            ('rgb/camera_info', '/rgb/camera_info'),
            ('depth/image',     '/depth_to_rgb/image_slam'),
            ('imu',             '/imu/filtered'),
            ('odom',            '/odom_visual'),
        ],
    )

    # ICP laser odometry — same as slam_fusion. Cross-references wheel odom
    # via scan-matching of the back 200° of the lidar. EKF fuses its vx/vy
    # with wheel vx + IMU gyro_z. Drift-resistant on carpet/skid-steer.
    icp_odom_node = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        output='screen',
        parameters=[{
            'frame_id':              'base_link',
            'odom_frame_id':         'odom',
            'publish_tf':            False,
            'expected_update_rate':  15.0,
            'wait_for_transform':    0.2,
            'Reg/Strategy':          '1',
            'Reg/Force3DoF':         'true',
            'Icp/Iterations':        '10',
            'Icp/VoxelSize':         '0.05',
            'Icp/PointToPlane':      'true',
            'Icp/Epsilon':           '0.001',
            'Icp/MaxCorrespondenceDistance': '0.1',
            'Icp/CorrespondenceRatio':       '0.05',
            'Icp/RangeMin':          '0.1',
            'Icp/RangeMax':          '12.0',
            'Odom/ResetCountdown':   '0',
            'Odom/Strategy':         '0',
            'Odom/ScanKeyFrameThr':  '0.7',
        }],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odom_lidar'),
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
            'subscribe_scan':            True,   # lidar refines relocalization (see Reg/Strategy=2)
            'approx_sync':               True,
            # Kinect frames arrive ~475 ms after their timestamp (driver +
            # depth->RGB registration). At 50 Hz odom, a queue of 10 only holds
            # 200 ms of history → odom messages matching the camera stamp are
            # dropped before the frame arrives. 100 = 2 s of backlog, plenty.
            'sync_queue_size':           100,
            'topic_queue_size':          100,
            'Mem/IncrementalMemory':     'False',
            'Mem/InitWMWithAllNodes':    'True',
            'Reg/Strategy':              '2',   # 2 = Visual + ICP: BoW finds relocalization candidates, lidar verifies
            'Reg/Force3DoF':             'true',
            'RGBD/NeighborLinkRefining': 'true',
            'Grid/Sensor':               '1',
            'Grid/CellSize':             '0.05',
            'Grid/MaxGroundAngle':       '45',
            'Grid/MaxObstacleHeight':    '2.0',
            'Grid/MinGroundHeight':      '-0.1',
            'Mem/UseOdomGravity':        'true',
            'Optimizer/GravitySigma':    '0.25',
            'RTAB-Map/TimeThr':          '0',
            'RTAB-Map/DetectionRate':    '5.0',   # start at 5 Hz; raise toward 10 only if CPU/latency allows
            'Mem/STMSize':               '10',    # localization needs little short-term memory
            'RGBD/LinearUpdate':         '0.05',  # re-localize every 5 cm
            'RGBD/AngularUpdate':        '0.02',  # re-localize every ~1.1°
            # Keep /map frozen to the clean stored grid: do NOT redraw it from
            # live depth every cycle (that baked transient obstacles into the
            # persistent map permanently). Live/dynamic obstacle avoidance is
            # handled by the Nav2 costmap voxel_layer (Kinect /points2 + rear
            # /scan) instead, which marks and ray-traces-clear on its own.
            'map_always_update':         False,
            'map_empty_ray_tracing':     False,
        }],
        remappings=[
            ('rgb/image',       '/rgb/image_slam'),
            ('rgb/camera_info', '/rgb/camera_info'),
            ('depth/image',     '/depth_to_rgb/image_slam'),
            ('scan',            '/scan'),
            # RTAB-Map localizes on the fused EKF odom, publishes only map->odom.
            ('odom',            '/odometry/filtered'),
        ],
    )

    # Lightweight dynamic topic router for C++ SLAM nodes
    camera_router_node = Node(
        package='wildbot_grasp',
        executable='camera_router_node',
        name='camera_router',
        output='screen'
    )

    return LaunchDescription(declared_arguments + [
        imu_filter_node,
        base_to_camera_tf,
        ekf_node,
        rgbd_odom_node,
        icp_odom_node,
        rtabmap_loc_node,
        camera_router_node,
    ])
