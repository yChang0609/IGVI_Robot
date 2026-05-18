import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    configs_dir = '/configs'

    calib_path = os.path.join(configs_dir, 'calibration.yaml')
    with open(calib_path, 'r') as f:
        calib = yaml.safe_load(f) or {}
    imu = calib.get('imu', {})
    gyro_bias = [
        float(imu.get('gyro_bias_x', 0.0)),
        float(imu.get('gyro_bias_y', 0.0)),
        float(imu.get('gyro_bias_z', 0.0)),
    ]

    kinect_node = Node(
        package='azure_kinect_ros_driver',
        executable='node',
        name='azure_kinect',
        output='screen',
        parameters=[{
            'depth_enabled': True,
            'color_enabled': True,
            'color_resolution': '720P',
            # NFOV_2X2BINNED: 320x288 depth (4x fewer pixels than NFOV_UNBINNED's
            # 640x576) → cuts the depth-to-RGB registration latency that was
            # driving the ~475 ms camera pipeline delay seen by rtabmap. Same
            # FOV (75°x65°), slightly extended range (0.5–5.46 m vs 0.5–3.86 m).
            # Trade-off: small distant obstacles less precise. Plenty for SLAM.
            'depth_mode': 'NFOV_2X2BINNED',
            'fps': 15,
            'imu_rate_target': 0,
            'point_cloud': True,
            'rgb_point_cloud': True,
        }],
    )

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
            ('calibration/start', '/imu/calibration/start'),
        ],
    )

    return LaunchDescription([
        kinect_node,
        imu_calibrator_node,
    ])
