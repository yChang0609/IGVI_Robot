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

    return LaunchDescription([imu_calibrator_node])
