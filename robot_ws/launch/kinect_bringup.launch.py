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

    kinect = calib.get('kinect', {})
    kinect_node = Node(
        package='azure_kinect_ros_driver',
        executable='node',
        name='azure_kinect',
        output='screen',
        parameters=[{
            'depth_enabled': True,
            'color_enabled': True,
            'color_resolution': kinect.get('color_resolution', '720P'),
            'depth_mode': kinect.get('depth_mode', 'NFOV_UNBINNED'),
            'fps': int(kinect.get('fps', 15)),
            'imu_rate_target': int(kinect.get('imu_rate_target', 200)),
            'point_cloud': True,
            'rgb_point_cloud': False,
            'exposure_time_absolute': int(kinect.get('exposure_time_absolute', -1)),
            'gain': int(kinect.get('gain', -1)),
            'white_balance': int(kinect.get('white_balance', -1)),
            'brightness': int(kinect.get('brightness', 128)),
            'contrast': int(kinect.get('contrast', 5)),
            'saturation': int(kinect.get('saturation', 32)),
            'sharpness': int(kinect.get('sharpness', 2)),
            'backlight_compensation': bool(kinect.get('backlight_compensation', False)),
            'powerline_frequency': int(kinect.get('powerline_frequency', 60)),
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
