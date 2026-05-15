#!/usr/bin/env python3
"""
Camera mount calibration checker.
Place the robot on a flat floor, 1 m from a flat wall facing forward.
Run this and it tells you exactly what to change in docker/compose/compose.yaml.
"""
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros
from rclpy.duration import Duration
from geometry_msgs.msg import TransformStamped


WALL_DISTANCE_M = 1.0   # how far the wall is from base_link (tape measure)


def apply_transform(pts: np.ndarray, tf: TransformStamped) -> np.ndarray:
    t = tf.transform.translation
    q = tf.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2*(y*y + z*z),  2*(x*y - w*z),      2*(x*z + w*y)],
        [    2*(x*y + w*z),  1 - 2*(x*x + z*z),  2*(y*z - w*x)],
        [    2*(x*z - w*y),  2*(y*z + w*x),      1 - 2*(x*x + y*y)],
    ])
    return (R @ pts.T).T + np.array([t.x, t.y, t.z])


class MountChecker(Node):
    def __init__(self):
        super().__init__('mount_checker')
        self.buf = tf2_ros.Buffer()
        self.lst = tf2_ros.TransformListener(self.buf, self)
        self.ready = False
        self.samples = []
        self.sub = self.create_subscription(PointCloud2, '/points2', self.cb, 1)
        # Wait until TF is actually stable before accepting any point cloud
        self.create_timer(0.5, self.check_tf)
        self.get_logger().info('Waiting for TF to stabilise ...')

    def check_tf(self):
        if self.ready:
            return
        try:
            # /points2 header says depth_camera_link but with rgb_point_cloud:=true
            # the coordinates are actually in rgb_camera_link space.
            # Use rgb_camera_link for the correct transform.
            self.buf.lookup_transform('base_link', 'rgb_camera_link',
                                      rclpy.time.Time(), Duration(seconds=0.1))
            self.get_logger().info('TF ready — collecting point clouds ...')
            self.ready = True
        except Exception:
            pass

    def cb(self, msg):
        if not self.ready:
            return
        try:
            tf = self.buf.lookup_transform(
                'base_link', 'rgb_camera_link',
                msg.header.stamp, Duration(seconds=0.5))
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return

        raw = np.array([(p[0], p[1], p[2])
                        for p in pc2.read_points(msg, field_names=('x', 'y', 'z'),
                                                  skip_nans=True)],
                       dtype=np.float32)
        if len(raw) < 100:
            return

        pts = apply_transform(raw, tf)
        self.samples.append(pts)

        if len(self.samples) >= 5:       # accumulate 5 frames for stability
            self.report(np.vstack(self.samples))
            rclpy.shutdown()

    def report(self, pts: np.ndarray):
        # ── Floor: points whose Z is in the bottom 8 % ─────────────────────
        z_low = np.percentile(pts[:, 2], 8)
        floor_pts = pts[pts[:, 2] < z_low]
        floor_z   = float(np.median(floor_pts[:, 2]))

        # ── Wall: find the dominant flat surface facing the robot ───────────
        # Step 1: find rough wall X using all above-floor, in-front-of-robot points.
        # Threshold relative to actual floor_z so it's robust to camera_mount_z changes.
        above_floor = pts[(pts[:, 2] > floor_z + 0.05) & (pts[:, 0] > 0.1)]
        if len(above_floor) > 50:
            # The wall is the nearest large cluster — use the 20th percentile of X
            # (robust to distant background objects pulling the median out)
            rough_wall_x = float(np.percentile(above_floor[:, 0], 20))
            wall_x_med   = rough_wall_x

            # Step 2: isolate the wall by keeping only points within ±30 cm of that X
            # This removes ceiling, far floor, and other room clutter from the tilt fit.
            wall_band = above_floor[np.abs(above_floor[:, 0] - rough_wall_x) < 0.30]

            if len(wall_band) > 50:
                # Re-estimate wall X from the band
                wall_x_med = float(np.median(wall_band[:, 0]))
                # Tilt: fit X = a*Z + b on wall-band points
                zz = wall_band[:, 2]
                xx = wall_band[:, 0]
                a, b = np.polyfit(zz, xx, 1)   # a = dX/dZ
                tilt_deg = float(np.degrees(np.arctan(a)))
            else:
                tilt_deg = None
        else:
            wall_x_med = None
            tilt_deg   = None

        # ── Print results ───────────────────────────────────────────────────
        print()
        print('=' * 58)
        print('  FLOOR CHECK')
        dz = -floor_z
        print(f'    Floor Z (median):    {floor_z:+.3f} m   (ideal: 0.000)')
        if abs(dz) < 0.005:
            print('    camera_mount_z  ✓  (error < 5 mm)')
        else:
            sign = '+' if dz > 0 else ''
            print(f'    → change camera_mount_z by {sign}{dz:.3f} m')

        print()
        print('  WALL CHECK  (robot must be exactly 1.0 m from flat wall)')
        if wall_x_med is not None:
            dx = WALL_DISTANCE_M - wall_x_med
            print(f'    Wall X (median):     {wall_x_med:+.3f} m   (ideal: {WALL_DISTANCE_M:.3f})')
            if abs(dx) < 0.01:
                print('    camera_mount_x  ✓  (error < 1 cm)')
            else:
                sign = '+' if dx > 0 else ''
                print(f'    → change camera_mount_x by {sign}{dx:.3f} m')
        else:
            print('    No wall points found above floor — move robot closer to wall')

        print()
        print('  WALL TILT CHECK  (diagnoses camera_mount_pitch error)')
        if tilt_deg is not None:
            print(f'    dX/dZ slope of wall:  {tilt_deg:+.2f}°   (ideal: 0.00°)')
            if abs(tilt_deg) < 1.0:
                print('    camera_mount_pitch  ✓  (wall appears vertical)')
            else:
                pitch_fix = float(np.radians(-tilt_deg))
                print(f'    Wall is tilted {abs(tilt_deg):.1f}° → '
                      f'adjust camera_mount_pitch by {pitch_fix:+.3f} rad')
        else:
            print('    Not enough wall points to check tilt')
        print('=' * 58)
        print()

def main():
    rclpy.init()
    rclpy.spin(MountChecker())

if __name__ == '__main__':
    main()
