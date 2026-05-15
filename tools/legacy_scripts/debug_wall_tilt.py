#!/usr/bin/env python3
"""
Debug: print raw depth_camera_link points and their base_link transforms
to diagnose the 34° wall tilt mystery.
"""
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros
from rclpy.duration import Duration


def apply_transform(pts, tf):
    t = tf.transform.translation
    q = tf.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2*(y*y + z*z),  2*(x*y - w*z),      2*(x*z + w*y)],
        [    2*(x*y + w*z),  1 - 2*(x*x + z*z),  2*(y*z - w*x)],
        [    2*(x*z - w*y),  2*(y*z + w*x),      1 - 2*(x*x + y*y)],
    ])
    return (R @ pts.T).T + np.array([t.x, t.y, t.z])


class DebugNode(Node):
    def __init__(self):
        super().__init__('debug_wall_tilt')
        self.buf = tf2_ros.Buffer()
        self.lst = tf2_ros.TransformListener(self.buf, self)
        self.done = False
        self.sub = self.create_subscription(PointCloud2, '/points2', self.cb, 1)
        self.create_timer(0.5, self.check_tf)
        self.ready = False

    def check_tf(self):
        if self.ready:
            return
        try:
            self.buf.lookup_transform('base_link', 'depth_camera_link',
                                      rclpy.time.Time(), Duration(seconds=0.1))
            self.get_logger().info('TF ready')
            self.ready = True
        except Exception:
            pass

    def cb(self, msg):
        if not self.ready or self.done:
            return
        self.done = True

        tf = self.buf.lookup_transform('base_link', msg.header.frame_id,
                                       msg.header.stamp, Duration(seconds=0.5))

        q = tf.transform.rotation
        t = tf.transform.translation
        print(f"\nTF base_link <- {msg.header.frame_id}:")
        print(f"  Translation: [{t.x:.4f}, {t.y:.4f}, {t.z:.4f}]")
        print(f"  Quaternion xyzw: [{q.x:.4f}, {q.y:.4f}, {q.z:.4f}, {q.w:.4f}]")

        x, y, z, w = q.x, q.y, q.z, q.w
        R = np.array([
            [1 - 2*(y*y + z*z),  2*(x*y - w*z),      2*(x*z + w*y)],
            [    2*(x*y + w*z),  1 - 2*(x*x + z*z),  2*(y*z - w*x)],
            [    2*(x*z - w*y),  2*(y*z + w*x),      1 - 2*(x*x + y*y)],
        ])
        print(f"  Rotation matrix:\n{R}")

        raw = np.array([(p[0], p[1], p[2])
                        for p in pc2.read_points(msg, field_names=('x', 'y', 'z'),
                                                  skip_nans=True)], dtype=np.float32)
        print(f"\nRaw points count: {len(raw)}")
        print(f"Raw Z range (depth_camera_link): [{raw[:,2].min():.3f}, {raw[:,2].max():.3f}]")
        print(f"Raw Y range (depth_camera_link): [{raw[:,1].min():.3f}, {raw[:,1].max():.3f}]")
        print(f"Raw X range (depth_camera_link): [{raw[:,0].min():.3f}, {raw[:,0].max():.3f}]")

        # Transform all points
        pts = apply_transform(raw, tf)
        print(f"\nTransformed to base_link:")
        print(f"  X range (forward): [{pts[:,0].min():.3f}, {pts[:,0].max():.3f}]")
        print(f"  Y range (left):    [{pts[:,1].min():.3f}, {pts[:,1].max():.3f}]")
        print(f"  Z range (up):      [{pts[:,2].min():.3f}, {pts[:,2].max():.3f}]")

        # Show floor points
        z_low = np.percentile(pts[:,2], 8)
        floor = pts[pts[:,2] < z_low]
        print(f"\nFloor Z (bottom 8%): {np.median(floor[:,2]):.4f} m")

        # Wall points: Z > 0.05 AND X > 0.1
        wall = pts[(pts[:,2] > 0.05) & (pts[:,0] > 0.1)]
        print(f"Wall point count (Z>0.05, X>0.1): {len(wall)}")
        if len(wall) > 10:
            print(f"  Wall X range: [{wall[:,0].min():.3f}, {wall[:,0].max():.3f}]")
            print(f"  Wall Z range: [{wall[:,2].min():.3f}, {wall[:,2].max():.3f}]")

            # Fit X vs Z
            a, b = np.polyfit(wall[:,2], wall[:,0], 1)
            tilt_deg = np.degrees(np.arctan(a))
            print(f"  Wall tilt (dX/dZ): slope={a:.4f}, angle={tilt_deg:.2f}°")

            # Show representative wall points sorted by Z
            idx = np.argsort(wall[:,2])
            step = max(1, len(idx)//10)
            print("\n  Sample wall points (sorted by Z):")
            print("    X_bl    Z_bl  | raw_x   raw_y   raw_z   (depth_cam_link)")
            for i in idx[::step][:10]:
                # Find closest raw point
                rx, ry, rz = raw[np.where((pts[:,0] == wall[i,0]) &
                                           (pts[:,1] == wall[i,1]))[0][0]] if False else (0,0,0)
                print(f"    {wall[i,0]:+.4f}  {wall[i,2]:+.4f}")

        rclpy.shutdown()


def main():
    rclpy.init()
    rclpy.spin(DebugNode())

if __name__ == '__main__':
    main()
