#!/usr/bin/env python3
import json
import math
import os
import time
import uuid

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import Empty, String
from sensor_msgs.msg import CameraInfo, Image, JointState
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped

from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel

import tf2_ros
from tf2_ros import TransformException
import tf2_geometry_msgs


class SemanticMemoryNode(Node):
    def __init__(self):
        super().__init__('semantic_memory_node')

        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('merge_radius_m', 0.4)
        self.declare_parameter('memory_timeout_sec', 600.0)
        self.declare_parameter('position_alpha', 0.3)
        self.declare_parameter('position_alpha_close', 0.7)
        self.declare_parameter('depth_topic', '/depth_to_rgb/image_raw')
        self.declare_parameter('depth_unit_scale', 0.001)
        self.declare_parameter('detection_topic', '/detections_json')
        self.declare_parameter('camera_info_topic', '/rgb/camera_info')

        self.target_frame = self.get_parameter('target_frame').value
        self.merge_radius = float(self.get_parameter('merge_radius_m').value)
        self.timeout_sec = self.get_parameter('memory_timeout_sec').value
        self.alpha = self.get_parameter('position_alpha').value
        self.alpha_close = self.get_parameter('position_alpha_close').value
        self.depth_scale = self.get_parameter('depth_unit_scale').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.bridge = CvBridge()
        self.cam_model = PinholeCameraModel()
        self.camera_info_received = False
        self.latest_depth_img = None
        self.camera_frame_id = None
        self.latest_depth_stamp = None

        # Arm state and shared parameters for dynamic bottom-half image masking
        env_home = os.environ.get("ARM_HOME_POSE_DEG")
        if env_home:
            try:
                default_home = [float(x.strip()) for x in env_home.split(",")]
            except Exception:
                default_home = [190.0, 0.0, 240.0]
        else:
            default_home = [190.0, 0.0, 240.0]

        env_tol = os.environ.get("ARM_JOINT_TOLERANCE_DEG")
        default_tol = float(env_tol) if env_tol else 10.0

        env_mask_pct = os.environ.get("ARM_MASK_HEIGHT_PCT")
        default_mask_pct = float(env_mask_pct) if env_mask_pct else 0.50

        self.declare_parameter('home_pose_deg', default_home)
        self.declare_parameter('joint_tolerance_deg', default_tol)
        self.declare_parameter('mask_height_pct', default_mask_pct)

        home_deg = self.get_parameter('home_pose_deg').value
        self.arm_1_home = math.radians(float(home_deg[0]))
        self.arm_2_home = math.radians(float(home_deg[1]))
        self.gripper_home = math.radians(float(home_deg[2]))
        self.tolerance = math.radians(self.get_parameter('joint_tolerance_deg').value)
        self.mask_height_pct = self.get_parameter('mask_height_pct').value

        self.arm_is_home = True

        # { object_id: {class_name, x, y, z, last_seen, hits, score, last_depth_m} }
        self.memory = {}
        self.memory_lock = __import__('threading').Lock()

        self.callback_group = ReentrantCallbackGroup()

        self.create_subscription(
            CameraInfo,
            self.get_parameter('camera_info_topic').value,
            self.camera_info_callback,
            10
        )
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10,
            callback_group=self.callback_group
        )
        self.create_subscription(
            String,
            self.get_parameter('detection_topic').value,
            self.detection_callback,
            10,
            callback_group=self.callback_group
        )
        # Continuous Dynamic Subscription variables for Depth topic
        self.current_subscribed_depth_type = None
        self.depth_sub = None

        # Background timer for continuous dynamic depth routing
        self.create_timer(1.0, self.check_depth_topic_and_subscribe)

        self.memory_pub = self.create_publisher(String, '/semantic_memory', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/semantic_memory_markers', 10)

        self.create_subscription(Empty, '/semantic_memory/clear', self._on_clear, 10,
                                 callback_group=self.callback_group)
        self.create_subscription(String, '/semantic_memory/remove', self._on_remove, 10,
                                 callback_group=self.callback_group)

        self.create_timer(1.0, self.cleanup_memory)
        self.create_timer(0.1, self.publish_memory)

        # Publish empty memory immediately so any bridge/UI caches reset on restart.
        self.publish_memory()

        self.get_logger().info(
            f"Semantic Memory Node started. frame={self.target_frame} "
            f"merge_radius={self.merge_radius}m"
        )

    def check_depth_topic_and_subscribe(self):
        filtered_topic = "/depth_to_rgb/image_filtered"
        try:
            num_publishers = self.count_publishers(filtered_topic)
            has_filtered = num_publishers > 0
        except Exception:
            has_filtered = False

        target_type = "filtered" if has_filtered else "raw"

        if self.current_subscribed_depth_type != target_type:
            self._do_depth_subscribe(target_type)

    def _do_depth_subscribe(self, target_type):
        # 1. Destroy existing subscription
        if self.depth_sub is not None:
            self.destroy_subscription(self.depth_sub)
            self.depth_sub = None

        # 2. Determine topic name
        if target_type == "filtered":
            depth_topic = "/depth_to_rgb/image_filtered"
        else:
            depth_topic = self.get_parameter('depth_topic').value

        # 3. Create new subscription
        self.depth_sub = self.create_subscription(
            Image,
            depth_topic,
            self.depth_callback,
            10
        )
        self.get_logger().info(
            f"Semantic Memory dynamically switched depth input to {target_type} topic: {depth_topic}"
        )
        self.current_subscribed_depth_type = target_type

    def _on_clear(self, _msg: Empty):
        with self.memory_lock:
            count = len(self.memory)
            self.memory.clear()
        self.get_logger().info(f"semantic memory cleared ({count} objects)")

    def _on_remove(self, msg: String):
        target_id = msg.data
        with self.memory_lock:
            if target_id in self.memory:
                del self.memory[target_id]
                self.get_logger().info(f"[Semantic Memory] Removed object {target_id} from semantic memory via topic request")
        # Publish memory immediately after removal to ensure fast updates
        self.publish_memory()

    def camera_info_callback(self, msg: CameraInfo):
        if not self.camera_info_received:
            self.cam_model.fromCameraInfo(msg)
            self.camera_info_received = True

    def depth_callback(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.latest_depth_img = np.asarray(cv_image)
        self.camera_frame_id = msg.header.frame_id
        self.latest_depth_stamp = msg.header.stamp

    def joint_states_callback(self, msg: JointState):
        joint_map = dict(zip(msg.name, msg.position))
        arm_1 = joint_map.get('arm_1_joint')
        arm_2 = joint_map.get('arm_2_joint')
        gripper = joint_map.get('gripper_joint')

        if arm_1 is None or arm_2 is None or gripper is None:
            return

        dev_1 = abs(arm_1 - self.arm_1_home)
        dev_2 = abs(arm_2 - self.arm_2_home)
        dev_g = abs(gripper - self.gripper_home)

        is_home = (dev_1 <= self.tolerance) and (dev_2 <= self.tolerance) and (dev_g <= self.tolerance)
        if is_home != self.arm_is_home:
            self.arm_is_home = is_home
            state_str = "HOME (All Detections Active)" if is_home else "ACTIVE (Bottom Masking Enabled)"
            self.get_logger().info(
                f"Arm state transition: {state_str}. "
                f"Joints: arm_1={math.degrees(arm_1):.1f}°, arm_2={math.degrees(arm_2):.1f}°, gripper={math.degrees(gripper):.1f}°"
            )

    def detection_callback(self, msg: String):
        if not self.camera_info_received:
            return

        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        frame_id = data.get('frame_id')
        detections = data.get('detections', [])
        if not detections or not frame_id:
            return

        msg_time = Time(seconds=data['stamp']['sec'], nanoseconds=data['stamp']['nanosec'])
        now_sec = msg_time.nanoseconds / 1e9

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, frame_id, msg_time, timeout=rclpy.duration.Duration(seconds=0.5)
            )
        except TransformException as ex:
            self.get_logger().warning(f"TF timeout: {ex}")
            return

        for det in detections:
            if not det.get('depth_valid'):
                continue

            depth_z = det['depth_m']
            # Filter out detections that are too close (depth < 0.45 meters).
            # This prevents adding the carried object (bear in gripper) inside the robot footprint
            # into spatial memory as a false ground detection while the robot is navigating.
            if depth_z < 0.45:
                continue

            class_name = det.get('class_name', str(det.get('class_id')))
            center_x = det['bbox']['center_x']
            center_y = det['bbox']['center_y']
            score = det.get('score', 0.0)

            # If the arm is active (not home) and the detection center_y is in the bottom masked region of the image,
            # we reject it entirely to prevent the carried object or raised arm from generating false spatial memory entries.
            if not self.arm_is_home:
                img_height = self.cam_model.height if (self.cam_model and self.cam_model.height) else 720
                mask_start_y = img_height * (1.0 - self.mask_height_pct)
                if center_y > mask_start_y:
                    self.get_logger().info(f"Rejecting detection in bottom half (y={center_y:.1f}/{img_height}) because arm is active.")
                    continue

            ray = self.cam_model.projectPixelTo3dRay((center_x, center_y))
            cam_x = ray[0] * (depth_z / ray[2])
            cam_y = ray[1] * (depth_z / ray[2])
            cam_z = depth_z

            point_cam = PointStamped()
            point_cam.header.frame_id = frame_id
            point_cam.header.stamp = msg_time.to_msg()
            point_cam.point.x, point_cam.point.y, point_cam.point.z = cam_x, cam_y, cam_z

            point_world = tf2_geometry_msgs.do_transform_point(point_cam, transform)
            self.associate_and_update(
                class_name,
                point_world.point.x, point_world.point.y, point_world.point.z,
                now_sec, score, depth_z
            )

    def associate_and_update(self, class_name, x, y, z, now_sec, score, depth_z=None):
        with self.memory_lock:
            closest_id = None
            min_dist = float('inf')

            for obj_id, obj_data in self.memory.items():
                if obj_data['class_name'] == class_name:
                    dist = math.sqrt((obj_data['x'] - x) ** 2 + (obj_data['y'] - y) ** 2)
                    if dist < min_dist:
                        min_dist = dist
                        closest_id = obj_id

            if closest_id is not None and min_dist < self.merge_radius:
                obj = self.memory[closest_id]
                # When the new observation is closer to the camera than the last
                # recorded depth, it is more accurate — use a higher alpha so the
                # position converges faster to the fresh, close-range reading.
                last_depth = obj.get('last_depth_m')
                if depth_z is not None and last_depth is not None and depth_z < last_depth:
                    alpha = self.alpha_close
                else:
                    alpha = self.alpha
                obj['x'] = alpha * x + (1 - alpha) * obj['x']
                obj['y'] = alpha * y + (1 - alpha) * obj['y']
                obj['z'] = alpha * z + (1 - alpha) * obj['z']
                obj['last_seen'] = now_sec
                obj['hits'] += 1
                obj['score'] = 0.5 * score + 0.5 * obj.get('score', score)
                if depth_z is not None:
                    obj['last_depth_m'] = depth_z
            else:
                new_id = str(uuid.uuid4())[:8]
                self.memory[new_id] = {
                    'class_name': class_name,
                    'x': x, 'y': y, 'z': z,
                    'last_seen': now_sec,
                    'hits': 1,
                    'score': score,
                    'last_depth_m': depth_z,
                }

    def cleanup_memory(self):
        if (self.latest_depth_img is None or self.camera_frame_id is None
                or not self.camera_info_received or self.latest_depth_stamp is None):
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9
        expired_ids = []
        img_height, img_width = self.latest_depth_img.shape[:2]

        try:
            transform_world_to_cam = self.tf_buffer.lookup_transform(
                self.camera_frame_id, self.target_frame,
                self.latest_depth_stamp, timeout=rclpy.duration.Duration(seconds=0.5)
            )
        except TransformException as e:
            self.get_logger().warning(f"cleanup TF failed: {e}")
            return

        with self.memory_lock:
            for obj_id, obj_data in self.memory.items():
                time_since_last_seen = now_sec - obj_data['last_seen']
                if time_since_last_seen < 2.0:
                    continue

                if time_since_last_seen > self.timeout_sec:
                    expired_ids.append(obj_id)
                    continue

                point_world = PointStamped()
                point_world.header.frame_id = self.target_frame
                point_world.point.x = obj_data['x']
                point_world.point.y = obj_data['y']
                point_world.point.z = obj_data['z']

                point_cam = tf2_geometry_msgs.do_transform_point(point_world, transform_world_to_cam)
                cam_x, cam_y, cam_z = point_cam.point.x, point_cam.point.y, point_cam.point.z

                if cam_z <= 0.1:
                    continue

                u, v = self.cam_model.project3dToPixel((cam_x, cam_y, cam_z))
                u, v = int(round(u)), int(round(v))

                margin = 30
                if margin <= u < img_width - margin and margin <= v < img_height - margin:
                    v_start = max(0, v - 2)
                    v_end = min(img_height, v + 3)
                    u_start = max(0, u - 2)
                    u_end = min(img_width, u + 3)
                    roi = self.latest_depth_img[v_start:v_end, u_start:u_end].astype(np.float32)
                    valid_depths = roi[(roi > 0) & np.isfinite(roi)] * self.depth_scale

                    if valid_depths.size > 0:
                        actual_depth = np.median(valid_depths)
                        if actual_depth < cam_z - 0.25:
                            continue
                        expired_ids.append(obj_id)

            for obj_id in expired_ids:
                if obj_id in self.memory:
                    obj_data = self.memory[obj_id]
                    time_since_seen = now_sec - obj_data['last_seen']
                    if time_since_seen > self.timeout_sec:
                        reason = f"timeout (last seen {time_since_seen:.1f}s ago, limit {self.timeout_sec:.1f}s)"
                    else:
                        reason = "missing at expected location (FOI check shows it should be visible but is gone/missing)"
                    self.get_logger().info(
                        f"[Semantic Memory] Deleting object {obj_id} ({obj_data['class_name']}) from memory. Reason: {reason}"
                    )
                    del self.memory[obj_id]

    def publish_memory(self):
        memory_list = []

        with self.memory_lock:
            for obj_id, obj_data in self.memory.items():
                if obj_data['hits'] >= 3:
                    memory_list.append({
                        "id": obj_id,
                        "class_name": obj_data['class_name'],
                        "score": obj_data.get('score', 0.0),
                        "position": {"x": obj_data['x'], "y": obj_data['y'], "z": obj_data['z']}
                    })

        msg = String()
        msg.data = json.dumps({"target_frame": self.target_frame, "objects": memory_list})
        self.memory_pub.publish(msg)

        marker_array = MarkerArray()

        with self.memory_lock:
            for obj_id, obj_data in self.memory.items():
                if obj_data['hits'] < 3:
                    continue

                stable_int_id = hash(obj_id) % 2147483647

                m = Marker()
                m.header.frame_id = self.target_frame
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = "semantic_objects"
                m.id = stable_int_id
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose.position.x = obj_data['x']
                m.pose.position.y = obj_data['y']
                m.pose.position.z = obj_data['z']
                m.scale.x = m.scale.y = m.scale.z = 0.15
                m.color.a = 0.8
                m.color.r = 0.0
                m.color.g = 1.0
                m.color.b = 0.0
                m.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()

                t = Marker()
                t.header = m.header
                t.ns = "semantic_labels"
                t.id = stable_int_id + 1000000
                t.type = Marker.TEXT_VIEW_FACING
                t.action = Marker.ADD
                t.pose.position.x = obj_data['x']
                t.pose.position.y = obj_data['y']
                t.pose.position.z = obj_data['z'] + 0.2
                t.scale.z = 0.1
                t.color.a = 1.0
                t.color.r = 1.0
                t.color.g = 1.0
                t.color.b = 1.0
                short_id = obj_id[:8] if len(obj_id) > 8 else obj_id
                t.text = f"{obj_data['class_name']} [{short_id}] ({obj_data.get('score', 0.0):.2f})"
                t.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()

                marker_array.markers.append(m)
                marker_array.markers.append(t)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = SemanticMemoryNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
