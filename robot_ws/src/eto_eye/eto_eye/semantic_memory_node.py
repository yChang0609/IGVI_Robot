#!/usr/bin/env python3
import json
import math
import time
import uuid

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo, Image
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

        # === 參數設定 ===
        self.declare_parameter('target_frame', 'odom')           
        self.declare_parameter('distance_threshold', 0.1)        
        self.declare_parameter('memory_timeout_sec', 10.0)       
        self.declare_parameter('position_alpha', 0.3)            
        self.declare_parameter('depth_topic', '/depth_to_rgb/image_raw')
        self.declare_parameter('depth_unit_scale', 0.001)

        self.target_frame = self.get_parameter('target_frame').value
        self.dist_thresh = self.get_parameter('distance_threshold').value
        self.timeout_sec = self.get_parameter('memory_timeout_sec').value
        self.alpha = self.get_parameter('position_alpha').value
        self.depth_scale = self.get_parameter('depth_unit_scale').value

        # === TF2 & 相機模型設定 ===
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.bridge = CvBridge()
        self.cam_model = PinholeCameraModel()
        self.latest_depth_img = None
        self.camera_frame_id = None

        # 記憶體結構 { object_id: {"class_name": str, "x": float, "y": float, "z": float, "last_seen": float, "hits": int} }
        self.memory = {}

        # === 訂閱與發布 ===
        self.create_subscription(CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 10)
        self.create_subscription(Image, self.get_parameter('depth_topic').value, self.depth_callback, 10)
        self.create_subscription(String, '/detections', self.detection_callback, 10)
        
        self.memory_pub = self.create_publisher(String, '/semantic_memory', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/semantic_memory_markers', 10)

        # 定期清理過期或消失的記憶
        self.create_timer(1.0, self.cleanup_memory)
        self.get_logger().info(f"Semantic Memory Node started. Target frame: {self.target_frame}")

    def camera_info_callback(self, msg: CameraInfo):
        if not self.cam_model.initialized():
            self.cam_model.fromCameraInfo(msg)
            self.get_logger().info("Camera model initialized.")

    def depth_callback(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.latest_depth_img = np.asarray(cv_image)
        self.camera_frame_id = msg.header.frame_id

    def detection_callback(self, msg: String):
        if not self.cam_model.initialized():
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
        now_sec = time.time()

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, frame_id, msg_time, timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except TransformException as ex:
            return

        for det in detections:
            if not det.get('depth_valid'):
                continue

            class_name = det.get('class_name', str(det.get('class_id')))
            center_x = det['bbox']['center_x']
            center_y = det['bbox']['center_y']
            depth_z = det['depth_m']

            # 1. 將 2D 像素透過相機模型轉為 3D 射線並乘上深度
            ray = self.cam_model.projectPixelTo3dRay((center_x, center_y))
            cam_x = ray[0] * (depth_z / ray[2])
            cam_y = ray[1] * (depth_z / ray[2])
            cam_z = depth_z

            # 2. 轉換至世界座標
            point_cam = PointStamped()
            point_cam.header.frame_id = frame_id
            point_cam.header.stamp = msg_time.to_msg()
            point_cam.point.x, point_cam.point.y, point_cam.point.z = cam_x, cam_y, cam_z

            point_world = tf2_geometry_msgs.do_transform_point(point_cam, transform)
            self.associate_and_update(class_name, point_world.point.x, point_world.point.y, point_world.point.z, now_sec)

        self.publish_memory()

    def associate_and_update(self, class_name, x, y, z, now_sec):
        closest_id = None
        min_dist = float('inf')

        for obj_id, obj_data in self.memory.items():
            if obj_data['class_name'] == class_name:
                dist = math.sqrt((obj_data['x']-x)**2 + (obj_data['y']-y)**2 + (obj_data['z']-z)**2)
                if dist < min_dist:
                    min_dist = dist
                    closest_id = obj_id

        if closest_id is not None and min_dist < self.dist_thresh:
            self.memory[closest_id]['x'] = self.alpha * x + (1 - self.alpha) * self.memory[closest_id]['x']
            self.memory[closest_id]['y'] = self.alpha * y + (1 - self.alpha) * self.memory[closest_id]['y']
            self.memory[closest_id]['z'] = self.alpha * z + (1 - self.alpha) * self.memory[closest_id]['z']
            self.memory[closest_id]['last_seen'] = now_sec
            self.memory[closest_id]['hits'] += 1
        else:
            new_id = str(uuid.uuid4())[:8]
            self.memory[new_id] = {
                'class_name': class_name, 'x': x, 'y': y, 'z': z, 'last_seen': now_sec, 'hits': 1
            }

    def cleanup_memory(self):
        if self.latest_depth_img is None or self.camera_frame_id is None or not self.cam_model.initialized():
            return

        now_sec = time.time()
        expired_ids = []
        img_height, img_width = self.latest_depth_img.shape[:2]

        try:
            # 取得「當下」的座標狀態，用來判定視野
            transform_world_to_cam = self.tf_buffer.lookup_transform(
                self.camera_frame_id, self.target_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except TransformException:
            return

        for obj_id, obj_data in self.memory.items():
            time_since_last_seen = now_sec - obj_data['last_seen']
            if time_since_last_seen < 2.0:
                continue # 容錯期
                
            if time_since_last_seen > self.timeout_sec * 5:
                expired_ids.append(obj_id) # 絕對超時
                continue

            point_world = PointStamped()
            point_world.header.frame_id = self.target_frame
            point_world.point.x, point_world.point.y, point_world.point.z = obj_data['x'], obj_data['y'], obj_data['z']
            
            point_cam = tf2_geometry_msgs.do_transform_point(point_world, transform_world_to_cam)
            cam_x, cam_y, cam_z = point_cam.point.x, point_cam.point.y, point_cam.point.z

            if cam_z <= 0.1:
                continue # 在相機背後

            u, v = self.cam_model.project3dToPixel((cam_x, cam_y, cam_z))
            u, v = int(round(u)), int(round(v))

            margin = 30
            if margin <= u < img_width - margin and margin <= v < img_height - margin:
                roi = self.latest_depth_img[v-2:v+3, u-2:u+3].astype(np.float32)
                valid_depths = roi[(roi > 0) & np.isfinite(roi)] * self.depth_scale
                
                if valid_depths.size > 0:
                    actual_depth = np.median(valid_depths)
                    if actual_depth < cam_z - 0.25:
                        continue # 被前方物體遮擋，保留記憶

                self.get_logger().info(f"Object missing confirmed: {obj_data['class_name']} [{obj_id}]")
                expired_ids.append(obj_id)

        for obj_id in expired_ids:
            if obj_id in self.memory:
                del self.memory[obj_id]

    def publish_memory(self):
        memory_list = []
        for obj_id, obj_data in self.memory.items():
            if obj_data['hits'] >= 3:
                memory_list.append({
                    "id": obj_id, "class_name": obj_data['class_name'],
                    "position": {"x": obj_data['x'], "y": obj_data['y'], "z": obj_data['z']}
                })
        
        msg = String()
        msg.data = json.dumps({"target_frame": self.target_frame, "objects": memory_list})
        self.memory_pub.publish(msg)

        marker_array = MarkerArray()
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        for i, (obj_id, obj_data) in enumerate(self.memory.items()):
            if obj_data['hits'] < 3: continue

            m = Marker()
            m.header.frame_id = self.target_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "semantic_objects"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = obj_data['x'], obj_data['y'], obj_data['z']
            m.scale.x = m.scale.y = m.scale.z = 0.15
            m.color.a = 0.8; m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0

            t = Marker()
            t.header = m.header
            t.ns = "semantic_labels"
            t.id = i + 1000
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x, t.pose.position.y, t.pose.position.z = obj_data['x'], obj_data['y'], obj_data['z'] + 0.2
            t.scale.z = 0.1
            t.color.a = 1.0; t.color.r = 1.0; t.color.g = 1.0; t.color.b = 1.0
            t.text = obj_data['class_name']

            marker_array.markers.append(m)
            marker_array.markers.append(t)

        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = SemanticMemoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
