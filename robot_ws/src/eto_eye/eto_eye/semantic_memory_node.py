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
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

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
        self.declare_parameter('target_frame', 'map')           
        self.declare_parameter('distance_threshold', 0.1)        
        self.declare_parameter('memory_timeout_sec', 600.0)       
        self.declare_parameter('position_alpha', 0.3)            
        self.declare_parameter('depth_topic', '/depth_to_rgb/image_raw')
        self.declare_parameter('depth_unit_scale', 0.001)
        self.declare_parameter('detection_topic', '/detections_json')
        self.declare_parameter('camera_info_topic', '/rgb/camera_info')

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
        self.camera_info_received = False
        self.latest_depth_img = None
        self.camera_frame_id = None
        self.latest_depth_stamp = None

        # 記憶體結構 { object_id: {"class_name": str, "x": float, "y": float, "z": float, "last_seen": float, "hits": int} }
        self.memory = {}
        self.memory_lock = __import__('threading').Lock()

        # === 訂閱與發布 ===
        # 建立一個允許平行處理的群組
        self.callback_group = ReentrantCallbackGroup()

        self.create_subscription(
            CameraInfo, 
            self.get_parameter('camera_info_topic').value, 
            self.camera_info_callback, 
            10
        )
        self.create_subscription(
            String, 
            self.get_parameter('detection_topic').value, 
            self.detection_callback, 
            10,
            callback_group=self.callback_group
        )
        self.create_subscription(
            Image, 
            self.get_parameter('depth_topic').value, 
            self.depth_callback, 
            10
        )
        self.create_subscription(String, '/detections', self.detection_callback, 10)
        
        self.memory_pub = self.create_publisher(String, '/semantic_memory', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/semantic_memory_markers', 10)

        # 定期清理過期或消失的記憶
        self.create_timer(1.0, self.cleanup_memory)
        
        # [新增] 定期發布記憶狀態 (0.1 秒一次，等於 10 Hz)
        self.create_timer(0.1, self.publish_memory)
        
        self.get_logger().debug(f"Semantic Memory Node started. Target frame: {self.target_frame}")

    def camera_info_callback(self, msg: CameraInfo):
        if not self.camera_info_received:
            self.cam_model.fromCameraInfo(msg)
            self.camera_info_received = True
            self.get_logger().debug("Camera model initialized.")

    def depth_callback(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.latest_depth_img = np.asarray(cv_image)
        self.camera_frame_id = msg.header.frame_id
        
        # [新增] 把這張深度圖專屬的時間戳記存下來
        self.latest_depth_stamp = msg.header.stamp

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
        # 改用 YOLO 影像本身的「精準時間」，取代電腦的處理時間
        now_sec = msg_time.nanoseconds / 1e9

        try:
            # 改回 msg_time，並給予 0.5 秒的耐心等待 SLAM 更新
            transform = self.tf_buffer.lookup_transform(
                self.target_frame, frame_id, msg_time, timeout=rclpy.duration.Duration(seconds=0.5)
            )
        except TransformException as ex:
            self.get_logger().warning(f"【TF 等待超時】 無法取得精準對齊的座標: {ex}")
            return

        for det in detections:
            if not det.get('depth_valid'):
                continue 

            class_name = det.get('class_name', str(det.get('class_id')))
            center_x = det['bbox']['center_x']
            center_y = det['bbox']['center_y']
            depth_z = det['depth_m']
            score = det.get('score', 0.0)

            ray = self.cam_model.projectPixelTo3dRay((center_x, center_y))
            cam_x = ray[0] * (depth_z / ray[2])
            cam_y = ray[1] * (depth_z / ray[2])
            cam_z = depth_z

            point_cam = PointStamped()
            point_cam.header.frame_id = frame_id
            point_cam.header.stamp = msg_time.to_msg()
            point_cam.point.x, point_cam.point.y, point_cam.point.z = cam_x, cam_y, cam_z

            # 1. 這裡計算出 point_world (注意縮排：前面有 12 個空格)
            point_world = tf2_geometry_msgs.do_transform_point(point_cam, transform)
            
            # 2. 存入記憶庫 (注意縮排：必須跟 point_world 對齊！前面有 12 個空格)
            self.associate_and_update(class_name, point_world.point.x, point_world.point.y, point_world.point.z, now_sec, score)
            
            # 3. 印出 Log (注意縮排：必須對齊！前面有 12 個空格)
            self.get_logger().debug(f"【記憶更新】 成功將 {class_name} 寫入 {self.target_frame} 世界坐標系！")

        # 4. 發布狀態 (注意縮排：這裡退回去了！前面只有 8 個空格)
        # self.publish_memory()  <--- 【刪除或註解這一行！】

    def associate_and_update(self, class_name, x, y, z, now_sec, score):
        with self.memory_lock:
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
                old_score = self.memory[closest_id].get('score', score)
                self.memory[closest_id]['score'] = 0.5 * score + 0.5 * old_score
            else:
                new_id = str(uuid.uuid4())[:8]
                self.memory[new_id] = {
                    'class_name': class_name, 'x': x, 'y': y, 'z': z, 'last_seen': now_sec, 'hits': 1, 'score': score
                }

    def cleanup_memory(self):
        if self.latest_depth_img is None or self.camera_frame_id is None or not self.camera_info_received or self.latest_depth_stamp is None:
            return

        # 這裡改用 ROS 專屬時鐘，確保支援 Gazebo 模擬時間與 Rosbag 暫停
        now_sec = self.get_clock().now().nanoseconds / 1e9
        expired_ids = []
        img_height, img_width = self.latest_depth_img.shape[:2]

        try:
            # [修改] 使用深度圖的專屬時間去查 TF，達到 100% 時空對齊
            transform_world_to_cam = self.tf_buffer.lookup_transform(
                self.camera_frame_id, self.target_frame, self.latest_depth_stamp, timeout=rclpy.duration.Duration(seconds=0.5)
            )
        except TransformException as e:
            # === 抓出兇手 3：如果 TF 失敗，印出警告 ===
            self.get_logger().warning(f"【清理異常】 TF 轉換失敗，暫停清理記憶: {e}")
            return

        with self.memory_lock:
            for obj_id, obj_data in self.memory.items():
                time_since_last_seen = now_sec - obj_data['last_seen']
                if time_since_last_seen < 2.0:
                    continue # 容錯期
                    
                # === 修復兇手 1：拿掉 * 5，只要超過 600 秒沒更新就強制刪除 ===
                if time_since_last_seen > self.timeout_sec:
                    self.get_logger().debug(f"【超時遺忘】 {obj_data['class_name']} [{obj_id}] 超過 600 秒未見，強制刪除！")
                    expired_ids.append(obj_id) 
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
                    v_start = max(0, v - 2)
                    v_end = min(img_height, v + 3)
                    u_start = max(0, u - 2)
                    u_end = min(img_width, u + 3)
                    roi = self.latest_depth_img[v_start:v_end, u_start:u_end].astype(np.float32)
                    valid_depths = roi[(roi > 0) & np.isfinite(roi)] * self.depth_scale
                    
                    # ==== 修正的邏輯區塊 ====
                    if valid_depths.size > 0:
                        actual_depth = np.median(valid_depths)
                        if actual_depth < cam_z - 0.1:
                            self.get_logger().debug(f"【保留記憶】 {obj_data['class_name']} [{obj_id}] 疑似被前方物體遮擋。")
                            continue # 被前方物體遮擋，保留記憶
    
                        # 只有在「確定測到有效深度，且沒有被遮擋」的情況下，才確認消失
                        self.get_logger().debug(f"【確認消失】 視野內該位置已淨空，立刻刪除 {obj_data['class_name']} [{obj_id}]！")
                        expired_ids.append(obj_id)
                    else:
                        # 深度圖在該位置剛好破洞、反光或超出感測範圍
                        # 為了安全起見，我們假裝沒看見，讓它繼續保留在記憶裡，等待 10 秒超時
                        # self.get_logger().debug(f"【深度無效】 {obj_data['class_name']} [{obj_id}] 無法判斷是否消失。")
                        pass
                else:
                    # 不在視野內，保留等待 timeout_sec (10秒)
                    pass
        
            for obj_id in expired_ids:
                if obj_id in self.memory:
                    del self.memory[obj_id]

    def publish_memory(self):
        memory_list = []
        
        with self.memory_lock:
            # === Debug 發布檢查 ===
            self.get_logger().debug(f"【Debug 發布檢查】 當前記憶庫共有 {len(self.memory)} 個物件")
            
            for obj_id, obj_data in self.memory.items():
                self.get_logger().debug(f"  -> {obj_data['class_name']} [{obj_id}]: hits={obj_data['hits']}, 座標=({obj_data['x']:.2f}, {obj_data['y']:.2f}, {obj_data['z']:.2f})")
                
                # 目前設定為 hits >= 1 (看過 1 次就發布，方便 Debug)
                # 等系統穩定後，建議改回 3 以過濾閃爍雜訊
                if obj_data['hits'] >= 1:
                    memory_list.append({
                        "id": obj_id, 
                        "class_name": obj_data['class_name'],
                        "score": obj_data.get('score', 0.0),
                        "position": {"x": obj_data['x'], "y": obj_data['y'], "z": obj_data['z']}
                    })

        # === 1. 發布 JSON 訊息 ===
        msg = String()
        msg.data = json.dumps({"target_frame": self.target_frame, "objects": memory_list})
        self.memory_pub.publish(msg)

        # === 2. 發布 RViz Marker ===
        marker_array = MarkerArray()

        with self.memory_lock:
            for obj_id, obj_data in self.memory.items():
                if obj_data['hits'] < 1: 
                    continue
    
                # 產生穩定的整數 ID，確保 RViz 知道這是同一個物件，直接覆蓋更新而不產生殘影
                stable_int_id = hash(obj_id) % 2147483647 
    
                # --- 球體 Marker ---
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
                m.scale.x = m.scale.y = m.scale.z = 0.15 # 15 公分大小
                m.color.a = 0.8
                m.color.r = 0.0
                m.color.g = 1.0
                m.color.b = 0.0
                # 加入壽命：10 秒內沒收到新的更新，RViz 就會自動刪除它
                m.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()
    
                # --- 文字標籤 Marker ---
                t = Marker()
                t.header = m.header
                t.ns = "semantic_labels"
                t.id = stable_int_id + 1000000  # 文字的 ID 加上偏移量避免與球體 ID 衝突
                t.type = Marker.TEXT_VIEW_FACING
                t.action = Marker.ADD
                t.pose.position.x = obj_data['x']
                t.pose.position.y = obj_data['y']
                t.pose.position.z = obj_data['z'] + 0.2 # 浮在球體正上方 20 公分處
                t.scale.z = 0.1
                t.color.a = 1.0
                t.color.r = 1.0
                t.color.g = 1.0
                t.color.b = 1.0
                short_id = obj_id[:8] if len(obj_id) > 8 else obj_id
                t.text = f"{obj_data['class_name']} [{short_id}] ({obj_data.get('score', 0.0):.2f})"
                t.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg() # 同樣加上 10 秒壽命
    
                marker_array.markers.append(m)
                marker_array.markers.append(t)

        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = SemanticMemoryNode()
    
    # 啟用多執行緒引擎，讓 TF 接收與 Callback 等待可以同時進行
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
