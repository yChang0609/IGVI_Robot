#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ComputePathToPose
from wildbot_grasp.action import SearchAndRetrieve, GrabObject

import json
import math
import time
import tf2_ros
import threading

class SearchRetrieveServer(Node):
    def __init__(self):
        super().__init__('search_retrieve_server')
        
        self.declare_parameter('standoff_distance', 0.25)
        self.declare_parameter('visual_servo_kp', 0.002)
        self.declare_parameter('visual_servo_timeout', 15.0)
        self.declare_parameter('visual_servo_tolerance_px', 15.0)
        self.declare_parameter('image_center_x', 320.0)
        
        self.callback_group = ReentrantCallbackGroup()
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.memory_sub = self.create_subscription(
            String, '/semantic_memory', self.memory_callback, 10, callback_group=self.callback_group)
        self.detections_sub = self.create_subscription(
            String, '/detections', self.detections_callback, 10, callback_group=self.callback_group)
            
        self.cmd_vel_pub = self.create_publisher(Twist, '/motion/cmd', 10)
        
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=self.callback_group)
        self.grab_client = ActionClient(self, GrabObject, 'grab_object', callback_group=self.callback_group)
        self.path_client = self.create_client(ComputePathToPose, 'compute_path_to_pose', callback_group=self.callback_group)
        
        self.action_server = ActionServer(
            self,
            SearchAndRetrieve,
            'search_retrieve',
            execute_callback=self.execute_callback,
            callback_group=self.callback_group,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        
        self.latest_memory = {}
        self.latest_detections = []
        self.memory_lock = threading.Lock()
        self.detections_lock = threading.Lock()
        
        self.get_logger().info('Ready: /search_retrieve')

    def memory_callback(self, msg):
        try:
            data = json.loads(msg.data)
            with self.memory_lock:
                self.latest_memory = {obj['id']: obj for obj in data.get('objects', [])}
        except Exception as e:
            self.get_logger().error(f"Error parsing semantic memory: {e}")

    def detections_callback(self, msg):
        try:
            data = json.loads(msg.data)
            with self.detections_lock:
                self.latest_detections = data
        except Exception as e:
            self.get_logger().error(f"Error parsing detections: {e}")

    def goal_callback(self, goal_request):
        self.get_logger().info(f"Received goal to search and retrieve target: {goal_request.target_id}")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Goal canceled")
        return CancelResponse.ACCEPT

    def publish_feedback(self, goal_handle, stage: str, progress: float, detail: str):
        feedback = SearchAndRetrieve.Feedback()
        feedback.stage = stage
        feedback.progress = float(progress)
        feedback.detail = detail
        goal_handle.publish_feedback(feedback)
        self.get_logger().info(f"{stage}: {detail}")

    def get_robot_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time(), rclpy.duration.Duration(seconds=1.0))
            x = t.transform.translation.x
            y = t.transform.translation.y
            
            q = t.transform.rotation
            yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
            return x, y, yaw
        except Exception as e:
            self.get_logger().warning(f"Could not get robot pose: {e}")
            return None

    def quaternion_from_euler(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return qx, qy, qz, qw

    async def execute_callback(self, goal_handle):
        result = SearchAndRetrieve.Result()
        goal = goal_handle.request
        target_id = goal.target_id

        # 1. Look for target in memory
        self.publish_feedback(goal_handle, "searching", 0.05, f"Looking for {target_id} in semantic memory")
        target_pos = None
        target_name = ""
        
        # Wait up to 5 seconds for target to appear in memory
        for _ in range(50):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                return result
                
            with self.memory_lock:
                if target_id in self.latest_memory:
                    obj = self.latest_memory[target_id]
                    target_pos = obj['position']
                    target_name = obj['class_name']
                    break
            time.sleep(0.1)
            
        if target_pos is None:
            result.success = False
            result.message = f"Target {target_id} not found in memory"
            goal_handle.abort()
            return result

        self.publish_feedback(goal_handle, "evaluating", 0.1, f"Found {target_name}. Evaluating approach paths.")

        # 2. Evaluate 8 approaches
        standoff = float(self.get_parameter('standoff_distance').value)
        best_path_len = float('inf')
        best_pose = None
        
        angles = [0, 45, 90, 135, 180, -135, -90, -45]
        
        if not self.path_client.wait_for_service(timeout_sec=5.0):
            result.success = False
            result.message = "compute_path_to_pose service not available"
            goal_handle.abort()
            return result

        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            result.success = False
            result.message = "Could not get robot pose for path planning"
            goal_handle.abort()
            return result

        start_pose = PoseStamped()
        start_pose.header.frame_id = 'map'
        start_pose.pose.position.x = robot_pose[0]
        start_pose.pose.position.y = robot_pose[1]

        for angle_deg in angles:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                return result

            angle_rad = math.radians(angle_deg)
            # The standoff point is r distance away in the angle direction
            cx = target_pos['x'] + standoff * math.cos(angle_rad)
            cy = target_pos['y'] + standoff * math.sin(angle_rad)
            
            # The robot needs to face the target (which is opposite to the angle from target)
            yaw = angle_rad + math.pi
            
            req = ComputePathToPose.Request()
            req.start = start_pose
            
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.pose.position.x = cx
            goal_pose.pose.position.y = cy
            qx, qy, qz, qw = self.quaternion_from_euler(0, 0, yaw)
            goal_pose.pose.orientation.x = qx
            goal_pose.pose.orientation.y = qy
            goal_pose.pose.orientation.z = qz
            goal_pose.pose.orientation.w = qw
            
            req.goal = goal_pose
            
            future = self.path_client.call_async(req)
            while rclpy.ok() and not future.done():
                time.sleep(0.01)
                
            path_resp = future.result()
            if path_resp is not None and len(path_resp.path.poses) > 0:
                # Calculate path length
                path_len = 0.0
                prev_p = start_pose.pose.position
                for p in path_resp.path.poses:
                    path_len += math.hypot(p.pose.position.x - prev_p.x, p.pose.position.y - prev_p.y)
                    prev_p = p.pose.position
                    
                self.get_logger().info(f"Angle {angle_deg}: length {path_len:.2f}m")
                if path_len < best_path_len:
                    best_path_len = path_len
                    best_pose = goal_pose

        if best_pose is None:
            result.success = False
            result.message = "No valid path found to any standoff point"
            goal_handle.abort()
            return result

        # 3. Navigate to pose
        self.publish_feedback(goal_handle, "navigating", 0.3, "Navigating to optimal standoff point")
        
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            result.success = False
            result.message = "NavigateToPose action server not available"
            goal_handle.abort()
            return result
            
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = best_pose
        
        send_goal_future = self.nav_client.send_goal_async(nav_goal)
        while rclpy.ok() and not send_goal_future.done():
            time.sleep(0.1)
            
        nav_goal_handle = send_goal_future.result()
        if not nav_goal_handle.accepted:
            result.success = False
            result.message = "Nav2 rejected the goal"
            goal_handle.abort()
            return result

        get_result_future = nav_goal_handle.get_result_async()
        
        while rclpy.ok() and not get_result_future.done():
            if goal_handle.is_cancel_requested:
                nav_goal_handle.cancel_goal_async()
                goal_handle.canceled()
                result.success = False
                return result
            time.sleep(0.1)
            
        # 4. Visual Servoing Alignment
        self.publish_feedback(goal_handle, "aligning", 0.6, "Visual servoing to center target")
        
        center_x_target = float(self.get_parameter('image_center_x').value)
        tolerance_px = float(self.get_parameter('visual_servo_tolerance_px').value)
        kp = float(self.get_parameter('visual_servo_kp').value)
        timeout = float(self.get_parameter('visual_servo_timeout').value)
        
        start_time = time.time()
        aligned = False
        
        while rclpy.ok() and (time.time() - start_time) < timeout:
            if goal_handle.is_cancel_requested:
                self.cmd_vel_pub.publish(Twist()) # Stop
                goal_handle.canceled()
                result.success = False
                return result
                
            det_center = None
            with self.detections_lock:
                for det in self.latest_detections:
                    # Match by class_name for now, since detections don't carry stable memory IDs yet
                    if det.get('class_name') == target_name:
                        det_center = det.get('bbox', {}).get('center_x')
                        break
                        
            if det_center is not None:
                error = center_x_target - det_center
                if abs(error) <= tolerance_px:
                    aligned = True
                    break
                    
                twist = Twist()
                twist.angular.z = max(-0.3, min(0.3, error * kp))
                self.cmd_vel_pub.publish(twist)
            else:
                self.cmd_vel_pub.publish(Twist()) # Stop if target lost
                
            time.sleep(0.1)
            
        self.cmd_vel_pub.publish(Twist()) # Ensure stopped
        
        if not aligned:
            self.get_logger().warning("Visual servoing timed out or target not seen. Continuing anyway.")

        # 5. Grab Object (Simulated)
        self.publish_feedback(goal_handle, "grabbing", 0.8, "Simulating grab for 5 seconds")
        
        # --- REAL GRAB CALL (COMMENTED OUT AS REQUESTED) ---
        # if self.grab_client.wait_for_server(timeout_sec=2.0):
        #     grab_goal = GrabObject.Goal()
        #     grab_goal.object_label = target_name
        #     grab_goal.distance_m = 0.2
        #     grab_future = self.grab_client.send_goal_async(grab_goal)
        #     while rclpy.ok() and not grab_future.done():
        #         time.sleep(0.1)
        #     grab_gh = grab_future.result()
        #     if grab_gh.accepted:
        #         res_future = grab_gh.get_result_async()
        #         while rclpy.ok() and not res_future.done():
        #             if goal_handle.is_cancel_requested:
        #                 grab_gh.cancel_goal_async()
        #                 goal_handle.canceled()
        #                 result.success = False
        #                 return result
        #             time.sleep(0.1)
        # ---------------------------------------------------
        
        # Simulation delay
        for _ in range(50):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                return result
            time.sleep(0.1)

        # 6. Return Home
        self.publish_feedback(goal_handle, "returning", 0.9, "Returning to specified home pose")
        
        home_pose = PoseStamped()
        home_pose.header.frame_id = 'map'
        home_pose.pose.position.x = goal.home_pose_x
        home_pose.pose.position.y = goal.home_pose_y
        qx, qy, qz, qw = self.quaternion_from_euler(0, 0, goal.home_pose_yaw)
        home_pose.pose.orientation.x = qx
        home_pose.pose.orientation.y = qy
        home_pose.pose.orientation.z = qz
        home_pose.pose.orientation.w = qw
        
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = home_pose
        
        send_goal_future = self.nav_client.send_goal_async(nav_goal)
        while rclpy.ok() and not send_goal_future.done():
            time.sleep(0.1)
            
        nav_goal_handle = send_goal_future.result()
        if nav_goal_handle.accepted:
            get_result_future = nav_goal_handle.get_result_async()
            while rclpy.ok() and not get_result_future.done():
                if goal_handle.is_cancel_requested:
                    nav_goal_handle.cancel_goal_async()
                    goal_handle.canceled()
                    result.success = False
                    return result
                time.sleep(0.1)

        # Simulated Release
        self.publish_feedback(goal_handle, "releasing", 0.98, "Releasing object")
        time.sleep(2.0)

        result.success = True
        result.message = "Successfully retrieved object and returned"
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = SearchRetrieveServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
