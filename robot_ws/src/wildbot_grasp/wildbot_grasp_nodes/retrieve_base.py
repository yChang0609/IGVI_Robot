#!/usr/bin/env python3
import json
import math
import threading
import time

import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from rclpy.action import ActionClient, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import String

from wildbot_grasp.action import GrabObject
from wildbot_grasp_nodes.motion import ArmCommander


class RetrieveBase(Node):
    """Shared infrastructure and helpers for retrieve task servers."""

    _feedback_class = None  # subclass must set to the correct Feedback type

    def __init__(
        self,
        node_name: str,
        *,
        standoff_distance: float = 0.22,
        visual_servo_kp: float = 0.002,
        visual_servo_timeout: float = 12.0,
    ):
        super().__init__(node_name)

        self.declare_parameter("standoff_distance", standoff_distance)
        self.declare_parameter("visual_servo_kp", visual_servo_kp)
        self.declare_parameter("visual_servo_timeout", visual_servo_timeout)
        self.declare_parameter("visual_servo_tolerance_px", 15.0)
        self.declare_parameter("image_center_x", 640.0)
        self.declare_parameter("nav_server_timeout", 30.0)
        self.declare_parameter("arrival_tolerance", 0.10)
        self.declare_parameter("arrival_timeout", 45.0)
        self.declare_parameter("approach_target_distance_m", 0.24)
        self.declare_parameter("approach_linear_speed", 0.05)
        self.declare_parameter("approach_timeout_sec", 20.0)

        self.callback_group = ReentrantCallbackGroup()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.arm = ArmCommander(self)

        self.memory_lock = threading.Lock()
        self.detections_lock = threading.Lock()
        self.latest_memory = {}
        self.latest_detections = []

        self.create_subscription(
            String, "/semantic_memory", self.memory_callback, 10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            String, "/detections", self.detections_callback, 10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            String, "/detections_json", self.detections_callback, 10,
            callback_group=self.callback_group,
        )
        self.cmd_vel_pub = self.create_publisher(Twist, "/motion/cmd", 10)
        self.nav_client = ActionClient(
            self, NavigateToPose, "navigate_to_pose",
            callback_group=self.callback_group,
        )
        self.path_client = ActionClient(
            self, ComputePathToPose, "compute_path_to_pose",
            callback_group=self.callback_group,
        )
        self.grab_client = ActionClient(
            self, GrabObject, "grab_object",
            callback_group=self.callback_group,
        )

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------

    def memory_callback(self, msg):
        try:
            data = json.loads(msg.data)
            with self.memory_lock:
                self.latest_memory = {obj["id"]: obj for obj in data.get("objects", [])}
        except Exception as exc:
            self.get_logger().error(f"Error parsing semantic memory: {exc}")

    def detections_callback(self, msg):
        try:
            data = json.loads(msg.data)
            detections = data.get("detections", data) if isinstance(data, dict) else data
            with self.detections_lock:
                self.latest_detections = detections if isinstance(detections, list) else []
        except Exception:
            return

    # ------------------------------------------------------------------
    # Action server callbacks (shared)
    # ------------------------------------------------------------------

    def cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def publish_feedback(self, goal_handle, stage: str, progress: float, detail: str):
        feedback = self._feedback_class()
        feedback.stage = stage
        feedback.progress = float(progress)
        feedback.detail = detail
        goal_handle.publish_feedback(feedback)
        self.get_logger().info(f"{stage}: {detail}")

    # ------------------------------------------------------------------
    # Geometry helpers
    # ------------------------------------------------------------------

    def get_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time(), rclpy.duration.Duration(seconds=1.0)
            )
            q = tf.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            return tf.transform.translation.x, tf.transform.translation.y, yaw
        except Exception as exc:
            self.get_logger().warning(f"Could not get robot pose: {exc}")
            return None

    def quaternion_from_yaw(self, yaw: float):
        half = yaw / 2.0
        return 0.0, 0.0, math.sin(half), math.cos(half)

    def make_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        qx, qy, qz, qw = self.quaternion_from_yaw(float(yaw))
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    # ------------------------------------------------------------------
    # Navigation
    # ------------------------------------------------------------------

    def navigate_to_pose(self, goal_handle, pose: PoseStamped, stage: str, progress: float):
        timeout = float(self.get_parameter("nav_server_timeout").value)
        if not self.nav_client.wait_for_server(timeout_sec=timeout):
            return False, "NavigateToPose action server not available"

        self.publish_feedback(
            goal_handle, stage, progress,
            f"Navigating to x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}",
        )
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = pose
        send_future = self.nav_client.send_goal_async(nav_goal)
        while rclpy.ok() and not send_future.done():
            if goal_handle.is_cancel_requested:
                return False, "mission canceled"
            time.sleep(0.05)

        nav_goal_handle = send_future.result()
        if not nav_goal_handle.accepted:
            return False, "Nav2 rejected goal"

        result_future = nav_goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            if goal_handle.is_cancel_requested:
                nav_goal_handle.cancel_goal_async()
                return False, "mission canceled"
            time.sleep(0.1)

        wrapped_result = result_future.result()
        if wrapped_result is None:
            return False, "Nav2 returned no result"
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            return False, f"Nav2 goal ended with status {wrapped_result.status}"

        return self.wait_until_arrived(goal_handle, pose)

    def wait_until_arrived(self, goal_handle, pose: PoseStamped):
        tolerance = float(self.get_parameter("arrival_tolerance").value)
        timeout = float(self.get_parameter("arrival_timeout").value)
        deadline = time.monotonic() + timeout
        goal_x = float(pose.pose.position.x)
        goal_y = float(pose.pose.position.y)
        last_dist = None

        while rclpy.ok() and time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                self.cmd_vel_pub.publish(Twist())
                return False, "mission canceled"
            robot_pose = self.get_robot_pose()
            if robot_pose is not None:
                dist = math.hypot(goal_x - robot_pose[0], goal_y - robot_pose[1])
                last_dist = dist
                if dist <= tolerance:
                    return True, f"arrived within {dist:.2f}m"
            time.sleep(0.1)

        if last_dist is None:
            return False, "navigation timed out; robot pose unavailable"
        return False, f"navigation timed out; still {last_dist:.2f}m from goal"

    # ------------------------------------------------------------------
    # Semantic memory
    # ------------------------------------------------------------------

    def find_target_from_memory(self, target_class: str, timeout_sec: float):
        """Find best target by class_name (used by BridgeRetrieveServer)."""
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            with self.memory_lock:
                matches = [
                    obj for obj in self.latest_memory.values()
                    if obj.get("class_name") == target_class and obj.get("position")
                ]
            if matches:
                return max(matches, key=lambda obj: (obj.get("score", 0.0), obj.get("id", "")))
            time.sleep(0.1)
        return None

    def find_target_by_id(self, target_id: str, timeout_sec: float = 5.0):
        """Find target by its memory id (used by SearchRetrieveServer)."""
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            with self.memory_lock:
                if target_id in self.latest_memory:
                    return self.latest_memory[target_id]
            time.sleep(0.1)
        return None

    # ------------------------------------------------------------------
    # Approach planning
    # ------------------------------------------------------------------

    def choose_approach_pose(self, target_pos: dict):
        if not self.path_client.wait_for_server(timeout_sec=5.0):
            return None, "compute_path_to_pose action server not available"
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            return None, "Could not get robot pose for path planning"

        start_pose = self.make_pose(robot_pose[0], robot_pose[1], robot_pose[2])
        standoff = float(self.get_parameter("standoff_distance").value)
        bx, by = float(target_pos["x"]), float(target_pos["y"])
        rx, ry = robot_pose[0], robot_pose[1]
        best_len = float("inf")
        best_pose = None

        all_angles = [0, 45, 90, 135, 180, -135, -90, -45]
        # Prefer standoffs on the same side of the bear as the robot.
        # dot(standoff - bear, robot - bear) >= 0 means they are on the same side.
        # This prevents the planner from choosing an approach that goes through the bear
        # (which is not in the costmap and thus appears as free space to Nav2).
        same_side = [a for a in all_angles
                     if (math.cos(math.radians(a)) * (rx - bx)
                         + math.sin(math.radians(a)) * (ry - by)) >= 0]
        candidates = same_side if same_side else all_angles

        for angle_deg in candidates:
            angle_rad = math.radians(angle_deg)
            cx = bx + standoff * math.cos(angle_rad)
            cy = by + standoff * math.sin(angle_rad)
            goal_pose = self.make_pose(cx, cy, angle_rad + math.pi)

            req = ComputePathToPose.Goal()
            req.use_start = True
            req.start = start_pose
            req.goal = goal_pose
            future = self.path_client.send_goal_async(req)
            while rclpy.ok() and not future.done():
                time.sleep(0.01)

            path_goal_handle = future.result()
            if path_goal_handle is None or not path_goal_handle.accepted:
                continue

            result_future = path_goal_handle.get_result_async()
            while rclpy.ok() and not result_future.done():
                time.sleep(0.01)

            result = result_future.result()
            resp = result.result if result is not None else None
            if resp is None or not resp.path.poses:
                continue

            path_len = 0.0
            prev = start_pose.pose.position
            for p in resp.path.poses:
                path_len += math.hypot(p.pose.position.x - prev.x, p.pose.position.y - prev.y)
                prev = p.pose.position
            if path_len < best_len:
                best_len = path_len
                best_pose = goal_pose

        if best_pose is None:
            return None, "No valid path found to target"
        return best_pose, f"selected approach path length {best_len:.2f}m"

    # ------------------------------------------------------------------
    # Visual alignment
    # ------------------------------------------------------------------

    def visual_align(self, goal_handle, target_class: str):
        center_x = float(self.get_parameter("image_center_x").value)
        tolerance = float(self.get_parameter("visual_servo_tolerance_px").value)
        kp = float(self.get_parameter("visual_servo_kp").value)
        timeout = float(self.get_parameter("visual_servo_timeout").value)
        deadline = time.monotonic() + timeout

        while rclpy.ok() and time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                self.cmd_vel_pub.publish(Twist())
                return False, "mission canceled"

            det_center = None
            with self.detections_lock:
                detections = list(self.latest_detections)
            for det in detections:
                if det.get("class_name") == target_class:
                    det_center = det.get("bbox", {}).get("center_x")
                    break

            if det_center is None:
                self.cmd_vel_pub.publish(Twist())
                time.sleep(0.1)
                continue

            error = center_x - float(det_center)
            if abs(error) <= tolerance:
                self.cmd_vel_pub.publish(Twist())
                return True, f"target centered; error={error:.1f}px"

            twist = Twist()
            twist.angular.z = max(-0.3, min(0.3, error * kp))
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)

        self.cmd_vel_pub.publish(Twist())
        return False, "visual alignment timed out; continuing"

    # ------------------------------------------------------------------
    # Visual approach: drive toward target until at grab distance
    # ------------------------------------------------------------------

    def visual_approach(self, goal_handle, target_class: str):
        """Slowly drive forward while centering on the target until
        the detection depth reaches approach_target_distance_m."""
        target_dist = float(self.get_parameter("approach_target_distance_m").value)
        linear_speed = float(self.get_parameter("approach_linear_speed").value)
        kp = float(self.get_parameter("visual_servo_kp").value)
        center_x = float(self.get_parameter("image_center_x").value)
        timeout = float(self.get_parameter("approach_timeout_sec").value)
        deadline = time.monotonic() + timeout

        while rclpy.ok() and time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                self.cmd_vel_pub.publish(Twist())
                return False, "mission canceled"

            with self.detections_lock:
                detections = list(self.latest_detections)

            best = None
            for det in detections:
                label = str(det.get("class_name") or det.get("class_id") or "")
                if label.lower() != target_class.lower():
                    continue
                if not det.get("depth_valid"):
                    continue
                depth = det.get("depth_m")
                if depth is None:
                    continue
                if best is None or depth < best.get("depth_m", float("inf")):
                    best = det

            if best is None:
                # Target not visible — stop and wait
                self.cmd_vel_pub.publish(Twist())
                time.sleep(0.1)
                continue

            depth = float(best["depth_m"])
            if depth <= target_dist:
                self.cmd_vel_pub.publish(Twist())
                return True, f"reached target at {depth:.2f}m"

            # Still approaching: center + drive forward
            det_cx = best.get("bbox", {}).get("center_x", center_x)
            angular = max(-0.3, min(0.3, (center_x - float(det_cx)) * kp))
            twist = Twist()
            twist.linear.x = linear_speed
            twist.angular.z = angular
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)

        self.cmd_vel_pub.publish(Twist())
        return False, f"visual approach timed out after {timeout:.0f}s"

    # ------------------------------------------------------------------
    # Grasp and release
    # ------------------------------------------------------------------

    def call_grab_object(self, goal_handle, target_class: str):
        if not self.grab_client.wait_for_server(timeout_sec=5.0):
            return False, "grab_object action server not available"

        grab_goal = GrabObject.Goal()
        grab_goal.object_label = target_class
        grab_goal.distance_m = 0.0
        send_future = self.grab_client.send_goal_async(grab_goal)
        while rclpy.ok() and not send_future.done():
            if goal_handle.is_cancel_requested:
                return False, "mission canceled"
            time.sleep(0.05)

        grab_goal_handle = send_future.result()
        if not grab_goal_handle.accepted:
            return False, "grab_object goal was rejected"

        result_future = grab_goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            if goal_handle.is_cancel_requested:
                grab_goal_handle.cancel_goal_async()
                return False, "mission canceled"
            time.sleep(0.1)

        wrapped = result_future.result()
        if wrapped is None:
            return False, "grab_object returned no result"
        r = wrapped.result
        if not r.object_grasped:
            return False, f"grab failed: {r.message}"
        return True, r.message

    def release_arm(self, goal_handle):
        self.arm.publish_named("release_object", "place_pose_deg")
        deadline = time.monotonic() + self.arm.motion_wait_sec()
        while rclpy.ok() and time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                return False, "mission canceled"
            time.sleep(0.05)
        return True, "object released"
