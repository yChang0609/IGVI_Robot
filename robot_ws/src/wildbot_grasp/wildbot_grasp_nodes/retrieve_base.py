#!/usr/bin/env python3
import json
import math
import threading
import time

import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
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
    ):
        super().__init__(node_name)

        self.declare_parameter("standoff_distance", standoff_distance)
        self.declare_parameter("visual_servo_kp", visual_servo_kp)
        self.declare_parameter("image_center_x", 640.0)
        self.declare_parameter("nav_server_timeout", 30.0)
        import os
        env_arrival_tol = os.environ.get("RETRIEVE_ARRIVAL_TOLERANCE")
        default_arrival_tol = float(env_arrival_tol) if env_arrival_tol else 0.1

        env_yaw_tol = os.environ.get("RETRIEVE_YAW_TOLERANCE")
        default_yaw_tol = float(env_yaw_tol) if env_yaw_tol else 0.1

        self.declare_parameter("arrival_tolerance", default_arrival_tol)
        self.declare_parameter("yaw_tolerance", default_yaw_tol)
        self.declare_parameter("arrival_timeout", 45.0)
        self.declare_parameter("approach_target_distance_m", 0.24)
        self.declare_parameter("approach_linear_speed", 0.05)
        self.declare_parameter("approach_timeout_sec", 20.0)
        # After a successful grab, reverse by however far the visual approach
        # drove the robot in, so it doesn't drag the held object through whatever
        # it approached when navigating away.
        self.declare_parameter("post_grab_backup_speed", 0.10)
        self.declare_parameter("post_grab_backup_timeout_sec", 10.0)
        # Grab only once the bbox is centered within this many px of image_center_x.
        self.declare_parameter("approach_center_tolerance_px", 25.0)
        # Bridge: locate target near the bridge waypoint / scan-rotate to find it.
        self.declare_parameter("bridge_memory_radius_m", 0.6)
        self.declare_parameter("scan_angular_speed", 0.4)
        self.declare_parameter("scan_step_timeout_sec", 15.0)
        self.declare_parameter("scan_total_timeout_sec", 40.0)
        # Stepped door-ref scan: pause per stop + extra rotation past the door ref.
        self.declare_parameter("scan_step_settle_sec", 0.5)
        self.declare_parameter("scan_step_extra_deg", 45.0)
        # face_point: standoff distance to back up to (for camera visibility).
        self.declare_parameter("face_point_distance_m", 0.3)
        # face_point: angular P controller for rotating to face the point.
        self.declare_parameter("face_point_ang_kp", 1.5)
        self.declare_parameter("face_point_ang_max", 0.45)
        self.declare_parameter("face_point_ang_floor", 0.30)
        # face_point: alignment threshold — start adding reverse motion once
        # |yaw_err| drops below this (deg). Until then, rotate only.
        self.declare_parameter("face_point_align_deg", 30.0)
        # face_point: reverse speed once aligned, and final stop yaw tolerance.
        self.declare_parameter("face_point_reverse_speed", 1.0)
        self.declare_parameter("face_point_yaw_tol_deg", 8.0)
        self.declare_parameter("face_point_timeout_sec", 10.0)

        self.callback_group = ReentrantCallbackGroup()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.arm = ArmCommander(self)

        self.memory_lock = threading.Lock()
        self.detections_lock = threading.Lock()
        self.latest_memory = {}
        self.latest_detections = []
        self.motion_state = "idle"

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
        self.create_subscription(
            String, "/motion/state", self.motion_state_callback, 10,
            callback_group=self.callback_group,
        )
        self.cmd_vel_pub = self.create_publisher(Twist, "/motion/cmd", 10)
        self.remove_memory_pub = self.create_publisher(
            String, "/semantic_memory/remove", 10,
            callback_group=self.callback_group,
        )
        self._plan_pub = self.create_publisher(Path, "/plan", 1)
        self._approach_pose_pub = self.create_publisher(PoseStamped, "/approach_pose", 1)
        self._clear_global_client = self.create_client(
            ClearEntireCostmap, "/global_costmap/clear_entirely_global_costmap",
            callback_group=self.callback_group,
        )
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

    def motion_state_callback(self, msg):
        self.motion_state = msg.data

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

    def remove_object_from_memory(self, target_id: str):
        msg = String()
        msg.data = str(target_id)
        self.remove_memory_pub.publish(msg)
        self.get_logger().info(f"Published request to remove object {target_id} from semantic memory")

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
                self._plan_pub.publish(Path())
                self.cmd_vel_pub.publish(Twist())
                return False, "mission canceled"
            time.sleep(0.05)

        nav_goal_handle = send_future.result()
        if not nav_goal_handle.accepted:
            return False, "Nav2 rejected goal"

        result_future = nav_goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            if goal_handle.is_cancel_requested:
                nav_goal_handle.cancel_goal_async()
                self._plan_pub.publish(Path())
                self.cmd_vel_pub.publish(Twist())
                return False, "mission canceled"
            time.sleep(0.1)

        wrapped_result = result_future.result()
        if wrapped_result is None:
            return False, "Nav2 returned no result"
        if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
            return False, f"Nav2 goal ended with status {wrapped_result.status}"

        return self.wait_until_arrived(goal_handle, pose)

    def wait_until_arrived(self, goal_handle, pose: PoseStamped):
        # We now unify arrival checking completely under motion_arbiter's state.
        # Once motion_arbiter returns to 'idle' state, it has successfully completed path tracking
        # and final yaw alignment according to its internal 'goal_tolerance' and 'yaw_tolerance'.
        
        # 1. Wait for motion_arbiter to transition away from 'idle' state.
        # We give it up to 1.5 seconds. If it doesn't transition, we assume
        # the goal is already reached or the plan was completed instantly.
        start_wait = time.time()
        has_started = False
        while time.time() - start_wait < 1.5:
            if goal_handle and goal_handle.is_cancel_requested:
                self._plan_pub.publish(Path())
                self.cmd_vel_pub.publish(Twist())
                return False, "mission canceled"
            if self.motion_state in ("path_tracking", "aligning"):
                has_started = True
                break
            time.sleep(0.05)
            
        start_time = time.time()
        timeout = 60.0
        while rclpy.ok():
            if goal_handle and goal_handle.is_cancel_requested:
                self._plan_pub.publish(Path())
                self.cmd_vel_pub.publish(Twist())
                return False, "mission canceled"
                
            if time.time() - start_time > timeout:
                return False, "navigation timeout"

            if has_started and self.motion_state == "idle":
                return True, "arrived at goal (confirmed by motion_arbiter)"
            elif not has_started and time.time() - start_time > 2.0:
                return True, "already at goal (confirmed by motion_arbiter)"

            time.sleep(0.1)

        return False, "ros shutdown"

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
        best_path = None

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
            deadline = time.monotonic() + 3.0
            while rclpy.ok() and not future.done() and time.monotonic() < deadline:
                time.sleep(0.01)
            if not future.done():
                continue

            path_goal_handle = future.result()
            if path_goal_handle is None or not path_goal_handle.accepted:
                continue

            result_future = path_goal_handle.get_result_async()
            deadline = time.monotonic() + 5.0
            while rclpy.ok() and not result_future.done() and time.monotonic() < deadline:
                time.sleep(0.01)
            if not result_future.done():
                continue

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
                best_path = resp.path

        if best_pose is None:
            return None, "No valid path found to target"

        self._approach_pose_pub.publish(best_pose)
        if best_path is not None:
            self._plan_pub.publish(best_path)

        return best_pose, f"selected approach path length {best_len:.2f}m"

    # ------------------------------------------------------------------
    # Visual approach: drive toward target until at grab distance
    # ------------------------------------------------------------------

    def visual_approach(self, goal_handle, target_class: str):
        """Drive toward the target while centering on the YOLO bbox, then grab
        once it is BOTH within grab distance AND centered left/right.

        Mirrors the bear-task approach with two robustness fixes:
        - centering gate: do not grab until the bbox is centered within
          approach_center_tolerance_px (stops the off-centre / grab-air failures);
        - depth-dropout latch: depth often drops at very close range — once we
          have reached grab distance, stay latched and grab instead of stalling.
        """
        # Clear any stored nav2 path in motion_arbiter. wait_until_arrived
        # returns on position tolerance alone, so the pure-pursuit ALIGNING
        # state can still be holding the original goal yaw/position. Without
        # this, every zero-Twist this loop emits (target lost, in-range stop)
        # falls back to PATH_TRACKING and pulls the robot back toward the
        # original nav goal, fighting the visual servo.
        self._plan_pub.publish(Path())
        time.sleep(0.1)

        target_dist = float(self.get_parameter("approach_target_distance_m").value)
        linear_speed = float(self.get_parameter("approach_linear_speed").value)
        kp = float(self.get_parameter("visual_servo_kp").value)
        center_x = float(self.get_parameter("image_center_x").value)
        tolerance = float(self.get_parameter("approach_center_tolerance_px").value)
        timeout = float(self.get_parameter("approach_timeout_sec").value)
        deadline = time.monotonic() + timeout
        reached_grab_range = False
        # When depth is unavailable but target is centered, track how long we've
        # been blindly approaching so we can trigger grab after a fixed interval.
        no_depth_centered_t0 = None
        no_detection_t0 = None

        def clamp_ang(value):
            return max(-0.3, min(0.3, value))

        while rclpy.ok() and time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                self.cmd_vel_pub.publish(Twist())
                return False, "mission canceled"

            with self.detections_lock:
                detections = list(self.latest_detections)

            best = None
            bbox_cx = None
            for det in detections:
                label = str(det.get("class_name") or det.get("class_id") or "")
                if label.lower() != target_class.lower():
                    continue
                cx = det.get("bbox", {}).get("center_x")
                if bbox_cx is None and cx is not None:
                    bbox_cx = float(cx)   # remember the bbox even without valid depth
                if not det.get("depth_valid"):
                    continue
                depth = det.get("depth_m")
                if depth is None:
                    continue
                if best is None or depth < best.get("depth_m", float("inf")):
                    best = det
                    if cx is not None:
                        bbox_cx = float(cx)

            if best is None:
                # No usable depth this cycle.
                if reached_grab_range:
                    self.cmd_vel_pub.publish(Twist())
                    return True, "depth lost at close range; grabbing (latched)"
                if bbox_cx is not None:
                    error_px = center_x - bbox_cx
                    centered = abs(error_px) <= tolerance
                    twist = Twist()
                    # Always creep forward even when not centered — mirrors the
                    # depth-valid path (drive + correct yaw simultaneously).
                    # Rotating in place without forward motion causes oscillation
                    # because YOLO latency means each correction overshoots.
                    twist.linear.x = linear_speed
                    if centered:
                        if no_depth_centered_t0 is None:
                            no_depth_centered_t0 = time.monotonic()
                        if time.monotonic() - no_depth_centered_t0 >= 2.0:
                            self.cmd_vel_pub.publish(Twist())
                            return True, "centered 2s without depth; attempting grab"
                    else:
                        no_depth_centered_t0 = None
                        twist.angular.z = clamp_ang(error_px * kp)
                    self.cmd_vel_pub.publish(twist)
                    time.sleep(0.05)
                    no_detection_t0 = None
                    continue
                
                if no_detection_t0 is None:
                    no_detection_t0 = time.monotonic()
                elif time.monotonic() - no_detection_t0 >= 3.0:
                    self.cmd_vel_pub.publish(Twist())
                    return False, "visual approach failed: target not seen for 3s"

                no_depth_centered_t0 = None
                self.cmd_vel_pub.publish(Twist())
                time.sleep(0.1)
                continue

            no_detection_t0 = None
            no_depth_centered_t0 = None  # depth valid; reset blind-approach timer
            depth = float(best["depth_m"])
            error_px = center_x - bbox_cx
            centered = abs(error_px) <= tolerance

            if depth <= target_dist:
                reached_grab_range = True
                if centered:
                    self.cmd_vel_pub.publish(Twist())
                    return True, f"in range ({depth:.2f}m) and centered ({error_px:+.0f}px); grabbing"
                # In range but off-centre: rotate to centre first, no forward motion.
                twist = Twist()
                twist.angular.z = clamp_ang(error_px * kp)
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.05)
                continue

            # Not in range yet: center + drive forward.
            twist = Twist()
            twist.linear.x = linear_speed
            twist.angular.z = clamp_ang(error_px * kp)
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)

        self.cmd_vel_pub.publish(Twist())
        return False, f"visual approach timed out after {timeout:.0f}s"

    # ------------------------------------------------------------------
    # Target localization: detections lookup, memory lookup, rotate/scan
    # ------------------------------------------------------------------

    @staticmethod
    def _ang_norm(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def detection_for_class(self, target_class: str, require_depth: bool = False):
        """Return the first live detection matching target_class, else None."""
        with self.detections_lock:
            detections = list(self.latest_detections)
        for det in detections:
            label = str(det.get("class_name") or det.get("class_id") or "")
            if label.lower() != str(target_class).lower():
                continue
            if require_depth and not det.get("depth_valid"):
                continue
            return det
        return None

    def find_memory_near(self, target_class: str, point, radius_m: float):
        """Return the closest remembered object of target_class within radius of point."""
        best = None
        best_dist = float(radius_m)
        with self.memory_lock:
            for obj in self.latest_memory.values():
                if obj.get("class_name") != target_class:
                    continue
                pos = obj.get("position")
                if not pos:
                    continue
                dist = math.hypot(pos["x"] - point[0], pos["y"] - point[1])
                if dist <= best_dist:
                    best_dist = dist
                    best = obj
        return best

    def rotate_relative(self, goal_handle, delta_deg: float, target_class: str = None,
                        angular_speed: float = None, timeout_sec: float = None):
        """Rotate in place by delta_deg (CCW positive, CW negative).

        If target_class is given, stop and return seen=True the moment that class
        appears in detections. Returns (ok, message, seen).
        """
        speed = (float(self.get_parameter("scan_angular_speed").value)
                 if angular_speed is None else angular_speed)
        timeout = (float(self.get_parameter("scan_step_timeout_sec").value)
                   if timeout_sec is None else timeout_sec)
        pose = self.get_robot_pose()
        if pose is None:
            return False, "could not read robot pose", False

        target_yaw = self._ang_norm(pose[2] + math.radians(delta_deg))
        direction = 1.0 if delta_deg >= 0 else -1.0
        deadline = time.monotonic() + timeout
        twist = Twist()

        while rclpy.ok() and time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                self.cmd_vel_pub.publish(Twist())
                return False, "mission canceled", False
            if target_class is not None and self.detection_for_class(target_class) is not None:
                self.cmd_vel_pub.publish(Twist())
                return True, "target detected during rotation", True
            cur = self.get_robot_pose()
            if cur is None:
                time.sleep(0.05)
                continue
            if abs(self._ang_norm(target_yaw - cur[2])) < math.radians(5.0):
                break
            twist.angular.z = direction * abs(speed)
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)

        self.cmd_vel_pub.publish(Twist())
        return True, "rotation complete", False

    def face_point(self, goal_handle, point, distance: float = None):
        """Rotate to face the point. Once roughly aligned (|yaw_err| <
        align_deg), also reverse to back away to `distance` so the target
        is visible in camera FOV. Done when aligned AND at/past standoff."""
        standoff = (float(self.get_parameter("face_point_distance_m").value)
                    if distance is None else float(distance))
        ang_kp = float(self.get_parameter("face_point_ang_kp").value)
        ang_max = float(self.get_parameter("face_point_ang_max").value)
        ang_floor = float(self.get_parameter("face_point_ang_floor").value)
        align_rad = math.radians(float(self.get_parameter("face_point_align_deg").value))
        rev_speed = float(self.get_parameter("face_point_reverse_speed").value)
        yaw_tol = math.radians(float(self.get_parameter("face_point_yaw_tol_deg").value))
        timeout = float(self.get_parameter("face_point_timeout_sec").value)
        bx, by = float(point[0]), float(point[1])

        # Clear nav2 path so motion_arbiter doesn't override our cmd_vel.
        self._plan_pub.publish(Path())
        self.cmd_vel_pub.publish(Twist())
        time.sleep(0.1)

        deadline = time.monotonic() + timeout
        twist = Twist()
        last_log = 0.0

        while rclpy.ok() and time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                self.cmd_vel_pub.publish(Twist())
                return False, "mission canceled"
            cur = self.get_robot_pose()
            if cur is None:
                time.sleep(0.05)
                continue

            dx, dy = bx - cur[0], by - cur[1]
            dist = math.hypot(dx, dy)
            yaw_err = self._ang_norm(math.atan2(dy, dx) - cur[2])

            aligned = abs(yaw_err) < yaw_tol
            far_enough = dist >= standoff
            if aligned and far_enough:
                self.cmd_vel_pub.publish(Twist())
                self.publish_feedback(
                    goal_handle, "face/done", 0.55,
                    f"faced at {dist:.2f}m, yaw_err={math.degrees(yaw_err):+.1f}deg",
                )
                return True, f"faced point at {dist:.2f}m"

            # Angular: P + anti-stiction floor, always on.
            ang_raw = ang_kp * yaw_err
            if abs(ang_raw) > 1e-3 and abs(ang_raw) < ang_floor:
                ang_raw = math.copysign(ang_floor, ang_raw)
            twist.angular.z = max(-ang_max, min(ang_max, ang_raw))

            # Linear: reverse only when roughly aligned AND not yet at standoff.
            if abs(yaw_err) < align_rad and not far_enough:
                twist.linear.x = -abs(rev_speed)
            else:
                twist.linear.x = 0.0

            self.cmd_vel_pub.publish(twist)

            now = time.monotonic()
            if now - last_log >= 0.5:
                last_log = now
                phase = "turn+rev" if twist.linear.x < 0 else "turn"
                self.publish_feedback(
                    goal_handle, "face/step", 0.52,
                    (f"{phase} dist={dist:.2f}m yaw_err={math.degrees(yaw_err):+.1f}deg "
                     f"lin={twist.linear.x:+.2f} ang={twist.angular.z:+.2f}"),
                )

            time.sleep(0.05)

        self.cmd_vel_pub.publish(Twist())
        return False, "face_point timed out"

    def scan_for_target(self, goal_handle, target_class: str):
        """Scan-rotate to find the target: CW 45deg, then CCW 90deg, then keep
        rotating until the target is seen or the total scan timeout elapses."""
        if self.detection_for_class(target_class) is not None:
            return True, "target already visible"

        _, _, seen = self.rotate_relative(goal_handle, -45.0, target_class=target_class)
        if seen:
            return True, "target found after 45deg CW"
        if goal_handle.is_cancel_requested:
            return False, "mission canceled"

        _, _, seen = self.rotate_relative(goal_handle, 90.0, target_class=target_class)
        if seen:
            return True, "target found after 90deg CCW"
        if goal_handle.is_cancel_requested:
            return False, "mission canceled"

        total_deadline = time.monotonic() + float(self.get_parameter("scan_total_timeout_sec").value)
        while rclpy.ok() and time.monotonic() < total_deadline:
            if goal_handle.is_cancel_requested:
                return False, "mission canceled"
            _, _, seen = self.rotate_relative(goal_handle, 90.0, target_class=target_class)
            if seen:
                return True, "target found during continued rotation"
        return False, "target not found after full scan"

    def scan_door_steps_for_target(self, goal_handle, target_class: str, door_point):
        """Stepped scan used when target_class is not yet in semantic memory.

        Rotate to face door_point, then `scan_step_extra_deg` further in the same
        rotation sense — two stops that together sweep the bear's expected region.
        At each stop hold still for `scan_step_settle_sec` so YOLO / semantic
        memory can register the target. Returns (obj, message): obj is the memory
        object the moment it appears at any stop, else None (with the reason)."""
        pose = self.get_robot_pose()
        if pose is None:
            return None, "could not read robot pose"
        settle = float(self.get_parameter("scan_step_settle_sec").value)
        extra = float(self.get_parameter("scan_step_extra_deg").value)

        desired = math.atan2(door_point[1] - pose[1], door_point[0] - pose[0])
        to_door_deg = math.degrees(self._ang_norm(desired - pose[2]))
        sense = 1.0 if to_door_deg >= 0 else -1.0
        # Stop 1: face the door ref. Stop 2: `extra` deg further, same sense.
        steps = [("door ref", to_door_deg), (f"door ref +{extra:.0f}deg", sense * extra)]

        for label, step_deg in steps:
            ok, msg, _ = self.rotate_relative(goal_handle, step_deg)
            if not ok:
                return None, msg
            # Hold still and let detections / semantic memory settle at this view.
            self.cmd_vel_pub.publish(Twist())
            self._plan_pub.publish(Path())
            obj = self.find_target_from_memory(target_class, timeout_sec=settle)
            if obj is not None:
                return obj, f"{target_class} found in memory facing {label}"
        return None, f"{target_class} not in memory after door-ref scan ({len(steps)} views)"

    def center_on_target(self, goal_handle, target_class: str):
        """Rotate in place (no forward motion) until target_class is centered within
        approach_center_tolerance_px. Requires 3 consecutive centered frames to confirm."""
        # Clear any stored path so zero-Twist → IDLE, not PATH_TRACKING.
        self._plan_pub.publish(Path())
        time.sleep(0.1)

        kp = float(self.get_parameter("visual_servo_kp").value)
        center_x = float(self.get_parameter("image_center_x").value)
        tolerance = float(self.get_parameter("approach_center_tolerance_px").value)
        deadline = time.monotonic() + 10.0
        centered_frames = 0
        lost_frames = 0

        def clamp_ang(v):
            return max(-0.3, min(0.3, v))

        while rclpy.ok() and time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                self.cmd_vel_pub.publish(Twist())
                return False, "mission canceled"

            det = self.detection_for_class(target_class)
            if det is None:
                lost_frames += 1
                if lost_frames > 40:  # ~2s of no detection → give up
                    self.cmd_vel_pub.publish(Twist())
                    return False, "target lost during centering"
                self.cmd_vel_pub.publish(Twist())
                time.sleep(0.05)
                continue
            lost_frames = 0

            bbox_cx = det.get("bbox", {}).get("center_x")
            if bbox_cx is None:
                time.sleep(0.05)
                continue

            error_px = center_x - float(bbox_cx)
            if abs(error_px) <= tolerance:
                centered_frames += 1
                self.cmd_vel_pub.publish(Twist())  # hold still while confirming
                if centered_frames >= 3:
                    return True, f"centered (error {error_px:+.0f}px)"
            else:
                centered_frames = 0
                twist = Twist()
                twist.angular.z = clamp_ang(error_px * kp)
                self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)

        self.cmd_vel_pub.publish(Twist())
        return False, "centering timed out"

    # ------------------------------------------------------------------
    # Grasp and release
    # ------------------------------------------------------------------

    def approach_and_grab(self, goal_handle, target_class: str):
        """Shared grasp state: YOLO visual approach (center + drive to grab
        distance) then call the grab_object action server. On success, reverse
        by however far the approach drove us in so we don't drag the held object
        back through whatever we approached."""
        start_pose = self.get_robot_pose()  # remember where the approach began
        ok, message = self.visual_approach(goal_handle, target_class)
        if not ok:
            return False, message
        # Clear any stored path in motion_arbiter before grab. Without this,
        # the zero-Twist stop from visual_approach immediately triggers PATH_TRACKING
        # (see arbiter_node._on_motion_cmd), causing the robot to drift left/right
        # along the old bridge_center path while the arm is trying to grasp.
        self._plan_pub.publish(Path())
        time.sleep(0.1)
        ok, message = self.call_grab_object(goal_handle, target_class)
        if not ok:
            return False, message

        # Best-effort retreat to the (already-navigable) approach start pose.
        if start_pose is not None:
            cur = self.get_robot_pose()
            if cur is not None:
                forward = math.hypot(cur[0] - start_pose[0], cur[1] - start_pose[1])
                bok, bmsg = self.back_up(goal_handle, forward)
                if not bok and goal_handle.is_cancel_requested:
                    return False, bmsg
                self.publish_feedback(goal_handle, "post_grab_backup", 0.8, bmsg)
        return True, message

    def back_up(self, goal_handle, distance: float, speed: float = None):
        """Drive straight backward by `distance` meters, tracking actual TF
        displacement so it works regardless of small heading changes during the
        approach. Returns (ok, message)."""
        distance = abs(float(distance))
        if distance < 1e-3:
            return True, "no back-up needed"
        speed = (float(self.get_parameter("post_grab_backup_speed").value)
                 if speed is None else abs(float(speed)))
        timeout = float(self.get_parameter("post_grab_backup_timeout_sec").value)

        # Clear nav2 path so motion_arbiter doesn't override our reverse cmd_vel.
        self._plan_pub.publish(Path())
        self.cmd_vel_pub.publish(Twist())
        time.sleep(0.1)

        start = self.get_robot_pose()
        if start is None:
            return False, "back-up: could not read robot pose"

        deadline = time.monotonic() + timeout
        twist = Twist()
        moved = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                self.cmd_vel_pub.publish(Twist())
                return False, "mission canceled"
            cur = self.get_robot_pose()
            if cur is not None:
                moved = math.hypot(cur[0] - start[0], cur[1] - start[1])
                if moved >= distance:
                    break
            twist.linear.x = -speed
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)

        self.cmd_vel_pub.publish(Twist())
        if moved >= distance:
            return True, f"backed up {moved:.2f}m"
        return False, f"back-up timed out after {moved:.2f}m of {distance:.2f}m"

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
            self.cmd_vel_pub.publish(Twist())  # keep robot stationary during grab
            time.sleep(0.1)

        wrapped = result_future.result()
        if wrapped is None:
            return False, "grab_object returned no result"
        r = wrapped.result
        if not r.object_grasped:
            return False, f"grab failed: {r.message}"
        return True, r.message

    def clear_costmaps(self):
        """Clear the global costmap so the held/placed object doesn't block
        Nav2 planning. Waits for the service so we know the clear landed
        before the next navigate_to_pose."""
        if not self._clear_global_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                "global_costmap clear service not ready; skipping clear"
            )
            return
        future = self._clear_global_client.call_async(ClearEntireCostmap.Request())
        deadline = time.monotonic() + 2.0
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if future.done():
            self.get_logger().info("global_costmap cleared")
        else:
            self.get_logger().warn("global_costmap clear timed out")

    def release_arm(self, goal_handle):
        self.arm.publish_named("release_object", "place_pose_deg")
        deadline = time.monotonic() + self.arm.motion_wait_sec()
        while rclpy.ok() and time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                return False, "mission canceled"
            time.sleep(0.05)
            
        # Return to home pose
        self.arm.publish_named("return_home", "home_pose_deg")
        deadline = time.monotonic() + self.arm.motion_wait_sec()
        while rclpy.ok() and time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                return False, "mission canceled"
            time.sleep(0.05)
            
        return True, "object released and arm returned home"

    def set_motion_arbiter_drift_correction(self, enabled: bool):
        """Dynamically set the enable_drift_correction parameter of motion_arbiter."""
        from rcl_interfaces.srv import SetParameters
        from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
        
        client = self.create_client(SetParameters, "/motion_arbiter/set_parameters")
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("SetParameters service for /motion_arbiter not available")
            return
            
        req = SetParameters.Request()
        val = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=bool(enabled))
        param = Parameter(name="enable_drift_correction", value=val)
        req.parameters = [param]
        
        self.get_logger().info(f"Setting /motion_arbiter enable_drift_correction to {enabled}")
        client.call_async(req)
