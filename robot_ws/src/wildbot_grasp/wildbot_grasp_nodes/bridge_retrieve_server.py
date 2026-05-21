#!/usr/bin/env python3
import json
import math
import threading
import time

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ComputePathToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from wildbot_grasp.action import BridgeRetrieve
from wildbot_grasp_nodes.motion import ArmCommander


class BridgeRetrieveServer(Node):
    def __init__(self):
        super().__init__("bridge_retrieve_server")

        self.declare_parameter("target_class", "xiong_qiao")
        self.declare_parameter("standoff_distance", 0.22)
        self.declare_parameter("visual_servo_kp", 0.002)
        self.declare_parameter("visual_servo_timeout", 12.0)
        self.declare_parameter("visual_servo_tolerance_px", 15.0)
        self.declare_parameter("image_center_x", 320.0)
        self.declare_parameter("search_memory_timeout", 8.0)
        self.declare_parameter("nav_server_timeout", 5.0)

        self.callback_group = ReentrantCallbackGroup()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.arm = ArmCommander(self)

        self.memory_lock = threading.Lock()
        self.detections_lock = threading.Lock()
        self.latest_memory = {}
        self.latest_detections = []

        self.create_subscription(
            String,
            "/semantic_memory",
            self.memory_callback,
            10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            String,
            "/detections",
            self.detections_callback,
            10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            String,
            "/detections_json",
            self.detections_callback,
            10,
            callback_group=self.callback_group,
        )
        self.cmd_vel_pub = self.create_publisher(Twist, "/motion/cmd", 10)
        self.nav_client = ActionClient(
            self, NavigateToPose, "navigate_to_pose", callback_group=self.callback_group
        )
        self.path_client = self.create_client(
            ComputePathToPose, "compute_path_to_pose", callback_group=self.callback_group
        )

        self.action_server = ActionServer(
            self,
            BridgeRetrieve,
            "bridge_retrieve",
            execute_callback=self.execute_callback,
            callback_group=self.callback_group,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.get_logger().info("Ready: /bridge_retrieve")

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
            # Some detector configurations publish vision_msgs on /detections.
            return

    def goal_callback(self, goal_request):
        target = goal_request.target_class or self.get_parameter("target_class").value
        self.get_logger().info(
            f"Bridge retrieve goal target={target} bridge=({goal_request.bridge_pose_x:.2f}, "
            f"{goal_request.bridge_pose_y:.2f}, {goal_request.bridge_pose_yaw:.2f})"
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def publish_feedback(self, goal_handle, stage: str, progress: float, detail: str):
        feedback = BridgeRetrieve.Feedback()
        feedback.stage = stage
        feedback.progress = float(progress)
        feedback.detail = detail
        goal_handle.publish_feedback(feedback)
        self.get_logger().info(f"{stage}: {detail}")

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

    def navigate_to_pose(self, goal_handle, pose: PoseStamped, stage: str, progress: float):
        timeout = float(self.get_parameter("nav_server_timeout").value)
        if not self.nav_client.wait_for_server(timeout_sec=timeout):
            return False, "NavigateToPose action server not available"

        self.publish_feedback(
            goal_handle,
            stage,
            progress,
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
        return True, "navigation completed"

    def find_target_from_memory(self, target_class: str, timeout_sec: float):
        deadline = time.monotonic() + timeout_sec
        best = None
        while rclpy.ok() and time.monotonic() < deadline:
            with self.memory_lock:
                matches = [
                    obj for obj in self.latest_memory.values()
                    if obj.get("class_name") == target_class and obj.get("position")
                ]
            if matches:
                best = max(matches, key=lambda obj: (obj.get("score", 0.0), obj.get("id", "")))
                break
            time.sleep(0.1)
        return best

    def choose_approach_pose(self, target_pos: dict):
        if not self.path_client.wait_for_service(timeout_sec=5.0):
            return None, "compute_path_to_pose service not available"
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            return None, "Could not get robot pose for path planning"

        start_pose = self.make_pose(robot_pose[0], robot_pose[1], robot_pose[2])
        standoff = float(self.get_parameter("standoff_distance").value)
        best_len = float("inf")
        best_pose = None

        for angle_deg in [0, 45, 90, 135, 180, -135, -90, -45]:
            angle_rad = math.radians(angle_deg)
            cx = float(target_pos["x"]) + standoff * math.cos(angle_rad)
            cy = float(target_pos["y"]) + standoff * math.sin(angle_rad)
            yaw = angle_rad + math.pi
            goal_pose = self.make_pose(cx, cy, yaw)

            req = ComputePathToPose.Request()
            req.start = start_pose
            req.goal = goal_pose
            future = self.path_client.call_async(req)
            while rclpy.ok() and not future.done():
                time.sleep(0.01)
            resp = future.result()
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

    def open_at_grasp_pose_deg(self):
        grasp_pose = self.arm.pose_deg("grasp_pose_deg")
        place_pose = self.arm.pose_deg("place_pose_deg")
        return [grasp_pose[0], grasp_pose[1], place_pose[2]]

    def place_holding_pose_deg(self):
        grasp_pose = self.arm.pose_deg("grasp_pose_deg")
        place_pose = self.arm.pose_deg("place_pose_deg")
        return [place_pose[0], place_pose[1], grasp_pose[2]]

    def execute_callback(self, goal_handle):
        result = BridgeRetrieve.Result()
        goal = goal_handle.request
        target_class = goal.target_class or str(self.get_parameter("target_class").value)

        home_pose_tuple = self.get_robot_pose()
        if home_pose_tuple is None:
            result.success = False
            result.message = "Could not read starting pose"
            goal_handle.abort()
            return result
        home_pose = self.make_pose(*home_pose_tuple)

        bridge_pose = self.make_pose(goal.bridge_pose_x, goal.bridge_pose_y, goal.bridge_pose_yaw)
        ok, message = self.navigate_to_pose(goal_handle, bridge_pose, "to_bridge_center", 0.15)
        if not ok:
            result.success = False
            result.message = message
            goal_handle.canceled() if goal_handle.is_cancel_requested else goal_handle.abort()
            return result

        self.publish_feedback(goal_handle, "searching_target", 0.35, f"Looking for {target_class}")
        target = self.find_target_from_memory(
            target_class, float(self.get_parameter("search_memory_timeout").value)
        )
        if target is None:
            result.success = False
            result.message = f"Target class {target_class} not found in semantic memory"
            goal_handle.abort()
            return result

        target_pos = target["position"]
        approach_pose, approach_detail = self.choose_approach_pose(target_pos)
        if approach_pose is None:
            result.success = False
            result.message = approach_detail
            goal_handle.abort()
            return result

        ok, message = self.navigate_to_pose(goal_handle, approach_pose, "approaching_target", 0.50)
        if not ok:
            result.success = False
            result.message = message
            goal_handle.canceled() if goal_handle.is_cancel_requested else goal_handle.abort()
            return result

        self.publish_feedback(goal_handle, "aligning", 0.62, "Centering target with YOLO bbox")
        aligned, align_detail = self.visual_align(goal_handle, target_class)
        self.publish_feedback(goal_handle, "aligning", 0.68, align_detail)

        if goal_handle.is_cancel_requested:
            result.success = False
            result.message = "mission canceled"
            goal_handle.canceled()
            return result

        self.publish_feedback(goal_handle, "grasping", 0.72, f"Closing gripper on {target_class}")
        self.arm.send_degrees("open_for_bridge_grasp", self.open_at_grasp_pose_deg())
        self.arm.send_named("close_bridge_gripper", "grasp_pose_deg")
        self.arm.send_degrees("carry_bridge_object", self.place_holding_pose_deg())

        ok, message = self.navigate_to_pose(goal_handle, home_pose, "returning_home", 0.88)
        if not ok:
            result.success = False
            result.message = message
            goal_handle.canceled() if goal_handle.is_cancel_requested else goal_handle.abort()
            return result

        self.publish_feedback(goal_handle, "releasing", 0.97, "Releasing object at start pose")
        self.arm.send_named("release_bridge_object", "place_pose_deg")

        result.success = True
        result.message = (
            f"Bridge mission complete: reached bridge center, acquired {target_class}, "
            "returned to starting pose and released"
        )
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = BridgeRetrieveServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
