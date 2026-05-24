from __future__ import annotations

import io
import json
import math
import os
import re
import struct
import tempfile
import threading
import time
from contextlib import suppress
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any
from urllib.parse import parse_qs, urlparse

import rclpy
import tf2_ros
from tf2_ros import TransformException
import yaml
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, TwistStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from wildbot_grasp.action import BridgeRetrieve, BridgeTraverse, SearchAndRetrieve
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image, Imu, JointState
from std_msgs.msg import Bool, Empty, Float64MultiArray, String
from std_srvs.srv import Empty as EmptySrv
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .door_mission import DoorMissionCoordinator
from .open_door_proxy import OpenDoorProxy

try:
    from PIL import Image as PILImage  # type: ignore
except ImportError:  # pragma: no cover
    PILImage = None  # type: ignore

def _make_parameter(name: str, value: Any) -> Parameter:
    """Wrap a Python value in an rcl_interfaces/msg/Parameter.

    Scalars map to BOOL/INTEGER/DOUBLE/STRING; lists map to the matching array
    type. Numeric lists (e.g. arm poses like [167.0, 80.0, 170.6]) become
    DOUBLE_ARRAY so float angles survive intact.
    """
    p = Parameter()
    p.name = name
    pv = ParameterValue()
    if isinstance(value, bool):  # must precede int — bool is an int subclass
        pv.type = ParameterType.PARAMETER_BOOL
        pv.bool_value = value
    elif isinstance(value, int):
        pv.type = ParameterType.PARAMETER_INTEGER
        pv.integer_value = int(value)
    elif isinstance(value, float):
        pv.type = ParameterType.PARAMETER_DOUBLE
        pv.double_value = float(value)
    elif isinstance(value, str):
        pv.type = ParameterType.PARAMETER_STRING
        pv.string_value = value
    elif isinstance(value, (list, tuple)):
        items = list(value)
        if items and all(isinstance(v, bool) for v in items):
            pv.type = ParameterType.PARAMETER_BOOL_ARRAY
            pv.bool_array_value = [bool(v) for v in items]
        elif items and all(isinstance(v, str) for v in items):
            pv.type = ParameterType.PARAMETER_STRING_ARRAY
            pv.string_array_value = [str(v) for v in items]
        elif items and all(isinstance(v, int) and not isinstance(v, bool) for v in items):
            pv.type = ParameterType.PARAMETER_INTEGER_ARRAY
            pv.integer_array_value = [int(v) for v in items]
        else:
            # Default numeric/empty/mixed-numeric lists to double array.
            pv.type = ParameterType.PARAMETER_DOUBLE_ARRAY
            pv.double_array_value = [float(v) for v in items]
    else:
        raise ValueError(f"unsupported parameter value type for {name}: {type(value).__name__}")
    p.value = pv
    return p

_MAP_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

# Latched so motion_arbiter / arm_safeguard receive the current E-stop state
# as soon as they subscribe, even if they start after the bridge.
_ESTOP_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

ARM_JOINT_NAMES = ["arm_1_joint", "arm_2_joint", "gripper_joint"]
JPEG_MAX_WIDTH = 800
JPEG_QUALITY = 70
# Persisted next to saved maps (/maps is the bridge's only writable volume;
# /configs is mounted read-only). Survives container restarts.
WAYPOINTS_PATH = "/maps/waypoints.yaml"
ANNOTATED_IMAGE_TOPIC = "/eto_eye/annotated_image/compressed"


class BridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("igvi_bridge")
        self._lock = threading.Lock()
        self._map: dict[str, Any] | None = None
        self._costmap: dict[str, Any] | None = None
        self._pose: dict[str, float] = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self._pose_source = "none"
        self._plan: dict[str, Any] | None = None
        self._approach_pose: dict[str, Any] | None = None

        self._image_lock = threading.Lock()
        self._image_topic: str | None = None
        self._image_sub = None
        self._image_jpeg: bytes | None = None
        self._image_meta: dict[str, Any] = {}

        self._nav_lock = threading.Lock()
        self._nav_state: str = "idle"
        self._nav_message: str = ""
        self._nav_goal_handle = None
        self._nav_goal: dict[str, float] | None = None
        self._nav_feedback: dict[str, float] = {}
        self._nav_goal_token = 0

        self._open_door = OpenDoorProxy(
            self,
            on_result=lambda state, message: self._door_mission.on_open_door_result(state, message),
        )

        self._search_lock = threading.Lock()
        self._search_state: str = "idle"
        self._search_message: str = ""
        self._search_goal_handle = None
        self._search_goal: dict[str, Any] | None = None
        self._search_feedback: dict[str, Any] = {}
        self._bridge_mission_lock = threading.Lock()
        self._bridge_mission_state: str = "idle"
        self._bridge_mission_message: str = ""
        self._bridge_mission_goal_handle = None
        self._bridge_mission_goal: dict[str, Any] | None = None
        self._bridge_mission_feedback: dict[str, Any] = {}
        self._bridge_traverse_lock = threading.Lock()
        self._bridge_traverse_state: str = "idle"
        self._bridge_traverse_message: str = ""
        self._bridge_traverse_goal_handle = None
        self._bridge_traverse_goal: dict[str, Any] | None = None
        self._bridge_traverse_feedback: dict[str, Any] = {}
        self._arena_mission_lock = threading.Lock()
        self._arena_mission_state: str = "idle"
        self._arena_mission_message: str = ""
        self._arena_mission_goal_handle = None
        self._arena_mission_goal: dict[str, Any] | None = None
        self._arena_mission_feedback: dict[str, Any] = {}


        self._waypoints_lock = threading.Lock()
        self._waypoints: dict[str, dict[str, float]] = self._load_waypoints()
        self._door_mission = DoorMissionCoordinator(
            get_waypoint=self._get_waypoint,
            send_nav_goal=self._send_door_mission_nav_goal,
            cancel_nav_goal=self.cancel_nav_goal,
            snapshot_nav=self.snapshot_nav,
            send_open_door_goal=self.send_open_door_goal,
            cancel_open_door_goal=self.cancel_open_door_goal,
            snapshot_open_door=self.snapshot_open_door,
        )

        self._arm_temp_lock = threading.Lock()
        self._arm_temperatures: list[float] = []
        self._arm_temperature_stamp_sec: float | None = None
        self._arm_state_lock = threading.Lock()
        self._arm_positions: list[float] | None = None
        self._imu_lock = threading.Lock()
        self._imu_calibration_status: dict[str, Any] = {
            "ok": False,
            "state": "unavailable",
            "message": "No /imu/calibration_state message received",
            "stationary": False,
            "converged": False,
            "online": False,
            "manual_required": False,
            "manual_active": False,
            "calibration_active": False,
            "manual_remaining_s": 0.0,
            "stationary_age_s": 0.0,
            "convergence_age_s": 0.0,
            "gyro_error_rad_s": 0.0,
            "gyro_bias": [],
            "raw": "",
        }

        # Freshness tracking for EKF input sources — drives the UI badge that
        # tells the operator at a glance whether the EKF is fusing all three
        # (wheel + IMU + lidar) or has lost one of them.
        self._fusion_lock = threading.Lock()
        self._fusion_last_seen: dict[str, float] = {}
        self.create_subscription(
            Odometry, "/base_controller/odom", self._on_wheel_freshness, 10,
        )
        # /imu/calibrated is published with sensor-data QoS (BEST_EFFORT); a
        # default RELIABLE subscriber would be QoS-incompatible and never
        # receive a single message, leaving the IMU badge permanently dark.
        self.create_subscription(
            Imu, "/imu/calibrated", self._on_imu_freshness, qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry, "/odom_lidar", self._on_lidar_freshness, 10,
        )
        # Camera health: RTAB-Map RGBD visual odometry. Fresh /odom_visual means
        # the camera + visual front-end are producing usable output.
        self.create_subscription(
            Odometry, "/odom_visual", self._on_camera_freshness, 10,
        )

        self.create_subscription(OccupancyGrid, "/map", self._on_map, _MAP_QOS)
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap", self._on_costmap, _MAP_QOS)
        self.create_subscription(Path, "/plan", self._on_plan, 10)
        self.create_subscription(PoseStamped, "/approach_pose", self._on_approach_pose, 10)
        
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self.create_timer(0.05, self._update_pose_from_tf)
        
        self.create_subscription(Float64MultiArray, "/arm_joint_temperatures", self._on_arm_temperatures, 10)
        self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_states,
            10,
        )

        # Manual override commands go to /motion/cmd (Twist) so motion_arbiter
        # owns the path → /cmd_vel pipeline. We also relay motion_arbiter's
        # /cmd_vel output onto /base_controller/cmd_vel for the wheel driver.
        self._motion_cmd_pub = self.create_publisher(Twist, "/motion/cmd", 10)
        # Drops any path motion_arbiter is currently tracking — used when a
        # door mission is canceled mid-drive, since canceling the (already
        # complete) Nav2 goal does not stop the pure-pursuit executor.
        self._motion_clear_path_pub = self.create_publisher(Empty, "/motion/clear_path", 10)
        self._wheel_cmd_pub = self.create_publisher(TwistStamped, "/base_controller/cmd_vel", 10)
        self._goal_pose_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self._initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self._arm_pub = self.create_publisher(JointTrajectory, "/arm_safeguard/target_trajectory", 10)
        self._imu_calibration_start_pub = self.create_publisher(Empty, "/imu/calibration/start", 10)

        # Latched emergency-stop state. motion_arbiter zeros the base and
        # arm_safeguard freezes the arm while this is True. The bridge owns the
        # authoritative state; UI toggles it and polls it back.
        self._estop_lock = threading.Lock()
        self._estop_engaged = False
        self._estop_pub = self.create_publisher(Bool, "/estop", _ESTOP_QOS)
        self._publish_estop(False)
        self._local_costmap_clear_client = self.create_client(
            ClearEntireCostmap, "/local_costmap/clear_entirely_local_costmap"
        )
        self._global_costmap_clear_client = self.create_client(
            ClearEntireCostmap, "/global_costmap/clear_entirely_global_costmap"
        )
        # RTAB-Map's backup service snapshots the active DB to <db_path>.back.
        # Used by save_map() to freeze the live mapping into arena_map.db.
        self._rtabmap_backup_client = self.create_client(EmptySrv, "/rtabmap/backup")
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._search_client = ActionClient(self, SearchAndRetrieve, "search_retrieve")
        self._bridge_mission_client = ActionClient(self, BridgeRetrieve, "bridge_retrieve")
        self._bridge_traverse_client = ActionClient(self, BridgeTraverse, "bridge_traverse")
        self._arena_mission_client = ActionClient(self, SearchAndRetrieve, "arena_mission")
        self._semantic_memory: dict = {}
        self.create_subscription(String, "/semantic_memory", self._on_semantic_memory, 10)
        self._semantic_memory_clear_pub = self.create_publisher(Empty, "/semantic_memory/clear", 10)
        # Relay motion_arbiter's /cmd_vel (TwistStamped) to /base_controller/cmd_vel.
        self.create_subscription(TwistStamped, "/cmd_vel", self._on_nav_cmd_vel_stamped, 10)
        # Track latest arbiter state for HTTP diagnostics.
        self._motion_state: str = "unknown"
        self.create_subscription(String, "/motion/state", self._on_motion_state, 10)
        self.create_subscription(String, "/imu/calibration_state", self._on_imu_calibration_state, 10)
        self.get_logger().info("igvi_bridge node started, HTTP on :8771")

    # ── ROS callbacks ────────────────────────────────────────────────────────

    def _on_map(self, msg: OccupancyGrid) -> None:
        with self._lock:
            self._map = {
                "width": msg.info.width,
                "height": msg.info.height,
                "resolution": float(msg.info.resolution),
                "origin_x": float(msg.info.origin.position.x),
                "origin_y": float(msg.info.origin.position.y),
                "data": list(msg.data),
            }

    def _on_costmap(self, msg: OccupancyGrid) -> None:
        with self._lock:
            self._costmap = {
                "width": msg.info.width,
                "height": msg.info.height,
                "resolution": float(msg.info.resolution),
                "origin_x": float(msg.info.origin.position.x),
                "origin_y": float(msg.info.origin.position.y),
                "data": list(msg.data),
            }

    def _on_plan(self, msg: Path) -> None:
        poses = [{"x": p.pose.position.x, "y": p.pose.position.y} for p in msg.poses]
        with self._lock:
            self._plan = {"poses": poses}

    def _on_approach_pose(self, msg: PoseStamped) -> None:
        with self._lock:
            self._approach_pose = _pose_from_q(
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.orientation,
            )

    def _update_pose_from_tf(self) -> None:
        try:
            t = self._tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            with self._lock:
                self._pose = _pose_from_q(
                    t.transform.translation.x,
                    t.transform.translation.y,
                    t.transform.rotation,
                )
                self._pose_source = "tf_map"
        except TransformException:
            try:
                t = self._tf_buffer.lookup_transform("odom", "base_link", rclpy.time.Time())
                with self._lock:
                    self._pose = _pose_from_q(
                        t.transform.translation.x,
                        t.transform.translation.y,
                        t.transform.rotation,
                    )
                    self._pose_source = "tf_odom"
            except TransformException:
                pass

    def _on_nav_cmd_vel_stamped(self, msg: TwistStamped) -> None:
        # Re-stamp before forwarding so the wheel controller's cmd_vel_timeout
        # measures against the bridge clock (DDS delivery may lag).
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = msg.header.frame_id
        out.twist = msg.twist
        self._wheel_cmd_pub.publish(out)

    def _publish_wheel_stop(self) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        self._wheel_cmd_pub.publish(msg)

    def _on_motion_state(self, msg) -> None:  # std_msgs/String
        state = str(getattr(msg, "data", ""))
        self._motion_state = state
        # Door mission waits in its "driving" phase for motion_arbiter to
        # finish executing the plan — Nav2's plan-only stack returns SUCCEEDED
        # at planning time, so this is the only signal that the robot is
        # actually at the goal.
        self._door_mission.on_motion_state(state)

    def _on_arm_temperatures(self, msg: Float64MultiArray) -> None:
        now = self.get_clock().now().nanoseconds / 1_000_000_000.0
        with self._arm_temp_lock:
            self._arm_temperatures = [float(value) for value in msg.data]
            self._arm_temperature_stamp_sec = now

    def _on_joint_states(self, msg: JointState) -> None:
        names = list(msg.name)
        positions = list(msg.position)
        if not names or len(positions) < len(names):
            return
        by_name = dict(zip(names, positions))
        if not all(name in by_name for name in ARM_JOINT_NAMES):
            return
        with self._arm_state_lock:
            self._arm_positions = [float(by_name[name]) for name in ARM_JOINT_NAMES]

    def _on_imu_calibration_state(self, msg: String) -> None:
        status = _parse_imu_calibration_status(str(msg.data))
        with self._imu_lock:
            self._imu_calibration_status = status

    def _on_image(self, msg: Image) -> None:
        jpeg = _encode_jpeg(msg)
        if jpeg is None:
            return
        with self._image_lock:
            self._image_jpeg = jpeg
            self._image_meta = {
                "width": int(msg.width),
                "height": int(msg.height),
                "encoding": str(msg.encoding),
            }

    def _on_compressed_image(self, msg: CompressedImage) -> None:
        with self._image_lock:
            self._image_jpeg = bytes(msg.data)
            self._image_meta = {
                "width": 0,
                "height": 0,
                "encoding": str(msg.format or "compressed"),
            }

    # ── Thread-safe snapshots ────────────────────────────────────────────────

    def snapshot_map(self) -> dict[str, Any] | None:
        with self._lock:
            return dict(self._map) if self._map else None

    def snapshot_costmap(self) -> dict[str, Any] | None:
        with self._lock:
            return dict(self._costmap) if self._costmap else None

    def snapshot_plan(self) -> dict[str, Any] | None:
        with self._lock:
            return dict(self._plan) if self._plan else None

    def snapshot_approach_pose(self) -> dict[str, Any] | None:
        with self._lock:
            return dict(self._approach_pose) if self._approach_pose else None

    def snapshot_pose(self) -> dict[str, float]:
        with self._lock:
            return dict(self._pose)

    def snapshot_health(self) -> dict[str, Any]:
        with self._lock:
            health = {
                "ok": True,
                "map": self._map is not None,
                "pose_source": self._pose_source,
            }
        # Sensor freshness drives the grouped sensor badge in the UI top bar,
        # which polls /api/health via the host's ui-bridge health check.
        health["fusion_sources"] = self.snapshot_fusion_sources()
        return health

    def publish_cmd_vel(self, linear_x: float, angular_z: float) -> None:
        # Manual override flows through motion_arbiter: publish a Twist on
        # /motion/cmd; arbiter will preempt path tracking and publish the
        # actual /cmd_vel which the bridge relays to /base_controller/cmd_vel.
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self._motion_cmd_pub.publish(msg)

    def emergency_stop(self) -> tuple[bool, str]:
        canceled: list[str] = []

        with self._bridge_mission_lock:
            bridge_handle = self._bridge_mission_goal_handle
        if bridge_handle is not None:
            self._update_bridge_mission_state("canceling", "emergency stop requested")
            bridge_future = bridge_handle.cancel_goal_async()
            bridge_future.add_done_callback(self._on_bridge_mission_cancel_response)
            canceled.append("bridge mission")

        with self._bridge_traverse_lock:
            traverse_handle = self._bridge_traverse_goal_handle
        if traverse_handle is not None:
            self._update_bridge_traverse_state("canceling", "emergency stop requested")
            traverse_future = traverse_handle.cancel_goal_async()
            traverse_future.add_done_callback(self._on_bridge_traverse_cancel_response)
            canceled.append("bridge traverse")

        with self._search_lock:
            search_handle = self._search_goal_handle
        if search_handle is not None:
            self._update_search_state("canceling", "emergency stop requested")
            search_future = search_handle.cancel_goal_async()
            search_future.add_done_callback(self._on_search_cancel_response)
            canceled.append("search mission")

        with self._nav_lock:
            nav_handle = self._nav_goal_handle
        if nav_handle is not None:
            self._update_nav_state("canceling", "emergency stop requested")
            nav_future = nav_handle.cancel_goal_async()
            nav_future.add_done_callback(self._on_nav_cancel_response)
            canceled.append("navigation")

        for _ in range(3):
            self.publish_cmd_vel(0.0, 0.0)
            self._publish_wheel_stop()

        with self._arm_state_lock:
            arm_positions = list(self._arm_positions) if self._arm_positions else None
        if arm_positions:
            self.publish_arm_trajectory(arm_positions, 0.1)
            canceled.append("arm hold")

        if canceled:
            return True, "emergency stop sent; cancel requested for " + ", ".join(canceled)
        return True, "emergency stop sent"

    def _publish_estop(self, engaged: bool) -> None:
        msg = Bool()
        msg.data = bool(engaged)
        self._estop_pub.publish(msg)

    def _apply_estop(self, engaged: bool) -> str:
        self._publish_estop(engaged)
        if engaged:
            # Latched /estop keeps the base zeroed and arm frozen; also cancel
            # any running missions so they don't resume fighting on release.
            self.emergency_stop()
            return "emergency stop engaged"
        return "emergency stop released"

    def set_estop(self, engaged: bool) -> tuple[bool, bool, str]:
        engaged = bool(engaged)
        with self._estop_lock:
            self._estop_engaged = engaged
        msg = self._apply_estop(engaged)
        return True, engaged, msg

    def snapshot_estop(self) -> dict[str, Any]:
        with self._estop_lock:
            return {"ok": True, "engaged": self._estop_engaged}

    def snapshot_motion_state(self) -> str:
        return self._motion_state

    def snapshot_arm_temperatures(self) -> dict[str, Any]:
        with self._arm_temp_lock:
            temperatures = list(self._arm_temperatures)
            stamp_sec = self._arm_temperature_stamp_sec
        gripper_index = 2
        gripper_temperature = temperatures[gripper_index] if gripper_index < len(temperatures) else None
        return {
            "ok": bool(temperatures),
            "temperatures": temperatures,
            "gripper_index": gripper_index,
            "gripper_temperature": gripper_temperature,
            "stamp_sec": stamp_sec,
        }

    def snapshot_imu_calibration(self) -> dict[str, Any]:
        with self._imu_lock:
            return dict(self._imu_calibration_status)

    def start_imu_calibration(self) -> None:
        self._imu_calibration_start_pub.publish(Empty())

    def publish_goal_pose(self, x: float, y: float, yaw: float, frame_id: str = "map") -> None:
        msg = PoseStamped()
        msg.header.frame_id = frame_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        half = yaw / 2.0
        msg.pose.orientation.z = math.sin(half)
        msg.pose.orientation.w = math.cos(half)
        self._goal_pose_pub.publish(msg)

    def publish_initial_pose(self, x: float, y: float, yaw: float, frame_id: str = "map") -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = frame_id
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        half = yaw / 2.0
        msg.pose.pose.orientation.z = math.sin(half)
        msg.pose.pose.orientation.w = math.cos(half)
        self._initial_pose_pub.publish(msg)

    def publish_arm_trajectory(self, positions: list[float], time_from_start: float) -> None:
        if len(positions) != len(ARM_JOINT_NAMES):
            raise ValueError(f"expected {len(ARM_JOINT_NAMES)} positions, got {len(positions)}")
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = list(ARM_JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        sec = int(time_from_start)
        nanosec = int((time_from_start - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=max(0, nanosec))
        msg.points = [point]
        self._arm_pub.publish(msg)

    # ── Image topic management ───────────────────────────────────────────────

    def list_image_topics(self) -> list[str]:
        topics = self.get_topic_names_and_types()
        out: list[str] = []
        for name, types in topics:
            try:
                if "sensor_msgs/msg/Image" in types:
                    if self.count_publishers(name) > 0:
                        out.append(name)
                elif (
                    name == ANNOTATED_IMAGE_TOPIC
                    and "sensor_msgs/msg/CompressedImage" in types
                ):
                    if self.count_publishers(name) > 0:
                        out.append(name)
            except Exception:
                out.append(name)
        return sorted(out)

    def set_image_topic(self, topic: str | None) -> None:
        with self._image_lock:
            if topic == self._image_topic:
                return
            if self._image_sub is not None:
                self.destroy_subscription(self._image_sub)
                self._image_sub = None
            self._image_topic = topic
            self._image_jpeg = None
            self._image_meta = {}
            if topic:
                topic_types = dict(self.get_topic_names_and_types()).get(topic, [])
                if "sensor_msgs/msg/CompressedImage" in topic_types:
                    self._image_sub = self.create_subscription(
                        CompressedImage, topic, self._on_compressed_image, qos_profile_sensor_data
                    )
                else:
                    self._image_sub = self.create_subscription(
                        Image, topic, self._on_image, qos_profile_sensor_data
                    )

    def snapshot_image(self) -> tuple[bytes | None, str | None, dict[str, Any]]:
        with self._image_lock:
            return self._image_jpeg, self._image_topic, dict(self._image_meta)

    # ── Map saving ────────────────────────────────────────────────────────────

    def save_map(self, filename: str = "arena_map", out_dir: str = "/slam") -> tuple[bool, str]:
        # Snapshot the active RTAB-Map DB into <db>.back, then copy it to
        # <out_dir>/<filename>.db so localization can load it next boot.
        # Requires slam_fusion's database_path to live on a host volume that's
        # also mounted into this container at out_dir.
        if not self._rtabmap_backup_client.wait_for_service(timeout_sec=2.0):
            return False, "/rtabmap/backup service not available — is slam_fusion running?"

        done = threading.Event()
        result: dict[str, Any] = {"ok": False, "message": "/rtabmap/backup timed out"}
        future = self._rtabmap_backup_client.call_async(EmptySrv.Request())

        def _finished(_future: Any) -> None:
            try:
                _future.result()
                result["ok"] = True
            except Exception as exc:  # noqa: BLE001
                result["message"] = f"/rtabmap/backup failed: {exc}"
            finally:
                done.set()

        future.add_done_callback(_finished)
        if not done.wait(timeout=15.0) or not result["ok"]:
            return False, str(result["message"])

        backup_src = os.path.join(out_dir, "rtabmap.db.back")
        dest = os.path.join(out_dir, f"{filename}.db")
        if not os.path.exists(backup_src):
            return False, f"backup ran but {backup_src} not found — check slam_fusion's database_path bind mount"

        try:
            os.makedirs(out_dir, exist_ok=True)
            import shutil
            shutil.copyfile(backup_src, dest)
        except OSError as exc:
            return False, f"copy to {dest} failed: {exc}"

        self.get_logger().info(f"Arena map saved to {dest}")
        return True, dest

    def clear_costmap(self, target: str = "local") -> tuple[bool, str]:
        if target == "global":
            client = self._global_costmap_clear_client
            name = "/global_costmap/clear_entirely_global_costmap"
        else:
            client = self._local_costmap_clear_client
            name = "/local_costmap/clear_entirely_local_costmap"

        if not client.wait_for_service(timeout_sec=1.0):
            return False, f"{name} service not available"

        done = threading.Event()
        result: dict[str, Any] = {"ok": False, "message": f"{name} timed out"}
        future = client.call_async(ClearEntireCostmap.Request())

        def _finished(_future: Any) -> None:
            try:
                _future.result()
                result["ok"] = True
                result["message"] = f"{target} costmap cleared"
            except Exception as exc:  # noqa: BLE001
                result["ok"] = False
                result["message"] = str(exc)
            finally:
                done.set()

        future.add_done_callback(_finished)
        done.wait(timeout=3.0)
        return bool(result["ok"]), str(result["message"])

    # ── Remote ROS parameter setting (live tuning) ────────────────────────────

    def set_remote_parameters(
        self, node_name: str, params: dict[str, Any]
    ) -> tuple[bool, str]:
        """Set parameters on another node via its /<node>/set_parameters service.

        Used by the UI to live-tune detectors and controllers (e.g. open_door's
        HSV thresholds) without redeploying. Values may be bool/int/float/str;
        the type is inferred per call.
        """
        if not node_name:
            return False, "node name required"
        service_name = f"/{node_name.strip('/')}/set_parameters"
        client = self.create_client(SetParameters, service_name)
        try:
            if not client.wait_for_service(timeout_sec=1.0):
                return False, f"service {service_name} not available"

            request = SetParameters.Request()
            for name, value in params.items():
                try:
                    request.parameters.append(_make_parameter(name, value))
                except ValueError as exc:
                    return False, str(exc)

            done = threading.Event()
            outcome: dict[str, Any] = {"ok": False, "message": "set_parameters timed out"}
            future = client.call_async(request)

            def _finished(_future: Any) -> None:
                try:
                    response = _future.result()
                    failures = [
                        f"{p.name}: {r.reason or 'rejected'}"
                        for p, r in zip(request.parameters, response.results)
                        if not r.successful
                    ]
                    if failures:
                        outcome["message"] = "; ".join(failures)
                    else:
                        outcome["ok"] = True
                        outcome["message"] = (
                            f"set {len(request.parameters)} parameter(s) on {node_name}"
                        )
                except Exception as exc:  # noqa: BLE001
                    outcome["message"] = f"set_parameters failed: {exc}"
                finally:
                    done.set()

            future.add_done_callback(_finished)
            done.wait(timeout=3.0)
            return bool(outcome["ok"]), str(outcome["message"])
        finally:
            # Don't leak service clients across many tuning calls.
            self.destroy_client(client)

    # ── Open-door action (red-bar FSM trigger) ────────────────────────────────

    def snapshot_open_door(self) -> dict[str, Any]:
        return self._open_door.snapshot()

    def send_open_door_goal(self, ready_distance_m: float = 0.0) -> tuple[bool, str]:
        return self._open_door.send_goal(ready_distance_m)

    def cancel_open_door_goal(self) -> tuple[bool, str]:
        return self._open_door.cancel_goal()

    # ── Door mission API proxy ───────────────────────────────────────────────

    def start_door_mission(
        self, waypoint: str = "door_approach", ready_distance_m: float = 0.0
    ) -> tuple[bool, str]:
        return self._door_mission.start(waypoint, ready_distance_m)

    def cancel_door_mission(self) -> tuple[bool, str]:
        # If the mission is mid-drive, the Nav2 goal is already complete and
        # canceling it is a no-op; the actual stop signal motion_arbiter
        # respects is /motion/clear_path.
        phase = self._door_mission.snapshot().get("phase", "")
        if phase == "driving":
            self._motion_clear_path_pub.publish(Empty())
        return self._door_mission.cancel()

    def snapshot_door_mission(self) -> dict[str, Any]:
        return self._door_mission.snapshot()

    def save_open_door_poses(self) -> tuple[bool, str]:
        return self._open_door.save_poses()

    def call_open_door_step(self, step: str) -> tuple[bool, str]:
        return self._open_door.call_step(step)

    # ── Waypoints (named map-frame poses, persisted to /maps) ─────────────────

    def _load_waypoints(self) -> dict[str, dict[str, float]]:
        try:
            with open(WAYPOINTS_PATH) as f:
                data = yaml.safe_load(f) or {}
        except (FileNotFoundError, OSError, yaml.YAMLError):
            return {}
        raw = data.get("waypoints") if isinstance(data, dict) else None
        out: dict[str, dict[str, float]] = {}
        if isinstance(raw, dict):
            for name, pose in raw.items():
                if not isinstance(pose, dict):
                    continue
                out[str(name)] = {
                    "x": float(pose.get("x", 0.0)),
                    "y": float(pose.get("y", 0.0)),
                    "yaw": float(pose.get("yaw", 0.0)),
                }
        return out

    def _write_waypoints(self) -> None:
        """Atomic write; caller must hold _waypoints_lock."""
        os.makedirs(os.path.dirname(WAYPOINTS_PATH), exist_ok=True)
        payload = yaml.dump({"waypoints": self._waypoints}, default_flow_style=False)
        tmp_fd, tmp_path = tempfile.mkstemp(
            dir=os.path.dirname(WAYPOINTS_PATH), suffix=".yaml"
        )
        try:
            os.write(tmp_fd, payload.encode())
            os.close(tmp_fd)
            os.replace(tmp_path, WAYPOINTS_PATH)
        except Exception:
            with suppress(OSError):
                os.close(tmp_fd)
            with suppress(OSError):
                os.unlink(tmp_path)
            raise

    def list_waypoints(self) -> dict[str, dict[str, float]]:
        with self._waypoints_lock:
            return {k: dict(v) for k, v in self._waypoints.items()}

    def _get_waypoint(self, name: str) -> dict[str, float] | None:
        with self._waypoints_lock:
            wp = self._waypoints.get(str(name).strip())
            return dict(wp) if wp else None

    def save_waypoint(
        self,
        name: str,
        x: float | None = None,
        y: float | None = None,
        yaw: float | None = None,
    ) -> tuple[bool, str]:
        name = str(name).strip()
        if not name:
            return False, "waypoint name is required"
        if x is None or y is None:
            # "Mark where I am": snapshot the current map-frame pose.
            pose = self.snapshot_pose()
            wp = {
                "x": float(pose["x"]),
                "y": float(pose["y"]),
                "yaw": float(pose["yaw"]),
            }
        else:
            wp = {"x": float(x), "y": float(y), "yaw": float(yaw or 0.0)}
        with self._waypoints_lock:
            self._waypoints[name] = wp
            try:
                self._write_waypoints()
            except OSError as exc:
                del self._waypoints[name]
                return False, f"failed to persist waypoint: {exc}"
        return True, f"saved waypoint '{name}'"

    def delete_waypoint(self, name: str) -> tuple[bool, str]:
        name = str(name).strip()
        with self._waypoints_lock:
            if name not in self._waypoints:
                return False, f"no waypoint named '{name}'"
            removed = self._waypoints.pop(name)
            try:
                self._write_waypoints()
            except OSError as exc:
                self._waypoints[name] = removed
                return False, f"failed to persist deletion: {exc}"
        return True, f"deleted waypoint '{name}'"

    def goto_waypoint(self, name: str) -> tuple[bool, str]:
        name = str(name).strip()
        with self._waypoints_lock:
            wp = self._waypoints.get(name)
            wp = dict(wp) if wp else None
        if wp is None:
            return False, f"no waypoint named '{name}'"
        return self.send_nav_goal(wp["x"], wp["y"], wp["yaw"])

    # ── Navigation (Nav2 NavigateToPose action) ──────────────────────────────

    def send_nav_goal(self, x: float, y: float, yaw: float) -> tuple[bool, str]:
        ok, msg, _token = self._dispatch_nav_goal(x, y, yaw)
        return ok, msg

    def _send_door_mission_nav_goal(
        self, x: float, y: float, yaw: float
    ) -> tuple[bool, str, int | None]:
        return self._dispatch_nav_goal(x, y, yaw)

    def _dispatch_nav_goal(
        self, x: float, y: float, yaw: float
    ) -> tuple[bool, str, int | None]:
        if not self._nav_client.server_is_ready():
            if not self._nav_client.wait_for_server(timeout_sec=3.5):
                hint = self._nav_diagnostic_hint()
                self._update_nav_state("unavailable", hint)
                return False, hint, None
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        half = float(yaw) / 2.0
        goal_msg.pose.pose.orientation.z = math.sin(half)
        goal_msg.pose.pose.orientation.w = math.cos(half)
        with self._nav_lock:
            self._nav_goal_token += 1
            goal_token = self._nav_goal_token
            self._nav_goal = {"x": float(x), "y": float(y), "yaw": float(yaw)}
            self._nav_feedback = {}
        self._update_nav_state("sending", "goal dispatched")
        future = self._nav_client.send_goal_async(goal_msg, feedback_callback=self._on_nav_feedback)
        future.add_done_callback(lambda done, token=goal_token: self._on_nav_response(done, token))
        return True, "goal dispatched", goal_token

    def cancel_nav_goal(self) -> tuple[bool, str]:
        with self._nav_lock:
            handle = self._nav_goal_handle
        if handle is None:
            return False, "no active goal"
        self._update_nav_state("canceling", "cancel requested")
        future = handle.cancel_goal_async()
        future.add_done_callback(self._on_nav_cancel_response)
        return True, "cancel requested"

    def snapshot_nav(self) -> dict[str, Any]:
        with self._nav_lock:
            payload = {
                "state": self._nav_state,
                "message": self._nav_message,
                "goal": dict(self._nav_goal) if self._nav_goal else None,
                "feedback": dict(self._nav_feedback),
                "server_ready": self._nav_client.server_is_ready(),
                "visible_actions": self._visible_action_names(),
            }
        payload["fusion_sources"] = self.snapshot_fusion_sources()
        return payload

    # ── Search and Retrieve ──────────────────────────────────────────────────

    def _update_search_state(self, state: str, message: str) -> None:
        with self._search_lock:
            self._search_state = state
            self._search_message = message
        self.get_logger().info(f"Search state: {state} — {message}")

    def send_search_goal(self, target_id: str, x: float, y: float, yaw: float) -> tuple[bool, str]:
        with self._search_lock:
            if self._search_state not in ("idle", "unavailable"):
                return False, f"Cannot start: already in state '{self._search_state}'"
                
        if not self._search_client.server_is_ready():
            if not self._search_client.wait_for_server(timeout_sec=2.0):
                self._update_search_state("unavailable", "Search server offline")
                return False, "Search server offline"
        goal_msg = SearchAndRetrieve.Goal()
        goal_msg.target_id = str(target_id)
        goal_msg.home_pose_x = float(x)
        goal_msg.home_pose_y = float(y)
        goal_msg.home_pose_yaw = float(yaw)
        with self._search_lock:
            self._search_goal = {"target_id": target_id, "home_pose_x": x, "home_pose_y": y, "home_pose_yaw": yaw}
            self._search_feedback = {}
        self._update_search_state("sending", "goal dispatched")
        future = self._search_client.send_goal_async(goal_msg, feedback_callback=self._on_search_feedback)
        future.add_done_callback(self._on_search_response)
        return True, "goal dispatched"

    def cancel_search_goal(self) -> tuple[bool, str]:
        with self._search_lock:
            handle = self._search_goal_handle
        if handle is None:
            return False, "no active goal"
        self._update_search_state("canceling", "cancel requested")
        future = handle.cancel_goal_async()
        future.add_done_callback(self._on_search_cancel_response)
        return True, "cancel requested"

    def snapshot_search(self) -> dict[str, Any]:
        with self._search_lock:
            return {
                "state": self._search_state,
                "message": self._search_message,
                "goal": dict(self._search_goal) if self._search_goal else None,
                "feedback": dict(self._search_feedback),
                "server_ready": self._search_client.server_is_ready(),
            }

    def _on_search_feedback(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        with self._search_lock:
            self._search_feedback = {
                "stage": fb.stage,
                "progress": float(fb.progress),
                "detail": fb.detail,
            }

    def _on_search_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._update_search_state("idle", f"Failed to send goal: {exc}")
            return
        if not goal_handle.accepted:
            self._update_search_state("idle", "goal rejected")
            return
        with self._search_lock:
            self._search_goal_handle = goal_handle
        self._update_search_state("active", "goal accepted")
        res_future = goal_handle.get_result_async()
        res_future.add_done_callback(self._on_search_result)

    def _on_search_result(self, future) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self._update_search_state("idle", f"Action server crashed or failed: {exc}")
            with self._search_lock:
                self._search_goal_handle = None
            return
            
        status = result.status
        with self._search_lock:
            self._search_goal_handle = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            msg = result.result.message if hasattr(result.result, "message") else "succeeded"
            self._update_search_state("idle", f"success: {msg}")
        elif status == GoalStatus.STATUS_CANCELED:
            self._update_search_state("idle", "canceled")
        elif status == GoalStatus.STATUS_ABORTED:
            msg = result.result.message if hasattr(result.result, "message") else "aborted"
            self._update_search_state("idle", f"aborted: {msg}")
        else:
            self._update_search_state("idle", f"completed with status {status}")

    def _on_search_cancel_response(self, future) -> None:
        response = future.result()
        if len(response.goals_canceling) > 0:
            self._update_search_state("idle", "cancel accepted")
        else:
            self._update_search_state("active", "cancel rejected")
    # ── Bridge Retrieve Mission ──────────────────────────────────────────────

    def _update_bridge_mission_state(self, state: str, message: str) -> None:
        with self._bridge_mission_lock:
            self._bridge_mission_state = state
            self._bridge_mission_message = message
        self.get_logger().info(f"Bridge mission state: {state} — {message}")

    def send_bridge_mission_goal(
        self,
        target_class: str,
        bridge_x: float,
        bridge_y: float,
        bridge_yaw: float,
    ) -> tuple[bool, str]:
        with self._bridge_mission_lock:
            if self._bridge_mission_state not in ("idle", "unavailable"):
                return False, f"Cannot start: already in state '{self._bridge_mission_state}'"
        with self._bridge_traverse_lock:
            if self._bridge_traverse_state not in ("idle", "unavailable"):
                return False, f"Cannot start: bridge traverse is in state '{self._bridge_traverse_state}'"
                
        if not self._bridge_mission_client.server_is_ready():
            if not self._bridge_mission_client.wait_for_server(timeout_sec=2.0):
                self._update_bridge_mission_state("unavailable", "Bridge retrieve server offline")
                return False, "Bridge retrieve server offline"
        with self._waypoints_lock:
            home_wp = self._waypoints.get("home")
            home_wp = dict(home_wp) if home_wp else None
            door_wp = self._waypoints.get("door_ref")
            door_wp = dict(door_wp) if door_wp else None
            # Ordered return via-points: waypoints named return_<N>, sorted by N.
            return_wps = [
                dict(self._waypoints[name])
                for name in sorted(
                    (n for n in self._waypoints if re.fullmatch(r"return_\d+", n)),
                    key=lambda n: int(n.split("_")[1]),
                )
            ]
        if home_wp is None:
            self._update_bridge_mission_state("error", "No home waypoint set — press Home first")
            return False, "No home waypoint set — press Home first"
        if door_wp is None:
            self._update_bridge_mission_state(
                "error", "No door ref point set — press Set Door Ref Point first")
            return False, "No door ref point set — press Set Door Ref Point first"

        goal_msg = BridgeRetrieve.Goal()
        goal_msg.target_class = str(target_class)
        goal_msg.bridge_pose_x = float(bridge_x)
        goal_msg.bridge_pose_y = float(bridge_y)
        goal_msg.bridge_pose_yaw = float(bridge_yaw)
        goal_msg.home_pose_valid = True
        goal_msg.home_pose_x = float(home_wp["x"])
        goal_msg.home_pose_y = float(home_wp["y"])
        goal_msg.home_pose_yaw = float(home_wp.get("yaw", 0.0))
        # door_ref is optional: only its direction biases the scan rotation sense.
        goal_msg.door_ref_valid = door_wp is not None
        goal_msg.door_ref_x = float(door_wp["x"]) if door_wp else 0.0
        goal_msg.door_ref_y = float(door_wp["y"]) if door_wp else 0.0
        # Ordered return via-points (empty → return straight to home).
        goal_msg.return_path_x = [float(wp["x"]) for wp in return_wps]
        goal_msg.return_path_y = [float(wp["y"]) for wp in return_wps]
        goal_msg.return_path_yaw = [float(wp.get("yaw", 0.0)) for wp in return_wps]
        with self._bridge_mission_lock:
            self._bridge_mission_goal = {
                "target_class": target_class,
                "bridge_pose_x": bridge_x,
                "bridge_pose_y": bridge_y,
                "bridge_pose_yaw": bridge_yaw,
                "home_pose": {"x": home_wp["x"], "y": home_wp["y"], "yaw": home_wp.get("yaw", 0.0)},
                "door_ref": {"x": door_wp["x"], "y": door_wp["y"]} if door_wp else None,
                "return_path": [{"x": wp["x"], "y": wp["y"]} for wp in return_wps],
            }
            self._bridge_mission_feedback = {}
        self._update_bridge_mission_state("sending", "goal dispatched")
        future = self._bridge_mission_client.send_goal_async(
            goal_msg,
            feedback_callback=self._on_bridge_mission_feedback,
        )
        future.add_done_callback(self._on_bridge_mission_response)
        return True, "goal dispatched"

    def send_bridge_mission_waypoint_goal(
        self,
        target_class: str,
        waypoint_name: str,
    ) -> tuple[bool, str]:
        name = str(waypoint_name).strip()
        with self._waypoints_lock:
            wp = self._waypoints.get(name)
            wp = dict(wp) if wp else None
        if wp is None:
            return False, f"no waypoint named '{name}'"
        return self.send_bridge_mission_goal(target_class, wp["x"], wp["y"], wp["yaw"])

    def cancel_bridge_mission_goal(self) -> tuple[bool, str]:
        with self._bridge_mission_lock:
            handle = self._bridge_mission_goal_handle
        if handle is None:
            return False, "no active goal"
        self._update_bridge_mission_state("canceling", "cancel requested")
        future = handle.cancel_goal_async()
        future.add_done_callback(self._on_bridge_mission_cancel_response)
        return True, "cancel requested"

    def snapshot_bridge_mission(self) -> dict[str, Any]:
        with self._bridge_mission_lock:
            return {
                "state": self._bridge_mission_state,
                "message": self._bridge_mission_message,
                "goal": dict(self._bridge_mission_goal) if self._bridge_mission_goal else None,
                "feedback": dict(self._bridge_mission_feedback),
                "server_ready": self._bridge_mission_client.server_is_ready(),
            }

    def _on_bridge_mission_feedback(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        with self._bridge_mission_lock:
            self._bridge_mission_feedback = {
                "stage": fb.stage,
                "progress": float(fb.progress),
                "detail": fb.detail,
            }

    def _on_bridge_mission_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._update_bridge_mission_state("idle", "goal rejected")
            return
        with self._bridge_mission_lock:
            self._bridge_mission_goal_handle = goal_handle
        self._update_bridge_mission_state("active", "goal accepted")
        res_future = goal_handle.get_result_async()
        res_future.add_done_callback(self._on_bridge_mission_result)

    def _on_bridge_mission_result(self, future) -> None:
        result = future.result()
        status = result.status
        with self._bridge_mission_lock:
            self._bridge_mission_goal_handle = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            msg = result.result.message if hasattr(result.result, "message") else "succeeded"
            self._update_bridge_mission_state("idle", f"success: {msg}")
        elif status == GoalStatus.STATUS_CANCELED:
            self._update_bridge_mission_state("idle", "canceled")
        elif status == GoalStatus.STATUS_ABORTED:
            msg = result.result.message if hasattr(result.result, "message") else "aborted"
            self._update_bridge_mission_state("idle", f"aborted: {msg}")
        else:
            self._update_bridge_mission_state("idle", f"completed with status {status}")

    def _on_bridge_mission_cancel_response(self, future) -> None:
        response = future.result()
        if len(response.goals_canceling) > 0:
            self._update_bridge_mission_state("idle", "cancel accepted")
        else:
            self._update_bridge_mission_state("active", "cancel rejected")

    # ── Bridge Traverse Mission ─────────────────────────────────────────────

    def _update_bridge_traverse_state(self, state: str, message: str) -> None:
        with self._bridge_traverse_lock:
            self._bridge_traverse_state = state
            self._bridge_traverse_message = message
        self.get_logger().info(f"Bridge traverse state: {state} — {message}")

    def send_bridge_traverse_goal(
        self,
        bridge_x: float,
        bridge_y: float,
        bridge_yaw: float,
    ) -> tuple[bool, str]:
        with self._bridge_traverse_lock:
            if self._bridge_traverse_state not in ("idle", "unavailable"):
                return False, f"Cannot start: already in state '{self._bridge_traverse_state}'"
        with self._bridge_mission_lock:
            if self._bridge_mission_state not in ("idle", "unavailable"):
                return False, f"Cannot start: bridge mission is in state '{self._bridge_mission_state}'"

        if not self._bridge_traverse_client.server_is_ready():
            if not self._bridge_traverse_client.wait_for_server(timeout_sec=2.0):
                self._update_bridge_traverse_state("unavailable", "Bridge traverse server offline")
                return False, "Bridge traverse server offline"

        with self._waypoints_lock:
            home_wp = self._waypoints.get("home")
            home_wp = dict(home_wp) if home_wp else None
            return_wps = [
                dict(self._waypoints[name])
                for name in sorted(
                    (n for n in self._waypoints if re.fullmatch(r"return_\d+", n)),
                    key=lambda n: int(n.split("_")[1]),
                )
            ]
        if home_wp is None:
            self._update_bridge_traverse_state("error", "No home waypoint set — press Home first")
            return False, "No home waypoint set — press Home first"

        goal_msg = BridgeTraverse.Goal()
        goal_msg.bridge_pose_x = float(bridge_x)
        goal_msg.bridge_pose_y = float(bridge_y)
        goal_msg.bridge_pose_yaw = float(bridge_yaw)
        goal_msg.home_pose_valid = True
        goal_msg.home_pose_x = float(home_wp["x"])
        goal_msg.home_pose_y = float(home_wp["y"])
        goal_msg.home_pose_yaw = float(home_wp.get("yaw", 0.0))
        goal_msg.return_path_x = [float(wp["x"]) for wp in return_wps]
        goal_msg.return_path_y = [float(wp["y"]) for wp in return_wps]
        goal_msg.return_path_yaw = [float(wp.get("yaw", 0.0)) for wp in return_wps]

        with self._bridge_traverse_lock:
            self._bridge_traverse_goal = {
                "bridge_pose_x": bridge_x,
                "bridge_pose_y": bridge_y,
                "bridge_pose_yaw": bridge_yaw,
                "home_pose": {"x": home_wp["x"], "y": home_wp["y"], "yaw": home_wp.get("yaw", 0.0)},
                "return_path": [{"x": wp["x"], "y": wp["y"]} for wp in return_wps],
            }
            self._bridge_traverse_feedback = {}
        self._update_bridge_traverse_state("sending", "goal dispatched")
        future = self._bridge_traverse_client.send_goal_async(
            goal_msg,
            feedback_callback=self._on_bridge_traverse_feedback,
        )
        future.add_done_callback(self._on_bridge_traverse_response)
        return True, "goal dispatched"

    def send_bridge_traverse_waypoint_goal(
        self,
        waypoint_name: str,
    ) -> tuple[bool, str]:
        name = str(waypoint_name).strip()
        with self._waypoints_lock:
            wp = self._waypoints.get(name)
            wp = dict(wp) if wp else None
        if wp is None:
            return False, f"no waypoint named '{name}'"
        return self.send_bridge_traverse_goal(wp["x"], wp["y"], wp["yaw"])

    def cancel_bridge_traverse_goal(self) -> tuple[bool, str]:
        with self._bridge_traverse_lock:
            handle = self._bridge_traverse_goal_handle
        if handle is None:
            return False, "no active goal"
        self._update_bridge_traverse_state("canceling", "cancel requested")
        future = handle.cancel_goal_async()
        future.add_done_callback(self._on_bridge_traverse_cancel_response)
        return True, "cancel requested"

    def snapshot_bridge_traverse(self) -> dict[str, Any]:
        with self._bridge_traverse_lock:
            return {
                "state": self._bridge_traverse_state,
                "message": self._bridge_traverse_message,
                "goal": dict(self._bridge_traverse_goal) if self._bridge_traverse_goal else None,
                "feedback": dict(self._bridge_traverse_feedback),
                "server_ready": self._bridge_traverse_client.server_is_ready(),
            }

    def _on_bridge_traverse_feedback(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        with self._bridge_traverse_lock:
            self._bridge_traverse_feedback = {
                "stage": fb.stage,
                "progress": float(fb.progress),
                "detail": fb.detail,
            }

    def _on_bridge_traverse_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._update_bridge_traverse_state("idle", "goal rejected")
            return
        with self._bridge_traverse_lock:
            self._bridge_traverse_goal_handle = goal_handle
        self._update_bridge_traverse_state("active", "goal accepted")
        res_future = goal_handle.get_result_async()
        res_future.add_done_callback(self._on_bridge_traverse_result)

    def _on_bridge_traverse_result(self, future) -> None:
        result = future.result()
        status = result.status
        with self._bridge_traverse_lock:
            self._bridge_traverse_goal_handle = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            msg = result.result.message if hasattr(result.result, "message") else "succeeded"
            self._update_bridge_traverse_state("idle", f"success: {msg}")
        elif status == GoalStatus.STATUS_CANCELED:
            self._update_bridge_traverse_state("idle", "canceled")
        elif status == GoalStatus.STATUS_ABORTED:
            msg = result.result.message if hasattr(result.result, "message") else "aborted"
            self._update_bridge_traverse_state("idle", f"aborted: {msg}")
        else:
            self._update_bridge_traverse_state("idle", f"completed with status {status}")

    def _on_bridge_traverse_cancel_response(self, future) -> None:
        response = future.result()
        if len(response.goals_canceling) > 0:
            self._update_bridge_traverse_state("idle", "cancel accepted")
        else:
            self._update_bridge_traverse_state("active", "cancel rejected")

    # ── Arena Mission ─────────────────────────────────────────────────────────

    def _update_arena_mission_state(self, state: str, message: str) -> None:
        with self._arena_mission_lock:
            self._arena_mission_state = state
            self._arena_mission_message = message

    def send_arena_mission_goal(self) -> tuple[bool, str]:
        with self._arena_mission_lock:
            if self._arena_mission_state not in ("idle", "unavailable", "error"):
                return False, f"Cannot start: already in state '{self._arena_mission_state}'"

        if not self._arena_mission_client.server_is_ready():
            if not self._arena_mission_client.wait_for_server(timeout_sec=2.0):
                self._update_arena_mission_state("unavailable", "Arena mission server offline")
                return False, "Arena mission server offline"

        goal_msg = SearchAndRetrieve.Goal()
        goal_msg.target_id = "arena_mode"

        with self._arena_mission_lock:
            self._arena_mission_goal_handle = None
            self._arena_mission_goal = {"target_id": "arena_mode"}
            self._arena_mission_feedback = {}
        
        self._update_arena_mission_state("sending", "goal dispatched")
        future = self._arena_mission_client.send_goal_async(
            goal_msg,
            feedback_callback=self._on_arena_mission_feedback,
        )
        future.add_done_callback(self._on_arena_mission_response)
        return True, "goal dispatched"

    def cancel_arena_mission_goal(self) -> tuple[bool, str]:
        with self._arena_mission_lock:
            handle = self._arena_mission_goal_handle
        if handle is None:
            return False, "no active arena mission goal to cancel"
        self._update_arena_mission_state("canceling", "cancel requested")
        handle.cancel_goal_async()
        return True, "cancel requested"

    def snapshot_arena_mission(self) -> dict[str, Any]:
        with self._arena_mission_lock:
            return {
                "state": self._arena_mission_state,
                "message": self._arena_mission_message,
                "goal": dict(self._arena_mission_goal) if self._arena_mission_goal else None,
                "feedback": dict(self._arena_mission_feedback),
                "server_ready": self._arena_mission_client.server_is_ready(),
            }

    def _on_arena_mission_feedback(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        with self._arena_mission_lock:
            self._arena_mission_feedback = {
                "stage": str(fb.stage),
                "progress": float(fb.progress),
                "detail": str(fb.detail),
            }

    def _on_arena_mission_response(self, future) -> None:
        try:
            goal_handle = future.result()
            with self._arena_mission_lock:
                if not goal_handle.accepted:
                    self._update_arena_mission_state("idle", "goal rejected by server; retry allowed")
                    return
                self._arena_mission_goal_handle = goal_handle
            self._update_arena_mission_state("active", "goal accepted")
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._on_arena_mission_result)
        except Exception as exc:
            self._update_arena_mission_state("error", f"goal send failed: {exc}")

    def _on_arena_mission_result(self, future) -> None:
        try:
            result = future.result().result
            status = future.result().status
            with self._arena_mission_lock:
                self._arena_mission_goal_handle = None
                self._arena_mission_state = "idle"
                self._arena_mission_message = (
                    f"finished (status {status}): {result.message} (success={result.success})"
                )
        except Exception as exc:
            with self._arena_mission_lock:
                self._arena_mission_goal_handle = None
            self._update_arena_mission_state("error", f"result processing failed: {exc}")

    # ── Semantic Memory ──────────────────────────────────────────────────────

    def _on_semantic_memory(self, msg: String) -> None:
        try:
            self._semantic_memory = json.loads(msg.data)
        except Exception:
            pass

    def snapshot_semantic_memory(self) -> dict[str, Any]:
        return self._semantic_memory

    def clear_semantic_memory(self) -> tuple[bool, str]:
        self._semantic_memory = {}
        self._semantic_memory_clear_pub.publish(Empty())
        return True, "semantic memory cleared"

    # ── EKF fusion source freshness ──────────────────────────────────────────

    def _on_wheel_freshness(self, _msg: Odometry) -> None:
        with self._fusion_lock:
            self._fusion_last_seen["wheel"] = time.monotonic()

    def _on_imu_freshness(self, _msg: Imu) -> None:
        with self._fusion_lock:
            self._fusion_last_seen["imu"] = time.monotonic()

    def _on_lidar_freshness(self, _msg: Odometry) -> None:
        with self._fusion_lock:
            self._fusion_last_seen["lidar"] = time.monotonic()

    def _on_camera_freshness(self, _msg: Odometry) -> None:
        with self._fusion_lock:
            self._fusion_last_seen["camera"] = time.monotonic()

    def snapshot_fusion_sources(self) -> dict[str, bool]:
        """Returns {source: True/False} for each sensor, fresh = last message
        within 2 s. Drives the grouped sensor badge in the UI status bar."""
        now = time.monotonic()
        fresh = 2.0
        with self._fusion_lock:
            return {
                "wheel":  (now - self._fusion_last_seen.get("wheel",  0.0)) < fresh,
                "imu":    (now - self._fusion_last_seen.get("imu",    0.0)) < fresh,
                "lidar":  (now - self._fusion_last_seen.get("lidar",  0.0)) < fresh,
                "camera": (now - self._fusion_last_seen.get("camera", 0.0)) < fresh,
            }

    def _visible_action_names(self) -> list[str]:
        try:
            from rclpy.action import get_action_names_and_types  # local import to avoid hard dep at module load
            pairs = get_action_names_and_types(self)
            return sorted(name for name, _types in pairs)
        except Exception:  # noqa: BLE001
            return []

    def _nav_diagnostic_hint(self) -> str:
        actions = self._visible_action_names()
        if not actions:
            return (
                "/navigate_to_pose action server not discoverable. "
                "Is nav2 running? (docker compose --profile navigation up -d). "
                "Also check ROS_DOMAIN_ID and DDS profile match between bridge and nav2."
            )
        nav_like = [a for a in actions if "navigate" in a.lower()]
        if nav_like:
            return (
                f"navigate_to_pose action not ready. Discovered similar action names: {', '.join(nav_like)}. "
                "Either the lifecycle manager hasn't activated bt_navigator yet, or the action is namespaced."
            )
        return (
            f"navigate_to_pose action not ready. {len(actions)} actions visible "
            f"({', '.join(actions[:6])}{'…' if len(actions) > 6 else ''}). "
            "Likely nav2 stack is down."
        )

    def _update_nav_state(self, state: str, message: str = "") -> None:
        with self._nav_lock:
            self._nav_state = state
            self._nav_message = message

    def _on_nav_feedback(self, feedback_msg: Any) -> None:
        try:
            fb = feedback_msg.feedback
            payload = {
                "distance_remaining": float(getattr(fb, "distance_remaining", 0.0)),
                "navigation_time": float(getattr(fb.navigation_time, "sec", 0))
                + float(getattr(fb.navigation_time, "nanosec", 0)) * 1e-9,
                "estimated_time_remaining": float(getattr(fb.estimated_time_remaining, "sec", 0))
                + float(getattr(fb.estimated_time_remaining, "nanosec", 0)) * 1e-9,
                "recoveries": int(getattr(fb, "number_of_recoveries", 0)),
            }
        except Exception:  # noqa: BLE001
            payload = {}
        with self._nav_lock:
            self._nav_feedback = payload
            if self._nav_state in ("sending", "accepted"):
                self._nav_state = "navigating"
                self._nav_message = "executing"

    def _on_nav_response(self, future: Any, goal_token: int) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            with self._nav_lock:
                if goal_token != self._nav_goal_token:
                    return
            self._update_nav_state("failed", f"send error: {exc}")
            return
        if not goal_handle.accepted:
            with self._nav_lock:
                if goal_token != self._nav_goal_token:
                    return
            self._update_nav_state("rejected", "goal rejected by server")
            return
        with self._nav_lock:
            if goal_token != self._nav_goal_token:
                return
            self._nav_goal_handle = goal_handle
            self._nav_state = "accepted"
            self._nav_message = "goal accepted"
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda done, token=goal_token: self._on_nav_result(done, token))

    def _on_nav_result(self, future: Any, goal_token: int) -> None:
        try:
            wrapped = future.result()
        except Exception as exc:  # noqa: BLE001
            with self._nav_lock:
                if goal_token != self._nav_goal_token:
                    return
            self._update_nav_state("failed", f"result error: {exc}")
            with self._nav_lock:
                self._nav_goal_handle = None
            return
        status_map = {
            GoalStatus.STATUS_SUCCEEDED: ("succeeded", "goal reached"),
            GoalStatus.STATUS_ABORTED: ("aborted", "goal aborted"),
            GoalStatus.STATUS_CANCELED: ("canceled", "goal canceled"),
        }
        state, message = status_map.get(wrapped.status, ("failed", f"status {wrapped.status}"))
        with self._nav_lock:
            if goal_token != self._nav_goal_token:
                return
            self._nav_state = state
            self._nav_message = message
            self._nav_goal_handle = None
        self._door_mission.on_nav_result(goal_token, state, message)

    def _on_nav_cancel_response(self, future: Any) -> None:
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self._update_nav_state("failed", f"cancel error: {exc}")
            return
        if not getattr(response, "goals_canceling", None):
            self._update_nav_state("canceled", "no active goal to cancel")
            with self._nav_lock:
                self._nav_goal_handle = None


# ── Helpers ──────────────────────────────────────────────────────────────────

def _pose_from_q(x: float, y: float, q: Any) -> dict[str, float]:
    yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )
    return {"x": float(x), "y": float(y), "yaw": float(yaw)}


def _encode_jpeg(msg: Image) -> bytes | None:
    if PILImage is None:
        return None
    w, h, enc = int(msg.width), int(msg.height), str(msg.encoding)
    raw = bytes(msg.data)
    try:
        if enc == "rgb8":
            img = PILImage.frombytes("RGB", (w, h), raw)
        elif enc == "bgr8":
            img = PILImage.frombytes("RGB", (w, h), raw)
            b, g, r = img.split()
            img = PILImage.merge("RGB", (r, g, b))
        elif enc == "rgba8":
            img = PILImage.frombytes("RGBA", (w, h), raw).convert("RGB")
        elif enc == "bgra8":
            img = PILImage.frombytes("RGBA", (w, h), raw)
            b, g, r, _a = img.split()
            img = PILImage.merge("RGB", (r, g, b))
        elif enc in ("mono8", "8UC1"):
            img = PILImage.frombytes("L", (w, h), raw)
        else:
            return None
    except (ValueError, OSError):
        return None

    if w > JPEG_MAX_WIDTH:
        new_h = max(1, int(h * JPEG_MAX_WIDTH / w))
        img = img.resize((JPEG_MAX_WIDTH, new_h))
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=JPEG_QUALITY)
    return buf.getvalue()


def _parse_imu_calibration_status(raw: str) -> dict[str, Any]:
    fields: dict[str, str] = {}
    for token in raw.split():
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        fields[key] = value

    state = fields.get("state", "unknown")
    bias_text = fields.get("gyro_bias", "").strip("[]")
    gyro_bias: list[float] = []
    if bias_text:
        try:
            gyro_bias = [float(item) for item in bias_text.split(",")]
        except ValueError:
            gyro_bias = []

    return {
        "ok": state != "unknown",
        "state": state,
        "message": raw,
        "stationary": _as_bool(fields.get("stationary")),
        "converged": _as_bool(fields.get("converged")),
        "online": _as_bool(fields.get("online")),
        "manual_required": _as_bool(fields.get("manual_required")),
        "manual_active": _as_bool(fields.get("manual_active")),
        "calibration_active": _as_bool(fields.get("calibration_active")),
        "manual_remaining_s": _as_float(fields.get("manual_remaining_s")),
        "stationary_age_s": _as_float(fields.get("stationary_age_s")),
        "convergence_age_s": _as_float(fields.get("convergence_age_s")),
        "gyro_error_rad_s": _as_float(fields.get("gyro_error_rad_s")),
        "gyro_bias": gyro_bias,
        "raw": raw,
    }


def _as_bool(value: str | None) -> bool:
    return str(value).lower() == "true"


def _as_float(value: str | None) -> float:
    try:
        return float(value) if value is not None else 0.0
    except ValueError:
        return 0.0


# ── HTTP server ──────────────────────────────────────────────────────────────

def _make_handler(node: BridgeNode) -> type[BaseHTTPRequestHandler]:
    class _Handler(BaseHTTPRequestHandler):
        def log_message(self, *_args: Any) -> None:
            pass

        def do_OPTIONS(self) -> None:  # noqa: N802
            self.send_response(200)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")
            self.end_headers()

        def do_GET(self) -> None:  # noqa: N802
            parsed = urlparse(self.path)
            path = parsed.path
            query = {k: v[0] for k, v in parse_qs(parsed.query).items()}
            if path == "/api/map":
                data = node.snapshot_map()
                self._json(data if data is not None else {})
            elif path == "/api/costmap":
                data = node.snapshot_costmap()
                self._json(data if data is not None else {})
            elif path == "/api/plan":
                data = node.snapshot_plan()
                self._json(data if data is not None else {})
            elif path == "/api/approach_pose":
                data = node.snapshot_approach_pose()
                self._json(data if data is not None else {})
            elif path == "/api/pose":
                self._json(node.snapshot_pose())
            elif path == "/api/health":
                self._json(node.snapshot_health())
            elif path == "/api/nav/status":
                self._json(node.snapshot_nav())
            elif path == "/api/semantic_memory":
                self._json(node.snapshot_semantic_memory())
            elif path == "/api/search_retrieve/status":
                self._json(node.snapshot_search())
            elif path == "/api/bridge_retrieve/status":
                self._json(node.snapshot_bridge_mission())
            elif path == "/api/bridge_traverse/status":
                self._json(node.snapshot_bridge_traverse())
            elif path == "/api/arena_mission/status":
                self._json(node.snapshot_arena_mission())

            elif path == "/api/waypoints":
                self._json({"waypoints": node.list_waypoints()})
            elif path == "/api/motion/state":
                self._json({"state": node.snapshot_motion_state()})
            elif path == "/api/estop":
                self._json(node.snapshot_estop())
            elif path == "/api/arm/temperatures":
                self._json(node.snapshot_arm_temperatures())
            elif path == "/api/imu/calibration":
                self._json(node.snapshot_imu_calibration())
            elif path == "/api/open_door/status":
                self._json(node.snapshot_open_door())
            elif path == "/api/door_mission/status":
                self._json(node.snapshot_door_mission())
            elif path == "/api/image/topics":
                self._json({"topics": node.list_image_topics()})
            elif path == "/api/image/frame":
                requested = query.get("topic") or None
                if requested is not None:
                    node.set_image_topic(requested)
                jpeg, active, meta = node.snapshot_image()
                if jpeg is None:
                    self.send_response(204)
                    self.send_header("X-Active-Topic", active or "")
                    self.send_header("Access-Control-Allow-Origin", "*")
                    self.end_headers()
                else:
                    self.send_response(200)
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(jpeg)))
                    self.send_header("X-Active-Topic", active or "")
                    self.send_header("X-Frame-Width", str(meta.get("width", 0)))
                    self.send_header("X-Frame-Height", str(meta.get("height", 0)))
                    self.send_header("X-Frame-Encoding", str(meta.get("encoding", "")))
                    self.send_header("Access-Control-Allow-Origin", "*")
                    self.end_headers()
                    self.wfile.write(jpeg)
            else:
                self.send_response(404)
                self.end_headers()

        def do_POST(self) -> None:  # noqa: N802
            path = urlparse(self.path).path
            length = int(self.headers.get("Content-Length", 0))
            body: dict[str, Any] = json.loads(self.rfile.read(length)) if length else {}
            if path == "/api/cmd_vel":
                node.publish_cmd_vel(float(body.get("linear_x", 0.0)), float(body.get("angular_z", 0.0)))
                self._json({"ok": True, "action": "cmd_vel"})
            elif path == "/api/stop":
                ok, msg = node.emergency_stop()
                self._json({"ok": ok, "action": "stop", "message": msg})
            elif path == "/api/estop":
                ok, engaged, msg = node.set_estop(bool(body.get("engaged", True)))
                self._json({"ok": ok, "action": "estop", "engaged": engaged, "message": msg})
            elif path == "/api/goal_pose":
                node.publish_goal_pose(
                    float(body.get("x", 0.0)), float(body.get("y", 0.0)),
                    float(body.get("yaw", 0.0)), str(body.get("frame_id", "map")),
                )
                self._json({"ok": True, "action": "goal_pose"})
            elif path == "/api/initial_pose":
                node.publish_initial_pose(
                    float(body.get("x", 0.0)), float(body.get("y", 0.0)),
                    float(body.get("yaw", 0.0)), str(body.get("frame_id", "map")),
                )
                self._json({"ok": True, "action": "initial_pose"})
            elif path == "/api/arm/trajectory":
                positions = body.get("positions") or []
                tfs = float(body.get("time_from_start", 0.3))
                try:
                    node.publish_arm_trajectory([float(p) for p in positions], tfs)
                except ValueError as exc:
                    self.send_response(400)
                    self.send_header("Access-Control-Allow-Origin", "*")
                    self.end_headers()
                    self.wfile.write(str(exc).encode())
                    return
                self._json({"ok": True, "action": "arm_trajectory"})
            elif path == "/api/nav/goal":
                ok, msg = node.send_nav_goal(
                    float(body.get("x", 0.0)),
                    float(body.get("y", 0.0)),
                    float(body.get("yaw", 0.0)),
                )
                self._json({"ok": ok, "action": "nav_goal", "message": msg})
            elif path == "/api/nav/cancel":
                ok, msg = node.cancel_nav_goal()
                self._json({"ok": ok, "action": "nav_cancel", "message": msg})
            elif path == "/api/search_retrieve/start":
                ok, msg = node.send_search_goal(
                    str(body.get("target_id", "")),
                    float(body.get("home_pose_x", 0.0)),
                    float(body.get("home_pose_y", 0.0)),
                    float(body.get("home_pose_yaw", 0.0)),
                )
                self._json({"ok": ok, "action": "search_start", "message": msg})
            elif path == "/api/search_retrieve/cancel":
                ok, msg = node.cancel_search_goal()
                self._json({"ok": ok, "action": "search_cancel", "message": msg})
            elif path == "/api/bridge_retrieve/start":
                waypoint_name = str(body.get("bridge_waypoint_name", ""))
                if waypoint_name:
                    ok, msg = node.send_bridge_mission_waypoint_goal(
                        str(body.get("target_class", "xiong_qiao")),
                        waypoint_name,
                    )
                else:
                    ok, msg = node.send_bridge_mission_goal(
                        str(body.get("target_class", "xiong_qiao")),
                        float(body.get("bridge_pose_x", 0.0)),
                        float(body.get("bridge_pose_y", 0.0)),
                        float(body.get("bridge_pose_yaw", 0.0)),
                    )
                self._json({"ok": ok, "action": "bridge_retrieve_start", "message": msg})
            elif path == "/api/bridge_retrieve/cancel":
                ok, msg = node.cancel_bridge_mission_goal()
                self._json({"ok": ok, "action": "bridge_retrieve_cancel", "message": msg})
            elif path == "/api/bridge_traverse/start":
                waypoint_name = str(body.get("bridge_waypoint_name", ""))
                if waypoint_name:
                    ok, msg = node.send_bridge_traverse_waypoint_goal(waypoint_name)
                else:
                    ok, msg = node.send_bridge_traverse_goal(
                        float(body.get("bridge_pose_x", 0.0)),
                        float(body.get("bridge_pose_y", 0.0)),
                        float(body.get("bridge_pose_yaw", 0.0)),
                    )
                self._json({"ok": ok, "action": "bridge_traverse_start", "message": msg})
            elif path == "/api/bridge_traverse/cancel":
                ok, msg = node.cancel_bridge_traverse_goal()
                self._json({"ok": ok, "action": "bridge_traverse_cancel", "message": msg})
            elif path == "/api/arena_mission/start":
                ok, msg = node.send_arena_mission_goal()
                self._json({"ok": ok, "action": "arena_mission_start", "message": msg})
            elif path == "/api/arena_mission/cancel":
                ok, msg = node.cancel_arena_mission_goal()
                self._json({"ok": ok, "action": "arena_mission_cancel", "message": msg})

            elif path == "/api/waypoints/save":
                x = body.get("x")
                y = body.get("y")
                ok, msg = node.save_waypoint(
                    str(body.get("name", "")),
                    None if x is None else float(x),
                    None if y is None else float(y),
                    float(body.get("yaw", 0.0)),
                )
                self._json({"ok": ok, "action": "waypoint_save", "message": msg})
            elif path == "/api/waypoints/delete":
                ok, msg = node.delete_waypoint(str(body.get("name", "")))
                self._json({"ok": ok, "action": "waypoint_delete", "message": msg})
            elif path == "/api/waypoints/goto":
                ok, msg = node.goto_waypoint(str(body.get("name", "")))
                self._json({"ok": ok, "action": "waypoint_goto", "message": msg})
            elif path == "/api/map/save":
                filename = str(body.get("filename", "arena_map"))
                ok, msg = node.save_map(filename=filename)
                self._json({"ok": ok, "action": "map_save", "message": msg})
            elif path == "/api/costmap/clear":
                target = str(body.get("target", "local"))
                ok, msg = node.clear_costmap(target=target)
                self._json({"ok": ok, "action": "costmap_clear", "message": msg})
            elif path == "/api/imu/calibration/start":
                node.start_imu_calibration()
                self._json({"ok": True, "action": "imu_calibration_start", "message": "IMU calibration window started"})
            elif path == "/api/params/set":
                target_node = str(body.get("node", "")).strip()
                params = body.get("params") or {}
                if not target_node or not isinstance(params, dict) or not params:
                    self.send_response(400)
                    self.send_header("Access-Control-Allow-Origin", "*")
                    self.end_headers()
                    self.wfile.write(b"node and non-empty params dict required")
                    return
                ok, msg = node.set_remote_parameters(target_node, params)
                self._json({"ok": ok, "action": "params_set", "message": msg})
            elif path == "/api/open_door/start":
                ok, msg = node.send_open_door_goal(float(body.get("ready_distance_m", 0.0)))
                self._json({"ok": ok, "action": "open_door_start", "message": msg})
            elif path == "/api/open_door/cancel":
                ok, msg = node.cancel_open_door_goal()
                self._json({"ok": ok, "action": "open_door_cancel", "message": msg})
            elif path == "/api/open_door/save_poses":
                ok, msg = node.save_open_door_poses()
                self._json({"ok": ok, "action": "open_door_save_poses", "message": msg})
            elif path == "/api/open_door/step":
                step = str(body.get("step", ""))
                ok, msg = node.call_open_door_step(step)
                self._json({"ok": ok, "action": f"open_door_{step}", "message": msg})
            elif path == "/api/door_mission/start":
                ok, msg = node.start_door_mission(
                    waypoint=str(body.get("waypoint", "door_approach")),
                    ready_distance_m=float(body.get("ready_distance_m", 0.0)),
                )
                self._json({"ok": ok, "action": "door_mission_start", "message": msg})
            elif path == "/api/door_mission/cancel":
                ok, msg = node.cancel_door_mission()
                self._json({"ok": ok, "action": "door_mission_cancel", "message": msg})
            elif path == "/api/semantic_memory/clear":
                ok, msg = node.clear_semantic_memory()
                self._json({"ok": ok, "action": "semantic_memory_clear", "message": msg})
            else:
                self.send_response(404)
                self.end_headers()

        def _json(self, obj: Any) -> None:
            body = json.dumps(obj).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(body)

    return _Handler


# ── Entry point ──────────────────────────────────────────────────────────────

def main() -> None:
    rclpy.init()
    node = BridgeNode()

    server = ThreadingHTTPServer(("0.0.0.0", 8771), _make_handler(node))
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
