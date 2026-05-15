from __future__ import annotations

import io
import json
import math
import os
import struct
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any
from urllib.parse import parse_qs, urlparse

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, TwistStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

try:
    from PIL import Image as PILImage  # type: ignore
except ImportError:  # pragma: no cover
    PILImage = None  # type: ignore

_MAP_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

ARM_JOINT_NAMES = ["arm_1_joint", "arm_2_joint", "gripper_joint"]
JPEG_MAX_WIDTH = 800
JPEG_QUALITY = 70


class BridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("igvi_bridge")
        self._lock = threading.Lock()
        self._map: dict[str, Any] | None = None
        self._pose: dict[str, float] = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self._pose_source = "none"

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

        self.create_subscription(OccupancyGrid, "/map", self._on_map, _MAP_QOS)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl, 10)

        # Manual override commands go to /motion/cmd (Twist) so motion_arbiter
        # owns the path → /cmd_vel pipeline. We also relay motion_arbiter's
        # /cmd_vel output onto /base_controller/cmd_vel for the wheel driver.
        self._motion_cmd_pub = self.create_publisher(Twist, "/motion/cmd", 10)
        self._wheel_cmd_pub = self.create_publisher(TwistStamped, "/base_controller/cmd_vel", 10)
        self._goal_pose_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self._initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self._arm_pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        # Relay motion_arbiter's /cmd_vel (TwistStamped) to /base_controller/cmd_vel.
        self.create_subscription(TwistStamped, "/cmd_vel", self._on_nav_cmd_vel_stamped, 10)
        # Track latest arbiter state for HTTP diagnostics.
        self._motion_state: str = "unknown"
        self.create_subscription(String, "/motion/state", self._on_motion_state, 10)
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

    def _on_odom(self, msg: Odometry) -> None:
        if self._pose_source == "amcl":
            return
        with self._lock:
            self._pose = _pose_from_q(
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.orientation,
            )
            self._pose_source = "odom"

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        with self._lock:
            self._pose = _pose_from_q(
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.orientation,
            )
            self._pose_source = "amcl"

    def _on_nav_cmd_vel_stamped(self, msg: TwistStamped) -> None:
        # Re-stamp before forwarding so the wheel controller's cmd_vel_timeout
        # measures against the bridge clock (DDS delivery may lag).
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = msg.header.frame_id
        out.twist = msg.twist
        self._wheel_cmd_pub.publish(out)

    def _on_motion_state(self, msg) -> None:  # std_msgs/String
        self._motion_state = str(getattr(msg, "data", ""))

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

    # ── Thread-safe snapshots ────────────────────────────────────────────────

    def snapshot_map(self) -> dict[str, Any] | None:
        with self._lock:
            return dict(self._map) if self._map else None

    def snapshot_pose(self) -> dict[str, float]:
        with self._lock:
            return dict(self._pose)

    def snapshot_health(self) -> dict[str, Any]:
        with self._lock:
            return {
                "ok": True,
                "map": self._map is not None,
                "pose_source": self._pose_source,
            }

    def publish_cmd_vel(self, linear_x: float, angular_z: float) -> None:
        # Manual override flows through motion_arbiter: publish a Twist on
        # /motion/cmd; arbiter will preempt path tracking and publish the
        # actual /cmd_vel which the bridge relays to /base_controller/cmd_vel.
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self._motion_cmd_pub.publish(msg)

    def snapshot_motion_state(self) -> str:
        return self._motion_state

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
            if "sensor_msgs/msg/Image" in types:
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
                self._image_sub = self.create_subscription(
                    Image, topic, self._on_image, qos_profile_sensor_data
                )

    def snapshot_image(self) -> tuple[bytes | None, str | None, dict[str, Any]]:
        with self._image_lock:
            return self._image_jpeg, self._image_topic, dict(self._image_meta)

    # ── Map saving ────────────────────────────────────────────────────────────

    def save_map(self, filename: str = "arena_map", out_dir: str = "/maps") -> tuple[bool, str]:
        with self._lock:
            snap = dict(self._map) if self._map else None
        if snap is None:
            return False, "no map data available"
        w, h = int(snap["width"]), int(snap["height"])
        res = float(snap["resolution"])
        ox, oy = float(snap["origin_x"]), float(snap["origin_y"])
        data = snap["data"]

        os.makedirs(out_dir, exist_ok=True)
        pgm_path = os.path.join(out_dir, f"{filename}.pgm")
        yaml_path = os.path.join(out_dir, f"{filename}.yaml")

        with open(pgm_path, "wb") as f:
            f.write(f"P5\n{w} {h}\n255\n".encode())
            for row in range(h - 1, -1, -1):
                for col in range(w):
                    cell = data[row * w + col]
                    if cell < 0:
                        px = 205
                    else:
                        px = max(0, min(255, 255 - int(cell * 255 / 100)))
                    f.write(struct.pack("B", px))

        yaml_content = (
            f"image: {pgm_path}\n"
            f"resolution: {res}\n"
            f"origin: [{ox}, {oy}, 0.0]\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.196\n"
        )
        with open(yaml_path, "w") as f:
            f.write(yaml_content)

        self.get_logger().info(f"Map saved to {yaml_path}")
        return True, yaml_path

    # ── Navigation (Nav2 NavigateToPose action) ──────────────────────────────

    def send_nav_goal(self, x: float, y: float, yaw: float) -> tuple[bool, str]:
        if not self._nav_client.server_is_ready():
            if not self._nav_client.wait_for_server(timeout_sec=3.5):
                hint = self._nav_diagnostic_hint()
                self._update_nav_state("unavailable", hint)
                return False, hint
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        half = float(yaw) / 2.0
        goal_msg.pose.pose.orientation.z = math.sin(half)
        goal_msg.pose.pose.orientation.w = math.cos(half)
        with self._nav_lock:
            self._nav_goal = {"x": float(x), "y": float(y), "yaw": float(yaw)}
            self._nav_feedback = {}
        self._update_nav_state("sending", "goal dispatched")
        future = self._nav_client.send_goal_async(goal_msg, feedback_callback=self._on_nav_feedback)
        future.add_done_callback(self._on_nav_response)
        return True, "goal dispatched"

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
            return {
                "state": self._nav_state,
                "message": self._nav_message,
                "goal": dict(self._nav_goal) if self._nav_goal else None,
                "feedback": dict(self._nav_feedback),
                "server_ready": self._nav_client.server_is_ready(),
                "visible_actions": self._visible_action_names(),
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

    def _on_nav_response(self, future: Any) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self._update_nav_state("failed", f"send error: {exc}")
            return
        if not goal_handle.accepted:
            self._update_nav_state("rejected", "goal rejected by server")
            return
        with self._nav_lock:
            self._nav_goal_handle = goal_handle
            self._nav_state = "accepted"
            self._nav_message = "goal accepted"
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future: Any) -> None:
        try:
            wrapped = future.result()
        except Exception as exc:  # noqa: BLE001
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
            self._nav_state = state
            self._nav_message = message
            self._nav_goal_handle = None

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
            elif path == "/api/pose":
                self._json(node.snapshot_pose())
            elif path == "/api/health":
                self._json(node.snapshot_health())
            elif path == "/api/nav/status":
                self._json(node.snapshot_nav())
            elif path == "/api/motion/state":
                self._json({"state": node.snapshot_motion_state()})
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
                node.publish_cmd_vel(0.0, 0.0)
                self._json({"ok": True, "action": "stop"})
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
            elif path == "/api/map/save":
                filename = str(body.get("filename", "arena_map"))
                ok, msg = node.save_map(filename=filename)
                self._json({"ok": ok, "action": "map_save", "message": msg})
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
