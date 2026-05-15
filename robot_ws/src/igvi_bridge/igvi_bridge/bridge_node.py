from __future__ import annotations

import io
import json
import math
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any
from urllib.parse import parse_qs, urlparse

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
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

        self.create_subscription(OccupancyGrid, "/map", self._on_map, _MAP_QOS)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl, 10)

        self._cmd_vel_pub = self.create_publisher(TwistStamped, "/base_controller/cmd_vel", 10)
        self._goal_pose_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self._initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self._arm_pub = self.create_publisher(JointTrajectory, "/arm_controller/joint_trajectory", 10)
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
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = linear_x
        msg.twist.angular.z = angular_z
        self._cmd_vel_pub.publish(msg)

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
