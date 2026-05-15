from __future__ import annotations

import json
import math
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from typing import Any

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

_MAP_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class BridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("igvi_bridge")
        self._lock = threading.Lock()
        self._map: dict[str, Any] | None = None
        self._pose: dict[str, float] = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self._pose_source = "none"

        self.create_subscription(OccupancyGrid, "/map", self._on_map, _MAP_QOS)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl, 10)
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


# ── Helpers ──────────────────────────────────────────────────────────────────

def _pose_from_q(x: float, y: float, q: Any) -> dict[str, float]:
    yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )
    return {"x": float(x), "y": float(y), "yaw": float(yaw)}


# ── HTTP server ──────────────────────────────────────────────────────────────

def _make_handler(node: BridgeNode) -> type[BaseHTTPRequestHandler]:
    class _Handler(BaseHTTPRequestHandler):
        def log_message(self, *_args: Any) -> None:
            pass

        def do_GET(self) -> None:  # noqa: N802
            path = self.path.split("?")[0]
            if path == "/api/map":
                data = node.snapshot_map()
                self._json(data if data is not None else {})
            elif path == "/api/pose":
                self._json(node.snapshot_pose())
            elif path == "/api/health":
                self._json(node.snapshot_health())
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

    server = HTTPServer(("0.0.0.0", 8771), _make_handler(node))
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
