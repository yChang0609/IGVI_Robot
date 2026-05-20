from __future__ import annotations

import json
import math
import statistics
from typing import Any

import numpy as np
import rclpy
import tf2_ros
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


def _stamp_to_float(stamp: Any) -> float:
    return float(getattr(stamp, "sec", 0)) + float(getattr(stamp, "nanosec", 0)) * 1e-9


def _q_rotate(q: Any, v: tuple[float, float, float]) -> tuple[float, float, float]:
    # Quaternion vector rotation: v' = v + 2*w*(qxyz x v) + 2*(qxyz x (qxyz x v))
    qx, qy, qz, qw = float(q.x), float(q.y), float(q.z), float(q.w)
    vx, vy, vz = v
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


def _transform_point(
    transform: Any,
    point: tuple[float, float, float],
) -> tuple[float, float, float]:
    rotated = _q_rotate(transform.transform.rotation, point)
    t = transform.transform.translation
    return (
        rotated[0] + float(t.x),
        rotated[1] + float(t.y),
        rotated[2] + float(t.z),
    )


class DetectionProjector(Node):
    """Project ETO/YOLO 2-D detections into base_link and map coordinates."""

    def __init__(self) -> None:
        super().__init__("detection_projector")

        self.declare_parameter("detections_topic", "/detections_json")
        self.declare_parameter("depth_topic", "/depth_to_rgb/image_raw")
        self.declare_parameter("camera_info_topic", "/rgb/camera_info")
        self.declare_parameter("output_topic", "/task/target_candidates")
        self.declare_parameter("target_classes", ["xiong", "xiong_qiao", "bear"])
        self.declare_parameter("fallback_to_any_class", False)
        self.declare_parameter("min_score", 0.20)
        self.declare_parameter("depth_window_px", 9)
        self.declare_parameter("max_depth_age_s", 0.8)
        self.declare_parameter("max_depth_m", 4.0)
        self.declare_parameter("min_depth_m", 0.15)

        self._depth: Image | None = None
        self._camera_info: CameraInfo | None = None
        self._last_payload: dict[str, Any] | None = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        detections_topic = str(self.get_parameter("detections_topic").value)
        depth_topic = str(self.get_parameter("depth_topic").value)
        camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)

        self.create_subscription(String, detections_topic, self._on_detections, 10)
        self.create_subscription(Image, depth_topic, self._on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, camera_info_topic, self._on_camera_info, 10)
        self._pub = self.create_publisher(String, output_topic, 10)

        self.get_logger().info(
            "detection_projector started: "
            f"{detections_topic} + {depth_topic} -> {output_topic}"
        )

    def _on_depth(self, msg: Image) -> None:
        self._depth = msg

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._camera_info = msg

    def _on_detections(self, msg: String) -> None:
        try:
            raw = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"bad detections_json: {exc}")
            return

        detections = raw.get("detections") or []
        if not detections:
            return
        if self._depth is None or self._camera_info is None:
            self.get_logger().warn("waiting for depth image and camera_info")
            return

        candidates = self._candidate_detections(detections)
        if not candidates:
            return
        camera_frame = str(raw.get("frame_id") or self._camera_info.header.frame_id)

        try:
            base_tf = self._tf_buffer.lookup_transform("base_link", camera_frame, rclpy.time.Time())
            map_tf = self._tf_buffer.lookup_transform("map", camera_frame, rclpy.time.Time())
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"TF unavailable for {camera_frame}: {exc}")
            return

        projected = [
            item
            for det in candidates
            if (item := self._project_detection(det, raw, camera_frame, base_tf, map_tf)) is not None
        ]
        if not projected:
            return

        selected = max(projected, key=lambda det: float(det.get("confidence", 0.0)))
        payload = dict(selected)
        payload["detections"] = projected
        payload["selected_index"] = projected.index(selected)
        payload["candidate_count"] = len(projected)
        self._last_payload = payload

        out = String()
        out.data = json.dumps(payload, separators=(",", ":"))
        self._pub.publish(out)
        self.get_logger().info(
            f"target {payload['class_name']} score={payload['confidence']:.2f} "
            f"base=({payload['x_base']:.2f},{payload['y_base']:.2f}) "
            f"map=({payload['x_map']:.2f},{payload['y_map']:.2f})"
        )

    def _candidate_detections(self, detections: list[dict[str, Any]]) -> list[dict[str, Any]]:
        target_classes = {
            str(x).strip()
            for x in self.get_parameter("target_classes").value
            if str(x).strip()
        }
        min_score = float(self.get_parameter("min_score").value)
        fallback = bool(self.get_parameter("fallback_to_any_class").value)

        candidates = [
            det for det in detections
            if float(det.get("score", det.get("confidence", 0.0))) >= min_score
            and str(det.get("class_name", "")) in target_classes
        ]
        if not candidates and fallback:
            candidates = [
                det for det in detections
                if float(det.get("score", det.get("confidence", 0.0))) >= min_score
            ]
        if not candidates:
            return []
        return candidates

    def _project_detection(
        self,
        detection: dict[str, Any],
        raw: dict[str, Any],
        camera_frame: str,
        base_tf: Any,
        map_tf: Any,
    ) -> dict[str, Any] | None:
        bbox = detection.get("bbox") or {}
        try:
            u = float(bbox.get("center_x"))
            v = float(bbox.get("center_y"))
        except (TypeError, ValueError):
            self.get_logger().warn(f"detection missing bbox center: {detection}")
            return None

        depth_m = self._sample_depth(u, v)
        if depth_m is None:
            return None

        camera_point = self._deproject(u, v, depth_m)
        base_point = _transform_point(base_tf, camera_point)
        map_point = _transform_point(map_tf, camera_point)
        return {
            "stamp": raw.get("stamp"),
            "source_frame": camera_frame,
            "class_id": detection.get("class_id"),
            "class_name": detection.get("class_name"),
            "confidence": float(detection.get("score", detection.get("confidence", 0.0))),
            "bbox": bbox,
            "depth_m": depth_m,
            "x_camera": camera_point[0],
            "y_camera": camera_point[1],
            "z_camera": camera_point[2],
            "x_base": base_point[0],
            "y_base": base_point[1],
            "z_base": base_point[2],
            "x_map": map_point[0],
            "y_map": map_point[1],
            "z_map": map_point[2],
        }

    def _sample_depth(self, u: float, v: float) -> float | None:
        assert self._depth is not None
        depth_age = abs(self.get_clock().now().nanoseconds * 1e-9 - _stamp_to_float(self._depth.header.stamp))
        if depth_age > float(self.get_parameter("max_depth_age_s").value):
            self.get_logger().warn(f"depth image too old: {depth_age:.2f}s")
            return None

        if self._depth.encoding != "16UC1":
            self.get_logger().warn(f"unsupported depth encoding: {self._depth.encoding}")
            return None

        width = int(self._depth.width)
        height = int(self._depth.height)
        cx = int(round(u))
        cy = int(round(v))
        if cx < 0 or cx >= width or cy < 0 or cy >= height:
            self.get_logger().warn(f"bbox center outside depth image: ({u:.1f}, {v:.1f})")
            return None

        arr = np.frombuffer(self._depth.data, dtype=np.uint16).reshape((height, width))
        half = max(1, int(self.get_parameter("depth_window_px").value) // 2)
        x0 = max(0, cx - half)
        x1 = min(width, cx + half + 1)
        y0 = max(0, cy - half)
        y1 = min(height, cy + half + 1)
        values_m = [
            float(x) * 0.001
            for x in arr[y0:y1, x0:x1].ravel()
            if x > 0
        ]
        min_depth = float(self.get_parameter("min_depth_m").value)
        max_depth = float(self.get_parameter("max_depth_m").value)
        values_m = [x for x in values_m if min_depth <= x <= max_depth]
        if not values_m:
            self.get_logger().warn(f"no valid depth near bbox center ({u:.1f}, {v:.1f})")
            return None
        return float(statistics.median(values_m))

    def _deproject(self, u: float, v: float, depth_m: float) -> tuple[float, float, float]:
        assert self._camera_info is not None
        k = self._camera_info.k
        fx = float(k[0])
        fy = float(k[4])
        cx = float(k[2])
        cy = float(k[5])
        x = (float(u) - cx) * depth_m / fx
        y = (float(v) - cy) * depth_m / fy
        z = depth_m
        return x, y, z


def main() -> None:
    rclpy.init()
    node = DetectionProjector()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
