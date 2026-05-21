"""Object detection subscriber wrapper.

Subscribes to /detections (std_msgs/String, JSON format from eto_eye).
find() blocks until a detection matching the label appears or timeout elapses.

Expected JSON schema published by eto_eye (DETECTION_FORMAT=json):
  {"detections": [{"class_id": "bear", "confidence": 0.87,
                   "x": 320, "y": 240, "w": 100, "h": 150,
                   "distance_m": 0.45}]}   ← distance_m present if depth enabled
"""
from __future__ import annotations

import json
import threading
import time
from dataclasses import dataclass
from typing import Optional

from rclpy.node import Node
from std_msgs.msg import String

from igvi_task_core.fsm import TaskFSM


@dataclass
class Detection:
    class_id: str
    confidence: float
    bbox_x: float
    bbox_y: float
    bbox_w: float
    bbox_h: float
    distance_m: float  # 0.0 if depth unavailable


class Detector:
    def __init__(self, node: Node, topic: str = "/detections") -> None:
        self._lock = threading.Lock()
        self._latest: list[Detection] = []
        self._sub = node.create_subscription(String, topic, self._cb, 10)
        node.get_logger().info(f"Detector subscribing to {topic}")

    def _cb(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            detections = [
                Detection(
                    class_id=str(d.get("class_id", "")),
                    confidence=float(d.get("confidence", 0.0)),
                    bbox_x=float(d.get("x", 0.0)),
                    bbox_y=float(d.get("y", 0.0)),
                    bbox_w=float(d.get("w", 0.0)),
                    bbox_h=float(d.get("h", 0.0)),
                    distance_m=float(d.get("distance_m", 0.0)),
                )
                for d in data.get("detections", [])
            ]
            with self._lock:
                self._latest = detections
        except Exception:
            pass

    def find(
        self,
        label: str,
        fsm: TaskFSM,
        timeout: float = 10.0,
        min_confidence: float = 0.5,
    ) -> Optional[Detection]:
        """Block until a detection matching *label* arrives or timeout.

        Returns the best-confidence Detection, or None on timeout.
        Raises InterruptedError if FSM is interrupted.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            fsm.check()
            with self._lock:
                candidates = [
                    d for d in self._latest
                    if d.class_id == label and d.confidence >= min_confidence
                ]
            if candidates:
                return max(candidates, key=lambda d: d.confidence)
            time.sleep(0.1)
        return None

    def peek(self, label: str, min_confidence: float = 0.5) -> Optional[Detection]:
        """Non-blocking check for a current detection (no FSM check)."""
        with self._lock:
            candidates = [
                d for d in self._latest
                if d.class_id == label and d.confidence >= min_confidence
            ]
        return max(candidates, key=lambda d: d.confidence) if candidates else None
