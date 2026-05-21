"""
igvi_task_core — TaskManagerNode

ROS 2 node that owns the task state machine and exposes a lightweight HTTP API
so igvi_host can relay UI commands without any ROS dependency.

HTTP API (port configurable, default 8772):
  POST /start           body: {"task": "bridge"|"bridge_grab"|"door"|"find_grab"}
  POST /pause
  POST /resume
  POST /interrupt
  GET  /status          returns current TaskStatus JSON

Waypoints are declared as ROS parameters (see _declare_waypoint_params).
Override them via a YAML params file mounted into the container.
"""
from __future__ import annotations

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from igvi_task_core.actions.detect import Detector
from igvi_task_core.actions.grab import Grabber
from igvi_task_core.actions.navigate import Navigator
from igvi_task_core.fsm import TaskFSM, TaskState
from igvi_task_core.tasks.task1_bridge import Task1Bridge
from igvi_task_core.tasks.task2_bridge_grab import Task2BridgeGrab
from igvi_task_core.tasks.task3_door import Task3Door
from igvi_task_core.tasks.task4_find_grab import Task4FindGrab

_TASK_MAP = {
    "bridge":       Task1Bridge,
    "bridge_grab":  Task2BridgeGrab,
    "door":         Task3Door,
    "find_grab":    Task4FindGrab,
}

_MAX_LOG = 300


class TaskManagerNode(Node):
    def __init__(self) -> None:
        super().__init__("igvi_task_core")
        self._lock = threading.Lock()

        # ── ROS interfaces ────────────────────────────────────────────────────
        self._status_pub = self.create_publisher(String, "/task/status", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_timer(0.2, self._publish_status)

        # ── Action / subscription wrappers ────────────────────────────────────
        self.navigator = Navigator(self)
        self.grabber = Grabber(self)
        self.detector = Detector(self)

        # ── Waypoint parameters ───────────────────────────────────────────────
        _declare_waypoint_params(self)

        # ── Task state ────────────────────────────────────────────────────────
        self._fsm: Optional[TaskFSM] = None
        self._active_task_id: str = ""
        self._task_thread: Optional[threading.Thread] = None
        self._log: list[str] = []

        # ── HTTP bridge ───────────────────────────────────────────────────────
        port = int(self.declare_parameter("http_port", 8772).value)
        self._http_server = _start_http_server(self, port)
        self.get_logger().info(f"TaskManagerNode ready  HTTP:{port}")

    # ── Waypoint accessors (used by tasks) ───────────────────────────────────

    def wp(self, name: str) -> tuple[float, float, float]:
        """Return (x, y, yaw) for a named single waypoint parameter."""
        x = self.get_parameter(f"wp.{name}.x").value
        y = self.get_parameter(f"wp.{name}.y").value
        yaw = self.get_parameter(f"wp.{name}.yaw").value
        return float(x), float(y), float(yaw)

    def wps(self, name: str) -> list[tuple[float, float, float]]:
        """Return list of (x, y, yaw) for a multi-waypoint parameter.

        Stored as wp.<name>.count + wp.<name>.<i>.x/y/yaw.
        """
        count = int(self.get_parameter(f"wp.{name}.count").value)
        result = []
        for i in range(count):
            x = self.get_parameter(f"wp.{name}.{i}.x").value
            y = self.get_parameter(f"wp.{name}.{i}.y").value
            yaw = self.get_parameter(f"wp.{name}.{i}.yaw").value
            result.append((float(x), float(y), float(yaw)))
        return result

    # ── Log helper (passed to tasks as self.log) ──────────────────────────────

    def task_log(self, msg: str) -> None:
        self.get_logger().info(msg)
        with self._lock:
            self._log.append(msg)
            if len(self._log) > _MAX_LOG:
                self._log = self._log[-_MAX_LOG:]

    # ── HTTP command handlers (called from HTTP thread) ───────────────────────

    def cmd_start(self, task_id: str) -> tuple[bool, str]:
        TaskClass = _TASK_MAP.get(task_id)
        if TaskClass is None:
            return False, f"unknown task: {task_id!r}"

        with self._lock:
            if self._fsm and self._fsm.state in (TaskState.RUNNING, TaskState.PAUSED):
                return False, "task already active"
            self._log.clear()
            self._active_task_id = task_id
            self._fsm = TaskFSM(self._on_transition)

        ok = self._fsm.start()
        if not ok:
            return False, "FSM start failed"

        task = TaskClass(self)
        self._task_thread = threading.Thread(
            target=self._run_task, args=(task, self._fsm), daemon=True
        )
        self._task_thread.start()
        return True, f"started {task_id}"

    def cmd_pause(self) -> tuple[bool, str]:
        with self._lock:
            fsm = self._fsm
        if fsm and fsm.pause():
            return True, "paused"
        return False, "cannot pause"

    def cmd_resume(self) -> tuple[bool, str]:
        with self._lock:
            fsm = self._fsm
        if fsm and fsm.resume():
            return True, "resumed"
        return False, "cannot resume"

    def cmd_interrupt(self) -> tuple[bool, str]:
        with self._lock:
            fsm = self._fsm
        if fsm and fsm.interrupt():
            self.navigator.cancel()
            self.grabber.cancel()
            return True, "interrupted"
        return False, "nothing to interrupt"

    def get_status(self) -> dict[str, Any]:
        with self._lock:
            fsm = self._fsm
            task_id = self._active_task_id
            log = list(self._log)
        if fsm is None:
            return {"ok": True, "task_id": task_id, "state": "idle",
                    "step_index": 0, "step_name": "", "log": log}
        return {
            "ok": True,
            "task_id": task_id,
            "state": fsm.state.value,
            "step_index": fsm.step_index,
            "step_name": fsm.step_name,
            "log": log,
        }

    # ── Internal ──────────────────────────────────────────────────────────────

    def _on_transition(self, state: TaskState, step: int, name: str) -> None:
        self.task_log(f"[fsm:{state.value}] step={step} {name!r}")

    def _run_task(self, task, fsm: TaskFSM) -> None:
        try:
            task.run(fsm)
            if fsm.state == TaskState.RUNNING:
                fsm.mark_completed()
                self.task_log("[task] completed")
        except InterruptedError:
            fsm.mark_interrupted()
            self.task_log("[task] interrupted")
        except Exception as exc:
            self.task_log(f"[task] failed: {exc}")
            fsm.mark_failed()

    def _publish_status(self) -> None:
        msg = String()
        msg.data = json.dumps(self.get_status())
        self._status_pub.publish(msg)


# ── Waypoint parameter declarations ──────────────────────────────────────────

def _declare_waypoint_params(node: TaskManagerNode) -> None:
    """Declare all waypoint parameters with sensible defaults.

    Override in a params YAML mounted at container start:
      ros2 run igvi_task_core task_manager_node --ros-args --params-file /configs/task_waypoints.yaml
    """
    # Single waypoints: target_bridge, home_bridge, target_door
    for name in ("target_bridge", "home_bridge", "target_door"):
        node.declare_parameter(f"wp.{name}.x", 0.0)
        node.declare_parameter(f"wp.{name}.y", 0.0)
        node.declare_parameter(f"wp.{name}.yaw", 0.0)

    # Multi-waypoint: target_find_grab (up to 8 points)
    node.declare_parameter("wp.target_find_grab.count", 1)
    for i in range(8):
        node.declare_parameter(f"wp.target_find_grab.{i}.x", 0.0)
        node.declare_parameter(f"wp.target_find_grab.{i}.y", 0.0)
        node.declare_parameter(f"wp.target_find_grab.{i}.yaw", 0.0)

    node.declare_parameter("http_port", 8772)


# ── HTTP bridge ───────────────────────────────────────────────────────────────

def _start_http_server(node: TaskManagerNode, port: int) -> ThreadingHTTPServer:
    handler = _make_handler(node)
    server = ThreadingHTTPServer(("0.0.0.0", port), handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server


def _make_handler(node: TaskManagerNode):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass  # suppress per-request stdout noise

        def _send_json(self, status: int, payload: dict) -> None:
            body = json.dumps(payload).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _read_json(self) -> dict:
            length = int(self.headers.get("Content-Length", 0))
            return json.loads(self.rfile.read(length)) if length else {}

        def do_GET(self):  # noqa: N802
            if self.path == "/status":
                self._send_json(200, node.get_status())
            elif self.path == "/health":
                self._send_json(200, {"ok": True})
            else:
                self._send_json(404, {"ok": False, "message": "not found"})

        def do_POST(self):  # noqa: N802
            path = self.path.rstrip("/")
            if path == "/start":
                body = self._read_json()
                task_id = body.get("task", "")
                ok, msg = node.cmd_start(task_id)
                self._send_json(200 if ok else 400, {"ok": ok, "message": msg})
            elif path == "/pause":
                ok, msg = node.cmd_pause()
                self._send_json(200 if ok else 400, {"ok": ok, "message": msg})
            elif path == "/resume":
                ok, msg = node.cmd_resume()
                self._send_json(200 if ok else 400, {"ok": ok, "message": msg})
            elif path == "/interrupt":
                ok, msg = node.cmd_interrupt()
                self._send_json(200 if ok else 400, {"ok": ok, "message": msg})
            else:
                self._send_json(404, {"ok": False, "message": "not found"})

    return Handler


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    rclpy.init()
    node = TaskManagerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node._http_server.shutdown()
        node.destroy_node()
        rclpy.shutdown()
