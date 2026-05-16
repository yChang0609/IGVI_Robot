"""Motion arbiter: unified controller for path tracking and manual override.

Subscribes:
  /plan        nav_msgs/Path                       — global plan from nav2 planner
  /motion/cmd  geometry_msgs/Twist                 — manual override command
  /amcl_pose   geometry_msgs/PoseWithCovarianceStamped  — map-frame pose (preferred)
  /odom        nav_msgs/Odometry                   — fallback pose + velocity feedback

Publishes:
  /cmd_vel              geometry_msgs/TwistStamped — final wheel velocity command
  /motion/state         std_msgs/String            — current arbiter state

State machine:
  IDLE          → no path, no manual cmd
  PATH_TRACKING → following stored path via pure pursuit
  OVERRIDE      → path retained, manual cmd active (suspends tracking)
  MANUAL        → no path, manual cmd active

A non-zero Twist on /motion/cmd while PATH_TRACKING → OVERRIDE.
After `override_timeout` seconds without a new cmd (or a zero Twist), OVERRIDE → PATH_TRACKING.
Path complete (within goal_tolerance of last point) → IDLE.

The output velocity is rate-limited (a P-loop toward the desired target,
clamped by `accel_linear` / `accel_angular`) and clamped to max limits — this
is the "simple PID" requested for command execution.
"""

from __future__ import annotations

import math
import threading
from enum import Enum

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import String


class State(Enum):
    IDLE = "idle"
    PATH_TRACKING = "path_tracking"
    OVERRIDE = "override"
    MANUAL = "manual"


class MotionArbiter(Node):
    def __init__(self) -> None:
        super().__init__("motion_arbiter")
        # ── Tunable parameters ────────────────────────────────────────────
        self.declare_parameter("control_rate", 20.0)
        self.declare_parameter("override_timeout", 0.6)
        self.declare_parameter("lookahead_distance", 0.35)
        self.declare_parameter("goal_tolerance", 0.18)
        self.declare_parameter("max_linear_velocity", 0.3)
        self.declare_parameter("max_angular_velocity", 1.2)
        self.declare_parameter("accel_linear", 0.6)
        self.declare_parameter("accel_angular", 1.8)
        self.declare_parameter("kp_angular", 1.4)
        self.declare_parameter("slow_heading_threshold", math.pi / 4)
        self.declare_parameter("slow_linear_velocity", 0.08)
        self.declare_parameter("output_topic", "/cmd_vel")

        self._control_rate = float(self.get_parameter("control_rate").value)
        self._override_timeout = float(self.get_parameter("override_timeout").value)

        # ── State ─────────────────────────────────────────────────────────
        self._lock = threading.Lock()
        self._state: State = State.IDLE
        self._path: list[tuple[float, float]] = []
        self._path_index: int = 0
        self._motion_target: tuple[float, float] = (0.0, 0.0)
        self._last_motion_time = None
        self._pose: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._pose_source: str = "none"
        self._current_vel: tuple[float, float] = (0.0, 0.0)

        # ── ROS I/O ───────────────────────────────────────────────────────
        self.create_subscription(Path, "/plan", self._on_path, 10)
        self.create_subscription(Twist, "/motion/cmd", self._on_motion_cmd, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl, 10)
        self.create_subscription(Odometry, "/odometry/filtered", self._on_odom, 10)

        output_topic = str(self.get_parameter("output_topic").value)
        self._cmd_pub = self.create_publisher(TwistStamped, output_topic, 10)
        self._state_pub = self.create_publisher(String, "/motion/state", 10)

        self.create_timer(1.0 / self._control_rate, self._tick)
        self.get_logger().info(
            f"motion_arbiter started; control@{self._control_rate:.1f}Hz output={output_topic}"
        )

    # ── ROS callbacks ─────────────────────────────────────────────────────

    def _on_path(self, msg: Path) -> None:
        if not msg.poses:
            return
        with self._lock:
            self._path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            self._path_index = 0
            if self._state == State.OVERRIDE:
                self.get_logger().info(
                    f"Path updated ({len(self._path)} pts) — held until override clears"
                )
            else:
                self._state = State.PATH_TRACKING
                self.get_logger().info(f"Path received ({len(self._path)} pts); tracking")

    def _on_motion_cmd(self, msg: Twist) -> None:
        with self._lock:
            self._motion_target = (float(msg.linear.x), float(msg.angular.z))
            self._last_motion_time = self.get_clock().now()
            is_zero = abs(msg.linear.x) < 1e-3 and abs(msg.angular.z) < 1e-3
            if is_zero:
                self._state = State.PATH_TRACKING if self._path else State.IDLE
            else:
                self._state = State.OVERRIDE if self._path else State.MANUAL

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose
        with self._lock:
            self._pose = _yaw_from_pose(p.position.x, p.position.y, p.orientation)
            self._pose_source = "amcl"

    def _on_odom(self, msg: Odometry) -> None:
        # Only trust /odom if amcl hasn't given us a map-frame pose yet.
        with self._lock:
            if self._pose_source == "amcl":
                return
            p = msg.pose.pose
            self._pose = _yaw_from_pose(p.position.x, p.position.y, p.orientation)
            self._pose_source = "odom"

    # ── Control loop ──────────────────────────────────────────────────────

    def _tick(self) -> None:
        now = self.get_clock().now()
        with self._lock:
            state = self._state
            path = list(self._path)
            path_index = self._path_index
            motion_target = self._motion_target
            last_motion = self._last_motion_time
            pose = self._pose

        # Override timeout: clear stale manual cmd
        if state in (State.OVERRIDE, State.MANUAL) and last_motion is not None:
            elapsed = (now - last_motion).nanoseconds * 1e-9
            if elapsed > self._override_timeout:
                with self._lock:
                    self._motion_target = (0.0, 0.0)
                    self._state = State.PATH_TRACKING if self._path else State.IDLE
                    state = self._state

        # Compute desired velocity for this tick
        if state == State.PATH_TRACKING and path:
            result = self._pure_pursuit(path, path_index, pose)
            if result is None:
                with self._lock:
                    self._path = []
                    self._path_index = 0
                    self._state = State.IDLE
                state = State.IDLE
                desired_vx, desired_wz = 0.0, 0.0
                self.get_logger().info("Path tracking complete")
            else:
                desired_vx, desired_wz, new_idx = result
                with self._lock:
                    self._path_index = new_idx
        elif state in (State.OVERRIDE, State.MANUAL):
            desired_vx, desired_wz = motion_target
        else:
            desired_vx, desired_wz = 0.0, 0.0

        # Velocity smoother (acceleration-limited P toward desired)
        max_lin = float(self.get_parameter("max_linear_velocity").value)
        max_ang = float(self.get_parameter("max_angular_velocity").value)
        accel_lin = float(self.get_parameter("accel_linear").value)
        accel_ang = float(self.get_parameter("accel_angular").value)
        dt = 1.0 / self._control_rate

        desired_vx = max(-max_lin, min(max_lin, desired_vx))
        desired_wz = max(-max_ang, min(max_ang, desired_wz))

        vx, wz = self._current_vel
        dv = max(-accel_lin * dt, min(accel_lin * dt, desired_vx - vx))
        dw = max(-accel_ang * dt, min(accel_ang * dt, desired_wz - wz))
        vx += dv
        wz += dw
        self._current_vel = (vx, wz)

        cmd = TwistStamped()
        cmd.header.stamp = now.to_msg()
        cmd.twist.linear.x = vx
        cmd.twist.angular.z = wz
        self._cmd_pub.publish(cmd)

        state_msg = String()
        state_msg.data = state.value
        self._state_pub.publish(state_msg)

    # ── Pure Pursuit ──────────────────────────────────────────────────────

    def _pure_pursuit(
        self,
        path: list[tuple[float, float]],
        path_index: int,
        pose: tuple[float, float, float],
    ) -> tuple[float, float, int] | None:
        """Returns (vx, wz, new_path_index) or None when path is complete."""
        if not path:
            return None
        x, y, yaw = pose

        gx, gy = path[-1]
        goal_tol = float(self.get_parameter("goal_tolerance").value)
        if math.hypot(gx - x, gy - y) < goal_tol:
            return None

        # Advance closest-point index (no rewinding)
        closest_idx = path_index
        closest_dist = float("inf")
        for i in range(path_index, len(path)):
            d = math.hypot(path[i][0] - x, path[i][1] - y)
            if d < closest_dist:
                closest_dist = d
                closest_idx = i

        # Look ahead from the closest segment
        lookahead = float(self.get_parameter("lookahead_distance").value)
        target_idx = len(path) - 1
        for i in range(closest_idx, len(path)):
            if math.hypot(path[i][0] - x, path[i][1] - y) >= lookahead:
                target_idx = i
                break

        tx, ty = path[target_idx]
        target_heading = math.atan2(ty - y, tx - x)
        heading_error = _wrap_angle(target_heading - yaw)

        slow_thr = float(self.get_parameter("slow_heading_threshold").value)
        if abs(heading_error) > slow_thr:
            vx = float(self.get_parameter("slow_linear_velocity").value)
        else:
            vx = float(self.get_parameter("max_linear_velocity").value)
        wz = float(self.get_parameter("kp_angular").value) * heading_error
        return vx, wz, closest_idx


# ── Helpers ──────────────────────────────────────────────────────────────

def _yaw_from_pose(x: float, y: float, q) -> tuple[float, float, float]:
    yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )
    return float(x), float(y), float(yaw)


def _wrap_angle(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def main() -> None:
    rclpy.init()
    node = MotionArbiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
