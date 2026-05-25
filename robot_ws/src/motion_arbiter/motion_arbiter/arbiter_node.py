"""Motion arbiter: unified controller for path tracking and manual override.

Subscribes:
  /plan             nav_msgs/Path        — global plan from nav2 planner (empty = clear)
  /motion/cmd       geometry_msgs/Twist  — manual override command
  /motion/clear_path std_msgs/Empty      — explicit "drop the stored path" signal
  /estop            std_msgs/Bool        — latched emergency stop

Pose is read from TF (path frame → base_link).

Publishes:
  /cmd_vel              geometry_msgs/TwistStamped — final wheel velocity command
  /motion/state         std_msgs/String            — current arbiter state

State machine:
  IDLE          → no path, no manual cmd
  PATH_TRACKING → following stored path via pure pursuit
  ALIGNING      → at goal position, correcting final heading / longitudinal drift
  OVERRIDE      → path retained, manual cmd active (suspends tracking)
  MANUAL        → no path, manual cmd active
  ESTOP         → emergency stop engaged; output hard-zeroed

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
import tf2_ros
from tf2_ros import TransformException
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, String

# Latched QoS matching the bridge's /estop publisher so we receive the current
# state immediately on subscribe, even if the arbiter starts after the bridge.
_ESTOP_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class State(Enum):
    IDLE = "idle"
    PATH_TRACKING = "path_tracking"
    ALIGNING = "aligning"
    OVERRIDE = "override"
    MANUAL = "manual"
    ESTOP = "estop"


class MotionArbiter(Node):
    def __init__(self) -> None:
        super().__init__("motion_arbiter")
        # ── Tunable parameters ────────────────────────────────────────────
        self.declare_parameter("control_rate", 20.0)
        self.declare_parameter("override_timeout", 0.6)
        self.declare_parameter("lookahead_distance", 0.35)
        import os
        env_goal_tol = os.environ.get("RETRIEVE_ARRIVAL_TOLERANCE")
        default_goal_tol = float(env_goal_tol) if env_goal_tol else 0.1

        env_yaw_tol = os.environ.get("RETRIEVE_YAW_TOLERANCE")
        default_yaw_tol = float(env_yaw_tol) if env_yaw_tol else 0.1

        self.declare_parameter("goal_tolerance", default_goal_tol)
        self.declare_parameter("yaw_tolerance", default_yaw_tol)
        self.declare_parameter("kp_linear_align", 0.8)
        self.declare_parameter("max_linear_velocity", 0.3)
        self.declare_parameter("max_angular_velocity", 0.5)
        self.declare_parameter("accel_linear", 0.6)
        self.declare_parameter("accel_angular", 2.0)
        self.declare_parameter("kp_angular", 1.4)
        # self.declare_parameter("kp_wz_feedback", 0.15)
        self.declare_parameter("slow_heading_threshold", math.pi / 4)
        self.declare_parameter("slow_linear_velocity", 0.0)
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("enable_drift_correction", True)

        self._control_rate = float(self.get_parameter("control_rate").value)
        self._override_timeout = float(self.get_parameter("override_timeout").value)

        # ── State ─────────────────────────────────────────────────────────
        self._lock = threading.Lock()
        self._state: State = State.IDLE
        self._estop: bool = False
        self._path: list[tuple[float, float, float]] = []
        self._path_index: int = 0
        self._aligning_correcting_drift: bool = False
        self._rot_start_pose: tuple[float, float, float] | None = None
        self._drift_rate_x: float = 0.0
        self._drift_rate_y: float = 0.0
        self._motion_target: tuple[float, float] = (0.0, 0.0)
        self._last_motion_time = None
        self._path_frame_id: str = "map"
        self._current_vel: tuple[float, float] = (0.0, 0.0)
        # self._current_wz_measured: float = 0.0

        # ── ROS I/O ───────────────────────────────────────────────────────
        self.create_subscription(Path, "/plan", self._on_path, 10)
        self.create_subscription(Twist, "/motion/cmd", self._on_motion_cmd, 10)
        self.create_subscription(Empty, "/motion/clear_path", self._on_clear_path, 10)
        self.create_subscription(Bool, "/estop", self._on_estop, _ESTOP_QOS)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        output_topic = str(self.get_parameter("output_topic").value)
        self._cmd_pub = self.create_publisher(TwistStamped, output_topic, 10)
        self._state_pub = self.create_publisher(String, "/motion/state", 10)

        self.create_timer(1.0 / self._control_rate, self._tick)
        self.get_logger().info(
            f"motion_arbiter started; control@{self._control_rate:.1f}Hz output={output_topic}"
        )

    # ── ROS callbacks ─────────────────────────────────────────────────────

    def _on_path(self, msg: Path) -> None:
        # Empty Path = explicit "abort tracking" from a task server (e.g. the
        # retrieve servers clear the path before visual servoing so the stale
        # nav2 plan doesn't pull the robot back during in-place centering /
        # approach). Nav2's planner_server only publishes on plan success, so
        # empty messages here are intentional clears, not planning failures.
        if not msg.poses:
            with self._lock:
                if not self._path:
                    return
                self._path = []
                self._path_index = 0
                self._aligning_correcting_drift = False
                self._rot_start_pose = None
                self._drift_rate_x = 0.0
                self._drift_rate_y = 0.0
                if self._estop:
                    self._state = State.ESTOP
                elif self._state in (State.OVERRIDE, State.MANUAL):
                    pass  # let the manual command finish; nothing to revert to
                else:
                    self._state = State.IDLE
            self.get_logger().info("Path cleared")
            return
        with self._lock:
            self._path_frame_id = msg.header.frame_id
            self._path = [
                _yaw_from_pose(p.pose.position.x, p.pose.position.y, p.pose.orientation)
                for p in msg.poses
            ]
            self._path_index = 0
            self._aligning_correcting_drift = False
            self._rot_start_pose = None
            self._drift_rate_x = 0.0
            self._drift_rate_y = 0.0
            if self._estop:
                self._state = State.ESTOP
                self.get_logger().info(
                    f"Path received ({len(self._path)} pts) — held, E-STOP engaged"
                )
            elif self._state == State.OVERRIDE:
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
            if self._estop:
                self._state = State.ESTOP
            elif is_zero:
                self._state = State.PATH_TRACKING if self._path else State.IDLE
            else:
                self._state = State.OVERRIDE if self._path else State.MANUAL

    def _on_clear_path(self, _msg: Empty) -> None:
        # External "abort tracking" signal on a dedicated topic (e.g. open_door
        # taking control, or a canceled door mission). Equivalent to receiving
        # an empty Path on /plan, but lets a node that doesn't publish /plan
        # (the bridge) drop a stale path. Without this, a stored plan resumes
        # via PATH_TRACKING once override_timeout elapses between commands.
        with self._lock:
            had_path = bool(self._path)
            self._path = []
            self._path_index = 0
            if self._estop:
                self._state = State.ESTOP
            elif self._state in (State.PATH_TRACKING, State.ALIGNING, State.OVERRIDE):
                self._state = (
                    State.MANUAL if self._state == State.OVERRIDE else State.IDLE
                )
        if had_path:
            self.get_logger().info("Path cleared via /motion/clear_path")

    def _on_estop(self, msg: Bool) -> None:
        engaged = bool(msg.data)
        with self._lock:
            self._estop = engaged
            if engaged:
                self._state = State.ESTOP
                self._motion_target = (0.0, 0.0)
            else:
                self._state = State.PATH_TRACKING if self._path else State.IDLE
        self.get_logger().warn("E-STOP ENGAGED" if engaged else "E-STOP RELEASED")

    # ── Control loop ──────────────────────────────────────────────────────

    def _tick(self) -> None:
        now = self.get_clock().now()
        with self._lock:
            estop = self._estop
            state = self._state
            path = list(self._path)
            path_index = self._path_index
            path_frame_id = self._path_frame_id
            motion_target = self._motion_target
            last_motion = self._last_motion_time
            # wz_measured = self._current_wz_measured

        # E-stop: hard-zero the output (no ramp) and publish until released.
        if estop:
            self._current_vel = (0.0, 0.0)
            cmd = TwistStamped()
            cmd.header.stamp = now.to_msg()
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0
            self._cmd_pub.publish(cmd)
            state_msg = String()
            state_msg.data = State.ESTOP.value
            self._state_pub.publish(state_msg)
            return

        # Override timeout: clear stale manual cmd
        if state in (State.OVERRIDE, State.MANUAL) and last_motion is not None:
            elapsed = (now - last_motion).nanoseconds * 1e-9
            if elapsed > self._override_timeout:
                with self._lock:
                    self._motion_target = (0.0, 0.0)
                    self._state = State.PATH_TRACKING if self._path else State.IDLE
                    state = self._state

        # Compute desired velocity for this tick
        if state in (State.PATH_TRACKING, State.ALIGNING) and path:
            pose = None
            try:
                t = self._tf_buffer.lookup_transform(
                    path_frame_id, "base_link", rclpy.time.Time()
                )
                pose = _yaw_from_pose(
                    t.transform.translation.x,
                    t.transform.translation.y,
                    t.transform.rotation,
                )
            except TransformException as ex:
                self.get_logger().warn(f"Could not get transform to base_link: {ex}")

            if pose is None:
                desired_vx, desired_wz = 0.0, 0.0
            else:
                result = self._pure_pursuit(path, path_index, pose, state)
                if result is None:
                    with self._lock:
                        self._path = []
                        self._path_index = 0
                        self._state = State.IDLE
                    state = State.IDLE
                    desired_vx, desired_wz = 0.0, 0.0
                    self.get_logger().info("Path tracking complete")
                else:
                    desired_vx, desired_wz, new_idx, new_state = result
                    with self._lock:
                        self._path_index = new_idx
                        self._state = new_state
                    state = new_state
        elif state in (State.OVERRIDE, State.MANUAL):
            desired_vx, desired_wz = motion_target
        else:
            desired_vx, desired_wz = 0.0, 0.0

        # Apply IMU-based closed-loop feedback for yaw rate
        # if state in (State.PATH_TRACKING, State.OVERRIDE, State.MANUAL):
        #     kp_feedback = float(self.get_parameter("kp_wz_feedback").value)
        #     wz_error = desired_wz - wz_measured
        #     desired_wz = desired_wz + (kp_feedback * wz_error)

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
        path: list[tuple[float, float, float]],
        path_index: int,
        pose: tuple[float, float, float],
        current_state: State,
    ) -> tuple[float, float, int, State] | None:
        """Returns (vx, wz, new_path_index, new_state) or None when path is complete."""
        if not path:
            return None
        x, y, yaw = pose
        gx, gy, gyaw = path[-1]
        goal_tol = float(self.get_parameter("goal_tolerance").value)
        dist_to_goal = math.hypot(gx - x, gy - y)

        is_at_goal = dist_to_goal < goal_tol
        if current_state == State.ALIGNING and dist_to_goal < goal_tol * 2.5:
            is_at_goal = True

        if is_at_goal:
            yaw_tol = float(self.get_parameter("yaw_tolerance").value)
            heading_error = _wrap_angle(gyaw - yaw)
            if abs(heading_error) < yaw_tol:
                # Once aligned in yaw, we only exit if we are within the strict goal tolerance
                if dist_to_goal < goal_tol:
                    self._aligning_correcting_drift = False
                    self._rot_start_pose = None
                    return None
                else:
                    # If we finished rotation but drifted slightly outside tolerance,
                    # we let it fall through to the pure-pursuit path tracker to drive straight back!
                    self._aligning_correcting_drift = False
                    self._rot_start_pose = None
            else:
                # Online drift prediction & compensation:
                # We calculate relative position to the goal:
                xr = x - gx
                yr = y - gy
                
                enable_drift_correction = bool(self.get_parameter("enable_drift_correction").value)
                
                # Update start pose of rotation segment when starting a new rotation phase
                if not self._aligning_correcting_drift:
                    if self._rot_start_pose is None:
                        self._rot_start_pose = (x, y, yaw)
                
                # Calculate Golden Predictive Correction Distance (s) if correction is enabled
                if enable_drift_correction:
                    predicted_dx = self._drift_rate_x * heading_error
                    predicted_dy = self._drift_rate_y * heading_error
                    
                    # Avoid singularity when remaining angle is small (below 20 deg / 0.35 rad)
                    if abs(heading_error) < 0.35:
                        s = -(xr * math.cos(yaw) + yr * math.sin(yaw))
                    else:
                        num = (xr + predicted_dx) * math.sin(yaw + heading_error) - (yr + predicted_dy) * math.cos(yaw + heading_error)
                        den = -math.sin(heading_error)
                        s = num / den

                    # Clamp the final compensation distance to a very safe maximum (max 4.5cm)
                    # This physically prevents any large sudden forward/backward movements.
                    s = max(-0.045, min(0.045, s))
                else:
                    # If drift correction is disabled (False), set s = 0.0 to completely bypass
                    # any forward/backward correction phase, performing only pure in-place rotation!
                    s = 0.0

                # Hysteresis Thresholds on predictive drift s
                DRIFT_START_THRESHOLD = 0.035  # 3.5cm predictive drift
                DRIFT_STOP_THRESHOLD = 0.012   # 1.2cm target accuracy
                
                if self._aligning_correcting_drift:
                    if abs(s) < DRIFT_STOP_THRESHOLD:
                        self._aligning_correcting_drift = False
                        # Ready to record starting pose when rotation resumes
                        self._rot_start_pose = None
                else:
                    if abs(s) > DRIFT_START_THRESHOLD:
                        self._aligning_correcting_drift = True
                        
                        # Transitioning from rotation to correction!
                        # Since the robot has stopped rotating, the EKF / SLAM has just settled,
                        # and we have completed a full rotation segment.
                        # Calculate the drift rate of this completed segment.
                        if enable_drift_correction and self._rot_start_pose is not None:
                            x_start, y_start, yaw_start = self._rot_start_pose
                            dyaw = _wrap_angle(yaw - yaw_start)
                            if abs(dyaw) > 0.15:
                                raw_drx = (x - x_start) / dyaw
                                raw_dry = (y - y_start) / dyaw
                                
                                # Clamp raw rates to prevent spikes and apply Exponential Moving Average (EMA) for smoothing
                                clamped_drx = max(-0.06, min(0.06, raw_drx))
                                clamped_dry = max(-0.06, min(0.06, raw_dry))
                                self._drift_rate_x = 0.7 * self._drift_rate_x + 0.3 * clamped_drx
                                self._drift_rate_y = 0.7 * self._drift_rate_y + 0.3 * clamped_dry
                        
                        # Reset for next rotation segment
                        self._rot_start_pose = None
                
                if self._aligning_correcting_drift:
                    # Correction phase: STOP rotation, drive straight forward/backward
                    wz = 0.0
                    kp_linear_align = float(self.get_parameter("kp_linear_align").value)
                    # Correct using the predictive offset s
                    vx = kp_linear_align * s
                    
                    # Clamp linear correction speed to a very safe maximum (e.g. max 0.04 m/s)
                    MAX_ALIGN_VX = 0.04
                    if vx > MAX_ALIGN_VX:
                        vx = MAX_ALIGN_VX
                    elif vx < -MAX_ALIGN_VX:
                        vx = -MAX_ALIGN_VX
                else:
                    # Rotation phase: rotate in place cleanly, keep linear velocity at 0.0
                    wz = float(self.get_parameter("kp_angular").value) * heading_error
                    vx = 0.0
                    
                    # Clamp angular velocity during alignment to be extremely smooth and stable (max 0.35 rad/s)
                    # This prevents high-speed tire slip and SLAM tracking delays.
                    MAX_ALIGN_WZ = 0.35
                    if wz > MAX_ALIGN_WZ:
                        wz = MAX_ALIGN_WZ
                    elif wz < -MAX_ALIGN_WZ:
                        wz = -MAX_ALIGN_WZ
                
                return vx, wz, path_index, State.ALIGNING

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

        tx, ty, _ = path[target_idx]
        target_heading = math.atan2(ty - y, tx - x)
        heading_error = _wrap_angle(target_heading - yaw)

        slow_thr = float(self.get_parameter("slow_heading_threshold").value)
        max_vx = float(self.get_parameter("max_linear_velocity").value)
        slow_vx = float(self.get_parameter("slow_linear_velocity").value)
        
        # 1. Linear deceleration based on heading error
        if slow_thr > 0.0:
            ratio = max(0.0, min(1.0, 1.0 - (abs(heading_error) / slow_thr)))
            vx = slow_vx + ratio * (max_vx - slow_vx)
        else:
            vx = max_vx
            
        # 2. Predictive deceleration based on physical acceleration limits to prevent overshoot
        accel_lin = float(self.get_parameter("accel_linear").value)
        # We ensure a minimum speed floor of 0.05 m/s so the robot never stalls due to friction before crossing the goal tolerance
        MIN_DRIVE_VELOCITY = 0.05
        max_allowed_vx = max(MIN_DRIVE_VELOCITY, math.sqrt(2.0 * accel_lin * max(0.0, dist_to_goal)))
        vx = min(vx, max_allowed_vx)

        wz = float(self.get_parameter("kp_angular").value) * heading_error
        return vx, wz, closest_idx, State.PATH_TRACKING


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
