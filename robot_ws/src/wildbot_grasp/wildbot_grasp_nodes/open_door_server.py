"""Door-opening FSM as a wildbot_grasp action server.

Consumes:
  /detections_json      std_msgs/String  (JSON from eto_eye; per detection:
                                          class_name, score, bbox.center_x/y/size_x/y,
                                          depth_m, depth_valid)
Publishes:
  /motion/cmd                       geometry_msgs/Twist
  /arm_safeguard/target_trajectory  trajectory_msgs/JointTrajectory  (via ArmCommander)

FSM:
  ALIGN     rotate-in-place until |x_norm| < align_pixel_tol for N ticks
  APPROACH  drive forward with mild centering until depth_m <= ready_distance_m
  PRESS     arm: above-handle → press-down (timed via ArmCommander), then hold
  PUSH      arm: push-forward (non-blocking) + base drives forward for push_duration_sec
  COMPLETE  stop base, retract to door_home_pose_deg, succeed
  ABORT     stop base, retract to door_home_pose_deg, abort

Base commands are published to /motion/cmd (Twist); motion_arbiter handles the
acceleration limiter and the /cmd_vel publish. The arbiter's override_timeout
is 0.6 s, so the FSM publishes every tick (10 Hz default).
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from enum import Enum
from threading import Lock
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String

from wildbot_grasp.action import OpenDoor

from .motion import ArmCommander


class State(Enum):
    IDLE = "idle"
    ALIGN = "align"
    APPROACH = "approach"
    PRESS = "press"
    PUSH = "push"
    COMPLETE = "complete"
    ABORT = "abort"


@dataclass
class Detection:
    stamp_monotonic: float
    x_norm: float            # [-1, 1] horizontal offset of bbox center from image center
    y_norm: float            # [-1, 1] vertical offset
    depth_m: float           # 0.0 when depth_valid is False
    depth_valid: bool


class OpenDoorServer(Node):
    def __init__(self):
        super().__init__("open_door_server")

        # ── Detection input ──────────────────────────────────────────────
        self.declare_parameter("detection_topic", "/detections_json")
        self.declare_parameter("knob_class", "men_ba")
        self.declare_parameter("image_width", 1280)
        self.declare_parameter("image_height", 720)
        self.declare_parameter("detection_timeout_sec", 0.5)
        self.declare_parameter("lost_grace_sec", 1.5)

        # ── Control loop ─────────────────────────────────────────────────
        self.declare_parameter("control_rate_hz", 10.0)
        self.declare_parameter("twist_topic", "/motion/cmd")

        # ── ALIGN ────────────────────────────────────────────────────────
        self.declare_parameter("align_pixel_tol", 0.05)
        self.declare_parameter("align_kp", 0.9)
        self.declare_parameter("align_max_wz", 0.5)
        self.declare_parameter("align_stable_ticks", 5)

        # ── APPROACH ─────────────────────────────────────────────────────
        self.declare_parameter("ready_distance_m", 0.45)
        self.declare_parameter("approach_speed", 0.10)
        self.declare_parameter("approach_center_kp", 0.4)
        self.declare_parameter("approach_max_wz", 0.25)

        # ── PRESS / PUSH ─────────────────────────────────────────────────
        self.declare_parameter("press_hold_sec", 0.6)
        self.declare_parameter("push_duration_sec", 3.0)
        self.declare_parameter("push_speed", 0.08)

        # ── Arm poses (degrees) — tune to the physical handle geometry ───
        # 3 DOF: [arm_1_joint, arm_2_joint, gripper_joint].
        self.declare_parameter("door_above_pose_deg", [167.0, 80.0, 170.6])
        self.declare_parameter("door_press_pose_deg", [167.0, 50.0, 170.6])
        self.declare_parameter("door_push_pose_deg", [140.0, 80.0, 170.6])
        self.declare_parameter("door_home_pose_deg", [167.0, 75.0, 170.6])

        self._cb_group = ReentrantCallbackGroup()
        self._lock = Lock()
        self._latest: Optional[Detection] = None

        self.arm = ArmCommander(self)
        self._twist_pub = self.create_publisher(
            Twist, str(self.get_parameter("twist_topic").value), 10
        )
        self.create_subscription(
            String,
            str(self.get_parameter("detection_topic").value),
            self._on_detections,
            10,
            callback_group=self._cb_group,
        )

        self._action_server = ActionServer(
            self,
            OpenDoor,
            "open_door",
            execute_callback=self._execute,
            callback_group=self._cb_group,
            goal_callback=lambda _r: GoalResponse.ACCEPT,
            cancel_callback=lambda _h: CancelResponse.ACCEPT,
        )
        self.get_logger().info("Ready: /open_door")

    # ── Detection ingest ──────────────────────────────────────────────────

    def _on_detections(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        target = str(self.get_parameter("knob_class").value)
        best = None
        for det in payload.get("detections", []):
            if str(det.get("class_name", "")) != target:
                continue
            if best is None or float(det.get("score", 0.0)) > float(best.get("score", 0.0)):
                best = det
        if best is None:
            return

        w = float(self.get_parameter("image_width").value)
        h = float(self.get_parameter("image_height").value)
        bbox = best.get("bbox", {}) or {}
        cx = float(bbox.get("center_x", w / 2.0))
        cy = float(bbox.get("center_y", h / 2.0))
        depth_valid = bool(best.get("depth_valid", False))
        depth_m = float(best.get("depth_m", 0.0)) if depth_valid else 0.0

        with self._lock:
            self._latest = Detection(
                stamp_monotonic=time.monotonic(),
                x_norm=(cx - 0.5 * w) / (0.5 * w),
                y_norm=(cy - 0.5 * h) / (0.5 * h),
                depth_m=depth_m,
                depth_valid=depth_valid,
            )

    def _fresh(self) -> Optional[Detection]:
        timeout = float(self.get_parameter("detection_timeout_sec").value)
        with self._lock:
            d = self._latest
        if d is None:
            return None
        if (time.monotonic() - d.stamp_monotonic) > timeout:
            return None
        return d

    # ── Action execute ────────────────────────────────────────────────────

    def _execute(self, goal_handle):
        result = OpenDoor.Result()
        goal = goal_handle.request

        if goal.knob_class:
            self.set_parameters([Parameter("knob_class", value=str(goal.knob_class))])
        if goal.ready_distance_m and float(goal.ready_distance_m) > 0.0:
            self.set_parameters(
                [Parameter("ready_distance_m", value=float(goal.ready_distance_m))]
            )

        period = 1.0 / float(self.get_parameter("control_rate_hz").value)
        state = State.ALIGN
        state_entered = time.monotonic()
        align_stable = 0
        push_started = 0.0

        self._publish_feedback(goal_handle, State.ALIGN.value, 0.05, "starting alignment")

        def lost_too_long() -> bool:
            grace = float(self.get_parameter("lost_grace_sec").value)
            return (time.monotonic() - state_entered) > grace

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._publish_twist(0.0, 0.0)
                goal_handle.canceled()
                result.success = False
                result.message = "canceled"
                return result

            det = self._fresh()

            # ── ALIGN ────────────────────────────────────────────────────
            if state == State.ALIGN:
                if det is None:
                    self._publish_twist(0.0, 0.0)
                    if lost_too_long():
                        state, state_entered, align_stable = State.ABORT, time.monotonic(), 0
                        self._publish_feedback(goal_handle, state.value, 0.0, "knob lost in ALIGN")
                else:
                    tol = float(self.get_parameter("align_pixel_tol").value)
                    kp = float(self.get_parameter("align_kp").value)
                    max_wz = float(self.get_parameter("align_max_wz").value)
                    wz = max(-max_wz, min(max_wz, -kp * det.x_norm))
                    self._publish_twist(0.0, wz)
                    align_stable = align_stable + 1 if abs(det.x_norm) < tol else 0
                    if align_stable >= int(self.get_parameter("align_stable_ticks").value):
                        self._publish_twist(0.0, 0.0)
                        self._publish_feedback(
                            goal_handle,
                            State.APPROACH.value,
                            0.25,
                            f"aligned (x_norm={det.x_norm:+.3f})",
                        )
                        state, state_entered, align_stable = State.APPROACH, time.monotonic(), 0

            # ── APPROACH ─────────────────────────────────────────────────
            elif state == State.APPROACH:
                if det is None or not det.depth_valid:
                    self._publish_twist(0.0, 0.0)
                    if lost_too_long():
                        state, state_entered = State.ABORT, time.monotonic()
                        self._publish_feedback(goal_handle, state.value, 0.0, "knob/depth lost in APPROACH")
                else:
                    ready = float(self.get_parameter("ready_distance_m").value)
                    if det.depth_m > 0.0 and det.depth_m <= ready:
                        self._publish_twist(0.0, 0.0)
                        self._publish_feedback(
                            goal_handle,
                            State.PRESS.value,
                            0.5,
                            f"at ready depth={det.depth_m:.3f}m",
                        )
                        state, state_entered = State.PRESS, time.monotonic()
                    else:
                        speed = float(self.get_parameter("approach_speed").value)
                        kp = float(self.get_parameter("approach_center_kp").value)
                        max_wz = float(self.get_parameter("approach_max_wz").value)
                        wz = max(-max_wz, min(max_wz, -kp * det.x_norm))
                        self._publish_twist(speed, wz)

            # ── PRESS ────────────────────────────────────────────────────
            elif state == State.PRESS:
                self._publish_twist(0.0, 0.0)
                self.arm.send_degrees("door_above", self._pose("door_above_pose_deg"))
                self.arm.send_degrees("door_press", self._pose("door_press_pose_deg"))
                time.sleep(float(self.get_parameter("press_hold_sec").value))
                self._publish_feedback(
                    goal_handle, State.PUSH.value, 0.7, "handle pressed; pushing door"
                )
                self.arm.publish_degrees("door_push", self._pose("door_push_pose_deg"))
                push_started = time.monotonic()
                state, state_entered = State.PUSH, time.monotonic()
                continue

            # ── PUSH ─────────────────────────────────────────────────────
            elif state == State.PUSH:
                self._publish_twist(float(self.get_parameter("push_speed").value), 0.0)
                if (time.monotonic() - push_started) >= float(
                    self.get_parameter("push_duration_sec").value
                ):
                    self._publish_twist(0.0, 0.0)
                    self._publish_feedback(goal_handle, State.COMPLETE.value, 0.95, "push duration met")
                    state, state_entered = State.COMPLETE, time.monotonic()

            # ── COMPLETE ─────────────────────────────────────────────────
            elif state == State.COMPLETE:
                self._publish_twist(0.0, 0.0)
                self.arm.send_degrees("door_home", self._pose("door_home_pose_deg"))
                goal_handle.succeed()
                result.success = True
                result.message = "door opened"
                return result

            # ── ABORT ────────────────────────────────────────────────────
            elif state == State.ABORT:
                self._publish_twist(0.0, 0.0)
                self.arm.send_degrees("door_home", self._pose("door_home_pose_deg"))
                goal_handle.abort()
                result.success = False
                result.message = "aborted"
                return result

            time.sleep(period)

        # rclpy.ok() flipped — shutdown path.
        self._publish_twist(0.0, 0.0)
        result.success = False
        result.message = "shutdown"
        return result

    # ── Helpers ───────────────────────────────────────────────────────────

    def _publish_twist(self, vx: float, wz: float) -> None:
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self._twist_pub.publish(msg)

    def _publish_feedback(self, goal_handle, stage: str, progress: float, detail: str) -> None:
        fb = OpenDoor.Feedback()
        fb.stage = stage
        fb.progress = float(progress)
        fb.detail = detail
        try:
            goal_handle.publish_feedback(fb)
        except Exception:  # noqa: BLE001
            pass
        self.get_logger().info(f"{stage}: {detail}")

    def _pose(self, name: str) -> list[float]:
        return [float(v) for v in self.get_parameter(name).value]


def main(args=None):
    rclpy.init(args=args)
    node = OpenDoorServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
