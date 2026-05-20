"""Door-opening FSM as a wildbot_grasp action server (color-based).

Centers and approaches the door's red push-bar; the physical knob sits at the
bar's right end, so the detector aims at the rightmost extent of the red mask.

Consumes:
  /rgb/image_raw                    sensor_msgs/Image  (Kinect color)
  /depth_to_rgb/image_raw           sensor_msgs/Image  (depth aligned to RGB)

Publishes:
  /motion/cmd                       geometry_msgs/Twist
  /arm_safeguard/target_trajectory  trajectory_msgs/JointTrajectory  (via ArmCommander)

FSM:
  ALIGN     rotate-in-place until |x_norm| < align_pixel_tol for N ticks
  APPROACH  drive forward with mild centering until depth_m <= ready_distance_m
  PRESS     arm: above-knob → press-down (timed via ArmCommander), then hold
  PUSH      arm: push-forward (non-blocking) + base drives forward for push_duration_sec
  COMPLETE  stop base, retract to door_home_pose_deg, succeed
  ABORT     stop base, retract to door_home_pose_deg, abort

Base commands are published to /motion/cmd (Twist); motion_arbiter handles the
acceleration limiter and the /cmd_vel publish. The arbiter's override_timeout
is 0.6 s, so the FSM publishes every tick (10 Hz default).
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from enum import Enum
from threading import Lock
from typing import Optional

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image

from wildbot_grasp.action import OpenDoor

from .motion import ArmCommander
from .red_bar_detector import (
    RedBarParams,
    annotate as annotate_red_bar,
    detect as detect_red_bar,
)


class State(Enum):
    IDLE = "idle"
    ALIGN = "align"
    APPROACH = "approach"
    PRESS = "press"
    PUSH = "push"
    COMPLETE = "complete"
    ABORT = "abort"


@dataclass
class Snapshot:
    stamp_monotonic: float
    x_norm: float            # [-1, 1] horizontal offset of aim point from image center
    depth_m: float           # 0.0 when depth_valid is False
    depth_valid: bool


class OpenDoorServer(Node):
    def __init__(self):
        super().__init__("open_door_server")

        # ── Camera input ─────────────────────────────────────────────────
        self.declare_parameter("rgb_topic", "/rgb/image_raw")
        self.declare_parameter("depth_topic", "/depth_to_rgb/image_raw")
        self.declare_parameter("detection_timeout_sec", 0.5)
        self.declare_parameter("lost_grace_sec", 1.5)

        # ── Debug visualization ──────────────────────────────────────────
        self.declare_parameter("debug_enabled", True)
        self.declare_parameter("debug_topic", "/open_door/debug_image")

        # ── Red bar detection (HSV) ──────────────────────────────────────
        self.declare_parameter("red_hue_lo1", 0)
        self.declare_parameter("red_hue_hi1", 10)
        self.declare_parameter("red_hue_lo2", 170)
        self.declare_parameter("red_hue_hi2", 179)
        self.declare_parameter("red_sat_min", 120)
        self.declare_parameter("red_val_min", 70)
        self.declare_parameter("min_red_area_px", 800)
        self.declare_parameter("aim_offset_px", 0)      # +ve nudges aim right of bar edge
        self.declare_parameter("depth_inset_px", 8)     # depth sampled inside the bar
        self.declare_parameter("depth_window_px", 5)

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
        self._latest: Optional[Snapshot] = None
        self._depth_img: Optional[np.ndarray] = None
        self._bridge = CvBridge()
        self._fsm_state: str = State.IDLE.value

        self.arm = ArmCommander(self)
        self._twist_pub = self.create_publisher(
            Twist, str(self.get_parameter("twist_topic").value), 10
        )
        self._debug_pub = self.create_publisher(
            Image, str(self.get_parameter("debug_topic").value), 5
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("depth_topic").value),
            self._on_depth,
            10,
            callback_group=self._cb_group,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("rgb_topic").value),
            self._on_rgb,
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
        self.get_logger().info("Ready: /open_door (red-bar detector)")

    # ── Detection ingest ──────────────────────────────────────────────────

    def _read_params(self) -> RedBarParams:
        return RedBarParams(
            hue_lo1=int(self.get_parameter("red_hue_lo1").value),
            hue_hi1=int(self.get_parameter("red_hue_hi1").value),
            hue_lo2=int(self.get_parameter("red_hue_lo2").value),
            hue_hi2=int(self.get_parameter("red_hue_hi2").value),
            sat_min=int(self.get_parameter("red_sat_min").value),
            val_min=int(self.get_parameter("red_val_min").value),
            min_area_px=int(self.get_parameter("min_red_area_px").value),
            aim_offset_px=int(self.get_parameter("aim_offset_px").value),
            depth_inset_px=int(self.get_parameter("depth_inset_px").value),
            depth_window_px=int(self.get_parameter("depth_window_px").value),
        )

    def _on_depth(self, msg: Image) -> None:
        try:
            img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"depth cv_bridge failed: {exc}", throttle_duration_sec=2.0
            )
            return
        with self._lock:
            self._depth_img = img

    def _on_rgb(self, msg: Image) -> None:
        try:
            rgb = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"rgb cv_bridge failed: {exc}", throttle_duration_sec=2.0
            )
            return
        with self._lock:
            depth = self._depth_img
        det, mask = detect_red_bar(rgb, depth, self._read_params())
        if det is not None:
            with self._lock:
                self._latest = Snapshot(
                    stamp_monotonic=time.monotonic(),
                    x_norm=det.x_norm,
                    depth_m=det.depth_m,
                    depth_valid=det.depth_valid,
                )

        if bool(self.get_parameter("debug_enabled").value):
            annotated = annotate_red_bar(rgb, det, mask, fsm_state=self._fsm_state)
            try:
                out_msg = self._bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
                out_msg.header = msg.header
                self._debug_pub.publish(out_msg)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(
                    f"debug publish failed: {exc}", throttle_duration_sec=2.0
                )

    def _fresh(self) -> Optional[Snapshot]:
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
                        self._publish_feedback(goal_handle, state.value, 0.0, "red bar lost in ALIGN")
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
                        self._publish_feedback(goal_handle, state.value, 0.0, "bar/depth lost in APPROACH")
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
                    goal_handle, State.PUSH.value, 0.7, "knob pressed; pushing door"
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
        self._fsm_state = stage
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
