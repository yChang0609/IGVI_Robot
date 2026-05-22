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
  PRESS     SLAM: play door_pose_1 → door_pose_2 → door_pose_3 in order (e.g.
            ready → raise straight up → slam straight down), each held
            pose_hold_sec. Unlatches the handle.
  ARM_PUSH  arm swings to door_arm_push_pose_deg to shove the door open with the
            arm, held arm_push_hold_sec. Skipped unless arm_push_after_slam.
  PUSH      base drives forward for push_duration_sec to open the door. Skipped
            unless drive_forward_after_poses.
  COMPLETE  stop base, retract to door_home_pose_deg, succeed
  ABORT     stop base, retract to door_home_pose_deg, abort

The three motions — arm slam (PRESS), arm push (ARM_PUSH), and base drive (PUSH)
— are independently tunable, savable, and triggerable (run_press / run_arm_push
/ run_push debug services) so a half-working step can never ram a latched door.

Base commands are published to /motion/cmd (Twist); motion_arbiter handles the
acceleration limiter and the /cmd_vel publish. The arbiter's override_timeout
is 0.6 s, so the FSM publishes every tick (10 Hz default).
"""

from __future__ import annotations

import os
import time
from dataclasses import dataclass
from enum import Enum
from threading import Lock
from typing import Optional

import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

from wildbot_grasp.action import OpenDoor

from .motion import ArmCommander
from .red_bar_detector import (
    RedBarParams,
    annotate as annotate_red_bar,
    detect as detect_red_bar,
)


# Parameters persisted to / restored from the poses YAML. Everything an
# operator tunes live in the UI, so a saved file fully reproduces a setup.
_PERSIST_PARAMS = [
    "ready_distance_m",
    "pose_hold_sec",
    "arm_push_after_slam",
    "arm_push_hold_sec",
    "drive_forward_after_poses",
    "hold_pose_during_push",
    "push_hold_repub_sec",
    "push_duration_sec",
    "push_speed",
    "door_home_pose_deg",
    "door_pose_1_deg",
    "door_pose_2_deg",
    "door_pose_3_deg",
    "door_arm_push_pose_deg",
    "red_hue_lo1",
    "red_hue_hi1",
    "red_hue_lo2",
    "red_hue_hi2",
    "red_sat_min",
    "red_val_min",
    "min_red_area_px",
    "aim_offset_px",
    "depth_inset_px",
    "depth_window_px",
]


class State(Enum):
    IDLE = "idle"
    ALIGN = "align"
    APPROACH = "approach"
    PRESS = "press"        # arm slam-down (unlatch the handle)
    ARM_PUSH = "arm_push"  # arm shoves the door open
    PUSH = "push"          # base drives forward
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

        # ── Persistence ──────────────────────────────────────────────────
        self.declare_parameter("poses_file", "/tuner_output/door_poses.yaml")

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

        # ── PRESS (arm slam) / ARM_PUSH (arm shove) / PUSH (drive) ───────
        self.declare_parameter("pose_hold_sec", 0.3)   # dwell between slam poses
        # Arm push: swing the arm to door_arm_push_pose_deg to shove the door
        # open after the slam unlatches it. Off by default so a mis-tuned push
        # pose can't drive the arm into a still-latched door.
        self.declare_parameter("arm_push_after_slam", False)
        self.declare_parameter("arm_push_hold_sec", 0.5)
        # Safety: defaults False so the base never rams a still-latched door.
        # Enable it from the UI only once pose 3 reliably opens/unlatches.
        self.declare_parameter("drive_forward_after_poses", False)
        # Re-assert pose 3 while pushing so the handle stays held down (defeats
        # the safeguard's relax) and the latch can't re-engage mid-push.
        self.declare_parameter("hold_pose_during_push", True)
        self.declare_parameter("push_hold_repub_sec", 0.4)
        self.declare_parameter("push_duration_sec", 3.0)
        self.declare_parameter("push_speed", 0.08)

        # ── Arm poses (degrees) — tune live from the UI Door tab ─────────
        # 3 DOF: [arm_1_joint, arm_2_joint, gripper_joint]. The three sequence
        # poses play in order during PRESS, e.g. ready → raise straight up →
        # push straight down. gripper must stay >= 168° or the motor overheats.
        self.declare_parameter("door_pose_1_deg", [167.0, 80.0, 170.6])
        self.declare_parameter("door_pose_2_deg", [167.0, 100.0, 170.6])
        self.declare_parameter("door_pose_3_deg", [167.0, 50.0, 170.6])
        # Single target pose the arm swings to during ARM_PUSH (shove the door
        # open with the arm). Defaults to the slammed-down pose so an unset value
        # is a no-op rather than a wild swing — tune it live from the UI.
        self.declare_parameter("door_arm_push_pose_deg", [167.0, 50.0, 170.6])
        self.declare_parameter("door_home_pose_deg", [167.0, 75.0, 170.6])

        self._cb_group = ReentrantCallbackGroup()
        self._lock = Lock()
        # Serializes base/arm motion across the action and the debug services so
        # a manual run_press/run_push can't fight an in-flight open_door goal.
        self._motion_lock = Lock()
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

        # Restore tuned poses/params from a previous save, then expose a service
        # the UI calls to write the current values back out.
        self._load_poses_file()
        self._save_srv = self.create_service(
            Trigger, "~/save_poses", self._on_save_poses, callback_group=self._cb_group
        )

        # Debug entry points: run the arm press and the forward push as separate,
        # independently-triggerable steps so a half-working sequence can't ram the
        # door. Each grabs _motion_lock and refuses to overlap the action or
        # another debug call. Call with e.g.:
        #   ros2 service call /open_door_server/run_press std_srvs/srv/Trigger
        self._run_press_srv = self.create_service(
            Trigger, "~/run_press", self._on_run_press, callback_group=self._cb_group
        )
        self._run_arm_push_srv = self.create_service(
            Trigger, "~/run_arm_push", self._on_run_arm_push, callback_group=self._cb_group
        )
        self._run_push_srv = self.create_service(
            Trigger, "~/run_push", self._on_run_push, callback_group=self._cb_group
        )
        self._go_home_srv = self.create_service(
            Trigger, "~/go_home", self._on_go_home, callback_group=self._cb_group
        )

        self.get_logger().info(
            "Ready: /open_door (red-bar detector); debug services: "
            "~/run_press ~/run_arm_push ~/run_push ~/go_home"
        )

    # ── Persistence (YAML save / load) ─────────────────────────────────────

    def _poses_path(self) -> str:
        return str(self.get_parameter("poses_file").value)

    def _load_poses_file(self) -> None:
        path = self._poses_path()
        if not path or not os.path.isfile(path):
            self.get_logger().info(f"No saved poses at {path}; using defaults.")
            return
        try:
            with open(path) as fh:
                doc = yaml.safe_load(fh) or {}
            params = (doc.get("open_door_server", {}) or {}).get("ros__parameters", {}) or {}
        except (OSError, yaml.YAMLError) as exc:
            self.get_logger().warning(f"Could not read {path}: {exc}")
            return
        to_set = []
        for name in _PERSIST_PARAMS:
            if name in params and params[name] is not None:
                to_set.append(Parameter(name, value=params[name]))
        if to_set:
            self.set_parameters(to_set)
            self.get_logger().info(f"Restored {len(to_set)} param(s) from {path}")

    def _save_poses_file(self) -> tuple[bool, str]:
        path = self._poses_path()
        ros_params = {name: self.get_parameter(name).value for name in _PERSIST_PARAMS}
        doc = {"open_door_server": {"ros__parameters": ros_params}}
        try:
            os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
            with open(path, "w") as fh:
                fh.write("# open_door_server poses/params — saved from the UI.\n")
                fh.write("# Loaded automatically on startup; usable as --params-file too.\n")
                yaml.safe_dump(doc, fh, default_flow_style=False, sort_keys=False)
        except OSError as exc:
            return False, f"write failed: {exc}"
        return True, f"saved {len(ros_params)} param(s) to {path}"

    def _on_save_poses(self, _request, response):
        ok, message = self._save_poses_file()
        response.success = ok
        response.message = message
        self.get_logger().info(message)
        return response

    # ── Debug services: run each door step independently ───────────────────

    def _on_run_press(self, _request, response):
        """Play the arm pose sequence once; the base never moves."""
        if not self._motion_lock.acquire(blocking=False):
            response.success = False
            response.message = "busy: another door motion is running"
            return response
        try:
            self._fsm_state = State.PRESS.value
            self._run_press_sequence()
            self._fsm_state = State.IDLE.value
            response.success = True
            response.message = "press done: arm pose 1→2→3; base did not move"
        except Exception as exc:  # noqa: BLE001
            self._publish_twist(0.0, 0.0)
            response.success = False
            response.message = f"press failed: {exc}"
        finally:
            self._motion_lock.release()
        self.get_logger().info(f"run_press: {response.message}")
        return response

    def _on_run_arm_push(self, _request, response):
        """Swing the arm to door_arm_push_pose_deg once; the base never moves."""
        if not self._motion_lock.acquire(blocking=False):
            response.success = False
            response.message = "busy: another door motion is running"
            return response
        try:
            self._fsm_state = State.ARM_PUSH.value
            self._run_arm_push_sequence()
            self._fsm_state = State.IDLE.value
            response.success = True
            response.message = "arm push done: arm at door_arm_push_pose; base did not move"
        except Exception as exc:  # noqa: BLE001
            self._publish_twist(0.0, 0.0)
            response.success = False
            response.message = f"arm push failed: {exc}"
        finally:
            self._motion_lock.release()
        self.get_logger().info(f"run_arm_push: {response.message}")
        return response

    def _on_run_push(self, _request, response):
        """Drive the base forward for push_duration_sec, holding pose 3."""
        if not self._motion_lock.acquire(blocking=False):
            response.success = False
            response.message = "busy: another door motion is running"
            return response
        try:
            dur = float(self.get_parameter("push_duration_sec").value)
            self._fsm_state = State.PUSH.value
            self._run_push()
            self._fsm_state = State.IDLE.value
            response.success = True
            response.message = f"push done: forward {dur:.1f}s holding pose 3"
        except Exception as exc:  # noqa: BLE001
            self._publish_twist(0.0, 0.0)
            response.success = False
            response.message = f"push failed: {exc}"
        finally:
            self._motion_lock.release()
        self.get_logger().info(f"run_push: {response.message}")
        return response

    def _on_go_home(self, _request, response):
        """Stop the base and retract the arm to door_home_pose_deg."""
        if not self._motion_lock.acquire(blocking=False):
            response.success = False
            response.message = "busy: another door motion is running"
            return response
        try:
            self._publish_twist(0.0, 0.0)
            self.arm.send_degrees("door_home", self._pose("door_home_pose_deg"))
            self._fsm_state = State.IDLE.value
            response.success = True
            response.message = "returned to door_home_pose"
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = f"go_home failed: {exc}"
        finally:
            self._motion_lock.release()
        self.get_logger().info(f"go_home: {response.message}")
        return response

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

    # ── Reusable motion primitives (shared by action + debug services) ─────

    def _run_press_sequence(self, feedback=None) -> None:
        """Play door_pose_1 → 2 → 3 in order, holding each pose_hold_sec.

        Arm-only: the base is explicitly stopped and never commanded here. This
        is the "arm press the handle down" half of the old PRESS→PUSH motion.
        """
        self._publish_twist(0.0, 0.0)
        hold = float(self.get_parameter("pose_hold_sec").value)
        for i in (1, 2, 3):
            if feedback is not None:
                feedback(State.PRESS.value, 0.5 + 0.05 * i, f"arm pose {i}/3")
            self.arm.send_degrees(f"door_pose_{i}", self._pose(f"door_pose_{i}_deg"))
            if hold > 0.0:
                time.sleep(hold)

    def _run_arm_push_sequence(self, feedback=None) -> None:
        """Swing the arm to door_arm_push_pose_deg to shove the door open.

        Arm-only: the base is explicitly stopped and never commanded here. This
        is the "push the door open with the arm" step, distinct from the base
        forward drive (_run_push).
        """
        self._publish_twist(0.0, 0.0)
        if feedback is not None:
            feedback(State.ARM_PUSH.value, 0.65, "arm pushing door open")
        self.arm.send_degrees("door_arm_push", self._pose("door_arm_push_pose_deg"))
        hold = float(self.get_parameter("arm_push_hold_sec").value)
        if hold > 0.0:
            time.sleep(hold)

    def _run_push(self, should_continue=None, feedback=None) -> None:
        """Drive the base forward for push_duration_sec, re-asserting pose 3.

        The "push the door open" half. When hold_pose_during_push is set,
        door_pose_3 is re-published every push_hold_repub_sec so the handle stays
        pressed (the safeguard otherwise relaxes and the latch re-engages). The
        base is always stopped on exit. should_continue() lets a caller (the
        action) bail early, e.g. on cancel.
        """
        if feedback is not None:
            feedback(State.PUSH.value, 0.7, "pushing forward (holding pose 3)")
        period = 1.0 / float(self.get_parameter("control_rate_hz").value)
        push_speed = float(self.get_parameter("push_speed").value)
        duration = float(self.get_parameter("push_duration_sec").value)
        hold_pose = bool(self.get_parameter("hold_pose_during_push").value)
        repub = float(self.get_parameter("push_hold_repub_sec").value)
        push_started = time.monotonic()
        last_hold_pub = 0.0
        try:
            while rclpy.ok():
                if should_continue is not None and not should_continue():
                    break
                now = time.monotonic()
                self._publish_twist(push_speed, 0.0)
                if hold_pose and (now - last_hold_pub) >= repub:
                    self.arm.publish_degrees("door_pose_3_hold", self._pose("door_pose_3_deg"))
                    last_hold_pub = now
                if (now - push_started) >= duration:
                    break
                time.sleep(period)
        finally:
            self._publish_twist(0.0, 0.0)

    # ── Action execute ────────────────────────────────────────────────────

    def _execute(self, goal_handle):
        result = OpenDoor.Result()
        # One motion at a time: refuse the goal if a debug service holds the lock.
        if not self._motion_lock.acquire(blocking=False):
            goal_handle.abort()
            result.success = False
            result.message = "busy: a debug motion (run_press/run_push) is running"
            return result
        try:
            return self._run_open_door(goal_handle, result)
        finally:
            self._publish_twist(0.0, 0.0)
            self._motion_lock.release()

    def _run_open_door(self, goal_handle, result):
        goal = goal_handle.request

        if goal.ready_distance_m and float(goal.ready_distance_m) > 0.0:
            self.set_parameters(
                [Parameter("ready_distance_m", value=float(goal.ready_distance_m))]
            )

        period = 1.0 / float(self.get_parameter("control_rate_hz").value)
        state = State.ALIGN
        state_entered = time.monotonic()
        align_stable = 0

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

            # ── PRESS: slam-down — play the three sequence poses in order ─
            elif state == State.PRESS:
                self._run_press_sequence(
                    feedback=lambda s, p, d: self._publish_feedback(goal_handle, s, p, d)
                )
                # Arm push first if enabled, then the base drive — each gated
                # independently so a slam that didn't unlatch can't get shoved.
                if bool(self.get_parameter("arm_push_after_slam").value):
                    self._publish_feedback(
                        goal_handle, State.ARM_PUSH.value, 0.6, "slam done; arm pushing door"
                    )
                    state, state_entered = State.ARM_PUSH, time.monotonic()
                    continue
                if bool(self.get_parameter("drive_forward_after_poses").value):
                    self._publish_feedback(
                        goal_handle, State.PUSH.value, 0.7, "slam done; holding handle + pushing"
                    )
                    state, state_entered = State.PUSH, time.monotonic()
                    continue
                self._publish_feedback(
                    goal_handle, State.COMPLETE.value, 0.9,
                    "slam done; arm push + forward drive disabled",
                )
                state, state_entered = State.COMPLETE, time.monotonic()
                continue

            # ── ARM_PUSH: shove the door open with the arm ───────────────
            elif state == State.ARM_PUSH:
                self._run_arm_push_sequence(
                    feedback=lambda s, p, d: self._publish_feedback(goal_handle, s, p, d)
                )
                if bool(self.get_parameter("drive_forward_after_poses").value):
                    self._publish_feedback(
                        goal_handle, State.PUSH.value, 0.75, "arm push done; holding handle + driving"
                    )
                    state, state_entered = State.PUSH, time.monotonic()
                    continue
                self._publish_feedback(
                    goal_handle, State.COMPLETE.value, 0.9,
                    "arm push done; forward drive disabled",
                )
                state, state_entered = State.COMPLETE, time.monotonic()
                continue

            # ── PUSH: ease base forward while re-asserting pose 3 ────────
            elif state == State.PUSH:
                self._run_push(
                    should_continue=lambda: rclpy.ok() and not goal_handle.is_cancel_requested,
                    feedback=lambda s, p, d: self._publish_feedback(goal_handle, s, p, d),
                )
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = "canceled"
                    return result
                self._publish_feedback(goal_handle, State.COMPLETE.value, 0.95, "push duration met")
                state, state_entered = State.COMPLETE, time.monotonic()
                continue

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
