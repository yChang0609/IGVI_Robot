"""Interactive tuner for open_door_server parameters.

Run with the rest of the stack already up: kros_car + arm_safeguard +
motion_arbiter + Kinect + eto_eye_gpu (publishing /detections_json).

What it does:
  * Jogs the arm with the keyboard, publishing JointTrajectory on
    /arm_safeguard/target_trajectory (same topic as ArmCommander).
  * Drives the base with the keyboard, publishing Twist on /motion/cmd
    (motion_arbiter handles smoothing). Republished at 10 Hz so the
    arbiter's override stays alive.
  * Snapshots actual joint angles from /joint_states.
  * Reads live knob depth from /detections_json (JSON, eto_eye output).
  * Writes a YAML snippet with door_*_pose_deg and ready_distance_m, ready
    to merge into your open_door_server params.

You should stop open_door_server (and grab_object_server, if running) while
tuning, so nothing else is fighting you for /motion/cmd or
/arm_safeguard/target_trajectory:

    docker compose -f docker/compose/compose.yaml stop open_door_server wildbot_grasp
"""

from __future__ import annotations

import json
import math
import os
import select
import sys
import termios
import threading
import time
import tty
from typing import Optional

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = ["arm_1_joint", "arm_2_joint", "gripper_joint"]

HELP = r"""
=========================================================================
 open_door tuner — drive the robot and capture parameters from reality
=========================================================================

ARM JOG (publishes JointTrajectory; angles tracked in radians, saved in deg):
  u / j   arm_1_joint    +/-
  i / k   arm_2_joint    +/-
  o / l   gripper_joint  +/-
  ,       halve arm step          .   double arm step

BASE JOG (publishes Twist on /motion/cmd; arbiter handles smoothing):
  w / s   linear  +/-                    a / d   angular +/-
  <space> stop base

CAPTURE  (snapshots whatever is true *right now*):
  1  door_home_pose_deg   (rest / retracted pose)
  2  door_above_pose_deg  (gripper hovering just above the handle)
  3  door_press_pose_deg  (gripper has pressed the handle down)
  4  door_push_pose_deg   (arm extended forward to push the door)
  5  ready_distance_m     (current depth to detected knob)

OTHER:
  v  view all captured values        ?  print this help
  f  write YAML and exit             x  quit without saving
=========================================================================
"""


class Tuner(Node):
    def __init__(self):
        super().__init__("open_door_tuner")
        self.declare_parameter("arm_topic", "/arm_safeguard/target_trajectory")
        self.declare_parameter("twist_topic", "/motion/cmd")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("detection_topic", "/detections_json")
        self.declare_parameter("knob_class", "men_ba")
        self.declare_parameter("move_duration_sec", 0.8)

        self._lock = threading.Lock()
        self._js: dict[str, float] = {}
        self._depth_m: float = 0.0
        self._depth_stamp_monotonic: float = 0.0

        self._cmd_arm_rad: dict[str, float] = {j: 0.0 for j in ARM_JOINTS}
        self._cmd_arm_initialized = False
        self._cmd_vx: float = 0.0
        self._cmd_wz: float = 0.0

        self.create_subscription(
            JointState, str(self.get_parameter("joint_states_topic").value), self._on_js, 10
        )
        self.create_subscription(
            String, str(self.get_parameter("detection_topic").value), self._on_det, 10
        )
        self._arm_pub = self.create_publisher(
            JointTrajectory, str(self.get_parameter("arm_topic").value), 10
        )
        self._twist_pub = self.create_publisher(
            Twist, str(self.get_parameter("twist_topic").value), 10
        )

        self.create_timer(0.1, self._republish_twist)

    def _on_js(self, msg: JointState) -> None:
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                self._js[name] = float(pos)
            if not self._cmd_arm_initialized and all(j in self._js for j in ARM_JOINTS):
                for j in ARM_JOINTS:
                    self._cmd_arm_rad[j] = self._js[j]
                self._cmd_arm_initialized = True

    def _on_det(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        target = str(self.get_parameter("knob_class").value)
        best = None
        for det in payload.get("detections", []):
            if str(det.get("class_name", "")) != target:
                continue
            if not bool(det.get("depth_valid", False)):
                continue
            if best is None or float(det.get("score", 0.0)) > float(best.get("score", 0.0)):
                best = det
        if best is None:
            return
        with self._lock:
            self._depth_m = float(best.get("depth_m", 0.0))
            self._depth_stamp_monotonic = time.monotonic()

    def _republish_twist(self) -> None:
        t = Twist()
        with self._lock:
            t.linear.x = self._cmd_vx
            t.angular.z = self._cmd_wz
        self._twist_pub.publish(t)

    def joint_snapshot_deg(self) -> Optional[list[float]]:
        with self._lock:
            if not all(j in self._js for j in ARM_JOINTS):
                return None
            return [math.degrees(self._js[j]) for j in ARM_JOINTS]

    def depth_snapshot(self) -> Optional[float]:
        with self._lock:
            if self._depth_stamp_monotonic == 0.0:
                return None
            if (time.monotonic() - self._depth_stamp_monotonic) > 1.0:
                return None
            if self._depth_m <= 0.0:
                return None
            return self._depth_m

    def cmd_arm_ready(self) -> bool:
        with self._lock:
            return self._cmd_arm_initialized

    def jog_arm(self, joint: str, delta_rad: float) -> bool:
        with self._lock:
            if not self._cmd_arm_initialized:
                return False
            self._cmd_arm_rad[joint] += delta_rad
            positions = [self._cmd_arm_rad[j] for j in ARM_JOINTS]
        duration_sec = float(self.get_parameter("move_duration_sec").value)
        msg = JointTrajectory()
        msg.joint_names = list(ARM_JOINTS)
        pt = JointTrajectoryPoint()
        pt.positions = positions
        sec = int(duration_sec)
        pt.time_from_start = Duration(sec=sec, nanosec=int((duration_sec - sec) * 1e9))
        msg.points = [pt]
        self._arm_pub.publish(msg)
        return True

    def adjust_twist(self, dvx: float = 0.0, dwz: float = 0.0, *, stop: bool = False) -> tuple[float, float]:
        with self._lock:
            if stop:
                self._cmd_vx = 0.0
                self._cmd_wz = 0.0
            else:
                self._cmd_vx += dvx
                self._cmd_wz += dwz
            return self._cmd_vx, self._cmd_wz


def _read_key(timeout: float = 0.1) -> Optional[str]:
    r, _, _ = select.select([sys.stdin], [], [], timeout)
    return sys.stdin.read(1) if r else None


def _format_pose(pose: Optional[list[float]]) -> str:
    if pose is None:
        return "(unset)"
    return "[" + ", ".join(f"{v:+.3f}" for v in pose) + "]"


def _write_yaml(path: str, captures: dict[str, list[float]], ready_distance_m: Optional[float]) -> None:
    def fmt_pose(p: Optional[list[float]]) -> str:
        if p is None:
            return "[0.0, 0.0, 0.0]  # UNSET — not captured"
        return "[" + ", ".join(f"{v:.3f}" for v in p) + "]"

    rd_line = (
        f"    ready_distance_m: {ready_distance_m:.3f}\n"
        if ready_distance_m is not None
        else "    ready_distance_m: 0.45  # UNSET — keep default\n"
    )
    body = (
        "# Captured by wildbot_grasp tune_open_door — merge into open_door_server params.\n"
        "open_door_server:\n"
        "  ros__parameters:\n"
        f"{rd_line}"
        f"    door_home_pose_deg:  {fmt_pose(captures.get('door_home_pose_deg'))}\n"
        f"    door_above_pose_deg: {fmt_pose(captures.get('door_above_pose_deg'))}\n"
        f"    door_press_pose_deg: {fmt_pose(captures.get('door_press_pose_deg'))}\n"
        f"    door_push_pose_deg:  {fmt_pose(captures.get('door_push_pose_deg'))}\n"
    )
    with open(path, "w") as fh:
        fh.write(body)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Tuner()
    spinner = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spinner.start()

    arm_step = 0.05        # rad/keystroke
    base_lin_step = 0.02   # m/s
    base_ang_step = 0.10   # rad/s
    captures: dict[str, list[float]] = {}
    ready_distance_m: Optional[float] = None

    fd = sys.stdin.fileno()
    old_attr = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    try:
        print(HELP, flush=True)
        print("Waiting up to 5s for /joint_states ...", flush=True)
        deadline = time.time() + 5.0
        while time.time() < deadline and not node.cmd_arm_ready():
            time.sleep(0.1)
        if node.cmd_arm_ready():
            print("joint_states received. Begin tuning.\r", flush=True)
        else:
            print("WARN: no /joint_states — arm jog ignored until it arrives.\r", flush=True)

        while rclpy.ok():
            key = _read_key(0.1)
            if key is None:
                continue
            if key == "x":
                print("\rquit without saving.", flush=True)
                break
            if key == "?":
                print(HELP, flush=True)
                continue
            if key == "v":
                rd = f"{ready_distance_m:.3f} m" if ready_distance_m is not None else "(unset)"
                print("\r---- captured ----", flush=True)
                for label in ("door_home_pose_deg", "door_above_pose_deg",
                              "door_press_pose_deg", "door_push_pose_deg"):
                    print(f"\r  {label}: {_format_pose(captures.get(label))}", flush=True)
                print(f"\r  ready_distance_m: {rd}", flush=True)
                print(
                    f"\r  arm step={arm_step:.3f} rad   base step={base_lin_step:.3f} m/s, "
                    f"{base_ang_step:.3f} rad/s",
                    flush=True,
                )
                continue
            if key == ",":
                arm_step = max(0.005, arm_step / 2)
                print(f"\rarm step → {arm_step:.3f} rad", flush=True)
                continue
            if key == ".":
                arm_step = min(0.5, arm_step * 2)
                print(f"\rarm step → {arm_step:.3f} rad", flush=True)
                continue

            arm_map = {
                "u": ("arm_1_joint", +arm_step), "j": ("arm_1_joint", -arm_step),
                "i": ("arm_2_joint", +arm_step), "k": ("arm_2_joint", -arm_step),
                "o": ("gripper_joint", +arm_step), "l": ("gripper_joint", -arm_step),
            }
            if key in arm_map:
                joint, delta = arm_map[key]
                ok = node.jog_arm(joint, delta)
                msg = f"jog {joint} {delta:+.3f} rad" if ok else "jog ignored (no joint_states yet)"
                print(f"\r{msg}", flush=True)
                continue

            if key == "w":
                vx, wz = node.adjust_twist(dvx=+base_lin_step)
                print(f"\rbase: vx={vx:+.2f} m/s wz={wz:+.2f} rad/s", flush=True); continue
            if key == "s":
                vx, wz = node.adjust_twist(dvx=-base_lin_step)
                print(f"\rbase: vx={vx:+.2f} m/s wz={wz:+.2f} rad/s", flush=True); continue
            if key == "a":
                vx, wz = node.adjust_twist(dwz=+base_ang_step)
                print(f"\rbase: vx={vx:+.2f} m/s wz={wz:+.2f} rad/s", flush=True); continue
            if key == "d":
                vx, wz = node.adjust_twist(dwz=-base_ang_step)
                print(f"\rbase: vx={vx:+.2f} m/s wz={wz:+.2f} rad/s", flush=True); continue
            if key == " ":
                node.adjust_twist(stop=True)
                print("\rbase STOP", flush=True); continue

            if key in "1234":
                pose = node.joint_snapshot_deg()
                if pose is None:
                    print("\rERR: joint_states not available", flush=True)
                    continue
                label = {"1": "door_home_pose_deg", "2": "door_above_pose_deg",
                         "3": "door_press_pose_deg", "4": "door_push_pose_deg"}[key]
                captures[label] = pose
                print(f"\rsaved {label}: {_format_pose(pose)}", flush=True)
                continue
            if key == "5":
                z = node.depth_snapshot()
                if z is None:
                    print(
                        "\rERR: no fresh detection with depth — is eto_eye running and knob in frame?",
                        flush=True,
                    )
                    continue
                ready_distance_m = z
                print(f"\rsaved ready_distance_m: {z:.3f} m", flush=True)
                continue

            if key == "f":
                print("\rstopping base and writing YAML ...", flush=True)
                node.adjust_twist(stop=True)
                out_dir = os.environ.get("OPEN_DOOR_TUNER_OUTPUT") or (
                    "/tuner_output" if os.path.isdir("/tuner_output") else "."
                )
                out_path = os.path.abspath(os.path.join(out_dir, "tuned_open_door.yaml"))
                _write_yaml(out_path, captures, ready_distance_m)
                print(f"\rwrote: {out_path}", flush=True)
                print(
                    "\rMerge these into open_door_server parameters (or pass via --params-file).",
                    flush=True,
                )
                break

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_attr)
        node.adjust_twist(stop=True)
        try:
            node._republish_twist()
        except Exception:  # noqa: BLE001
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()
