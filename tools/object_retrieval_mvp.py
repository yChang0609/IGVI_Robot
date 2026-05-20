#!/usr/bin/env python3
"""Run the first object-retrieval MVP through the existing IGVI Host API.

This script intentionally uses the same HTTP API as the UI instead of adding a
new ROS package. It assumes the UI/host agent and bridge are already running.

Default flow:
  1. Go to target_1_pregrasp waypoint.
  2. Optionally creep forward for a short fixed duration.
  3. Execute a fixed arm/gripper pick sequence.
  4. Retreat.
  5. Go to home waypoint.
  6. Execute a fixed place sequence.

Use --nav-only for the first robot test: it only drives to the target waypoint
and stops there, without arm commands, approach creep, retreat, or return-home.

The first run should use --dry-run, then tune arm angles and approach timing.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from typing import Any


DONE_STATES = {"succeeded"}
FAIL_STATES = {"aborted", "rejected", "failed", "canceled", "unavailable"}


class HostApiError(RuntimeError):
    pass


@dataclass
class HostApi:
    base_url: str
    timeout: float = 5.0

    def request(
        self,
        method: str,
        path: str,
        payload: dict[str, Any] | None = None,
        timeout: float | None = None,
    ) -> Any:
        data = None
        headers = {"accept": "application/json"}
        if payload is not None:
            data = json.dumps(payload).encode("utf-8")
            headers["content-type"] = "application/json"

        req = urllib.request.Request(
            self.base_url.rstrip("/") + path,
            data=data,
            headers=headers,
            method=method,
        )
        try:
            with urllib.request.urlopen(req, timeout=timeout or self.timeout) as resp:
                body = resp.read().decode("utf-8")
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")
            raise HostApiError(f"{exc.code}: {detail}") from exc
        except OSError as exc:
            raise HostApiError(str(exc)) from exc

        if not body:
            return None
        return json.loads(body)

    def health(self) -> dict[str, Any]:
        return self.request("GET", "/api/health")

    def pose(self) -> dict[str, Any]:
        return self.request("GET", "/api/ros/pose")

    def latest_target(self) -> dict[str, Any]:
        return self.request("GET", "/api/ros/task/target")

    def nav_status(self) -> dict[str, Any]:
        return self.request("GET", "/api/ros/nav/status")

    def waypoints(self) -> dict[str, dict[str, float]]:
        data = self.request("GET", "/api/ros/waypoints")
        return dict((data or {}).get("waypoints", {}))

    def goto_waypoint(self, name: str) -> dict[str, Any]:
        return self.request(
            "POST",
            "/api/ros/waypoints/goto",
            {"name": name},
            timeout=10.0,
        )

    def nav_goal(self, x: float, y: float, yaw: float) -> dict[str, Any]:
        return self.request(
            "POST",
            "/api/ros/nav/goal",
            {"x": x, "y": y, "yaw": yaw},
            timeout=10.0,
        )

    def cmd_vel(self, linear_x: float, angular_z: float) -> None:
        self.request(
            "POST",
            "/api/ros/cmd_vel",
            {"linear_x": linear_x, "angular_z": angular_z},
        )

    def stop(self) -> None:
        self.request("POST", "/api/ros/stop", {})

    def arm(self, arm1_deg: float, arm2_deg: float, gripper_deg: float, seconds: float) -> None:
        self.request(
            "POST",
            "/api/ros/arm/trajectory",
            {
                "positions": [
                    math.radians(arm1_deg),
                    math.radians(arm2_deg),
                    math.radians(gripper_deg),
                ],
                "time_from_start": seconds,
            },
        )


def log(message: str) -> None:
    print(f"[object-mvp] {message}", flush=True)


def require_ok(result: dict[str, Any], action: str) -> None:
    if not result.get("ok"):
        raise HostApiError(f"{action} failed: {result.get('message') or result}")


def wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def load_detection_json(path: str) -> dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    if isinstance(data, list):
        if not data:
            raise HostApiError(f"detection JSON is empty: {path}")
        # Prefer highest confidence if a detector dumps a list of candidates.
        return max(data, key=lambda item: float(item.get("confidence", item.get("score", 0.0))))
    if isinstance(data, dict):
        if isinstance(data.get("detections"), list) and data["detections"]:
            return max(
                data["detections"],
                key=lambda item: float(item.get("confidence", item.get("score", 0.0))),
            )
        return data
    raise HostApiError(f"unsupported detection JSON format: {path}")


def detection_target(args: argparse.Namespace, pose: dict[str, Any]) -> tuple[float, float] | None:
    """Return bear/object position in map frame, or None to use waypoint mode.

    Supported sources:
      --bear-map-x/--bear-map-y
      --bear-base-x/--bear-base-y
      --detection-json with x_map/y_map or x_base/y_base fields
    """
    x_map = args.bear_map_x
    y_map = args.bear_map_y
    x_base = args.bear_base_x
    y_base = args.bear_base_y

    if args.use_latest_target:
        try:
            latest = args._api.latest_target()
        except HostApiError as exc:
            if not args.bridge_url:
                raise
            log(f"host target endpoint unavailable ({exc}); trying bridge target endpoint")
            latest = HostApi(args.bridge_url, timeout=args.http_timeout).request("GET", "/api/task/target")
        if not latest.get("ok"):
            raise HostApiError(str(latest.get("message") or "No latest target available"))
        log(f"latest target: {latest}")
        x_map = latest.get("x_map", x_map)
        y_map = latest.get("y_map", y_map)
        x_base = latest.get("x_base", x_base)
        y_base = latest.get("y_base", y_base)

    if args.detection_json:
        det = load_detection_json(args.detection_json)
        log(f"loaded detection: {det}")
        x_map = det.get("x_map", det.get("map_x", x_map))
        y_map = det.get("y_map", det.get("map_y", y_map))
        x_base = det.get("x_base", det.get("base_x", x_base))
        y_base = det.get("y_base", det.get("base_y", y_base))

    if x_map is not None and y_map is not None:
        return float(x_map), float(y_map)

    if x_base is not None and y_base is not None:
        robot_x = float(pose.get("x", 0.0))
        robot_y = float(pose.get("y", 0.0))
        robot_yaw = float(pose.get("yaw", 0.0))
        xb = float(x_base)
        yb = float(y_base)
        cos_y = math.cos(robot_yaw)
        sin_y = math.sin(robot_yaw)
        return (
            robot_x + cos_y * xb - sin_y * yb,
            robot_y + sin_y * xb + cos_y * yb,
        )

    return None


def pregrasp_goal(
    robot_pose: dict[str, Any],
    object_x: float,
    object_y: float,
    standoff: float,
) -> tuple[float, float, float]:
    robot_x = float(robot_pose.get("x", 0.0))
    robot_y = float(robot_pose.get("y", 0.0))
    dx = object_x - robot_x
    dy = object_y - robot_y
    distance = math.hypot(dx, dy)
    if distance < 1e-3:
        raise HostApiError("object position overlaps robot pose; cannot compute pregrasp goal")

    ux = dx / distance
    uy = dy / distance
    goal_x = object_x - ux * standoff
    goal_y = object_y - uy * standoff
    yaw = wrap_angle(math.atan2(object_y - goal_y, object_x - goal_x))
    return goal_x, goal_y, yaw


def wait_nav(api: HostApi, label: str, timeout_s: float, poll_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    last_state = ""
    while time.monotonic() < deadline:
        status = api.nav_status()
        state = str(status.get("state") or "idle")
        msg = str(status.get("message") or "")
        if state != last_state:
            log(f"{label}: nav state={state} {msg}")
            last_state = state
        if state in DONE_STATES:
            return
        if state in FAIL_STATES:
            raise HostApiError(f"{label}: navigation ended with state={state}: {msg}")
        time.sleep(poll_s)
    raise TimeoutError(f"{label}: navigation timeout after {timeout_s:.1f}s")


def sleep_step(seconds: float, dry_run: bool) -> None:
    if dry_run:
        return
    time.sleep(max(0.0, seconds))


def send_arm_step(
    api: HostApi,
    name: str,
    arm1: float,
    arm2: float,
    gripper: float,
    move_time: float,
    settle_time: float,
    dry_run: bool,
) -> None:
    log(f"arm {name}: arm1={arm1:.1f} arm2={arm2:.1f} gripper={gripper:.1f}")
    if not dry_run:
        api.arm(arm1, arm2, gripper, move_time)
    sleep_step(move_time + settle_time, dry_run)


def drive_for(
    api: HostApi,
    name: str,
    linear_x: float,
    angular_z: float,
    seconds: float,
    dry_run: bool,
) -> None:
    if seconds <= 0:
        log(f"{name}: skipped")
        return
    log(f"{name}: cmd_vel linear={linear_x:.3f} angular={angular_z:.3f} for {seconds:.2f}s")
    if dry_run:
        return
    end = time.monotonic() + seconds
    try:
        while time.monotonic() < end:
            api.cmd_vel(linear_x, angular_z)
            time.sleep(0.1)
    finally:
        api.stop()
        time.sleep(0.2)


def run(args: argparse.Namespace) -> None:
    api = HostApi(args.host_url, timeout=args.http_timeout)
    args._api = api

    if not args.yes and not args.dry_run:
        raise SystemExit("Refusing to move robot without --yes. Use --dry-run first.")

    log(f"host={args.host_url}")
    if args.dry_run:
        log("dry-run enabled: no robot commands will be sent")

    health = api.health()
    require_ok(health, "host health")

    pose = api.pose()
    if not pose.get("ok"):
        raise HostApiError("ROS pose is unavailable. Start bridge/localization first.")
    log(f"pose x={pose.get('x', 0):.2f} y={pose.get('y', 0):.2f} yaw={pose.get('yaw', 0):.2f}")

    nav = api.nav_status()
    if not nav.get("server_ready"):
        raise HostApiError(
            "Nav2 action server is not ready. Start navigation and check Probe Actions in the UI."
        )

    object_xy = detection_target(args, pose)
    using_detection_target = object_xy is not None

    if using_detection_target:
        object_x, object_y = object_xy
        goal_x, goal_y, goal_yaw = pregrasp_goal(pose, object_x, object_y, args.standoff)
        log(f"bear/object map position: x={object_x:.2f} y={object_y:.2f}")
        log(
            "computed pregrasp goal: "
            f"x={goal_x:.2f} y={goal_y:.2f} yaw={goal_yaw:.2f} standoff={args.standoff:.2f}m"
        )
    else:
        waypoints = api.waypoints()
        required = [args.target] if args.nav_only else [args.home, args.target]
        missing = [name for name in required if name not in waypoints]
        if missing:
            available = ", ".join(sorted(waypoints)) or "(none)"
            raise HostApiError(f"Missing waypoint(s): {', '.join(missing)}. Available: {available}")
        log(f"target waypoint: {args.target} -> {waypoints[args.target]}")
        if not args.nav_only:
            log(f"home waypoint:   {args.home} -> {waypoints[args.home]}")

    log("go to detected pregrasp goal" if using_detection_target else f"go to {args.target}")
    if not args.dry_run:
        if using_detection_target:
            require_ok(api.nav_goal(goal_x, goal_y, goal_yaw), "go detected pregrasp goal")
            wait_nav(api, "detected pregrasp", args.nav_timeout, args.poll)
        else:
            require_ok(api.goto_waypoint(args.target), f"go {args.target}")
            wait_nav(api, args.target, args.nav_timeout, args.poll)
        api.stop()

    if args.nav_only:
        log("nav-only complete")
        return

    drive_for(
        api,
        "fixed approach",
        args.approach_linear,
        0.0,
        args.approach_seconds,
        args.dry_run,
    )

    send_arm_step(
        api,
        "open/pre-pick",
        args.arm1_pre,
        args.arm2_pre,
        args.gripper_open,
        args.arm_move_time,
        args.arm_settle,
        args.dry_run,
    )
    send_arm_step(
        api,
        "close",
        args.arm1_pre,
        args.arm2_pre,
        args.gripper_close,
        args.arm_move_time,
        args.gripper_settle,
        args.dry_run,
    )
    send_arm_step(
        api,
        "lift",
        args.arm1_lift,
        args.arm2_lift,
        args.gripper_close,
        args.arm_move_time,
        args.arm_settle,
        args.dry_run,
    )

    drive_for(api, "retreat", -abs(args.retreat_linear), 0.0, args.retreat_seconds, args.dry_run)

    log(f"go to {args.home}")
    if not args.dry_run:
        require_ok(api.goto_waypoint(args.home), f"go {args.home}")
        wait_nav(api, args.home, args.nav_timeout, args.poll)

    send_arm_step(
        api,
        "place/lower",
        args.arm1_place,
        args.arm2_place,
        args.gripper_close,
        args.arm_move_time,
        args.arm_settle,
        args.dry_run,
    )
    send_arm_step(
        api,
        "release",
        args.arm1_place,
        args.arm2_place,
        args.gripper_open,
        args.arm_move_time,
        args.arm_settle,
        args.dry_run,
    )
    send_arm_step(
        api,
        "arm home",
        args.arm1_home,
        args.arm2_home,
        args.gripper_open,
        args.arm_move_time,
        args.arm_settle,
        args.dry_run,
    )

    if not args.dry_run:
        api.stop()
    log("done")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Execute the waypoint-based object retrieval MVP via IGVI Host API."
    )
    parser.add_argument("--host-url", default="http://127.0.0.1:8770")
    parser.add_argument(
        "--bridge-url",
        default="http://127.0.0.1:8771",
        help="Fallback bridge API used only for --use-latest-target.",
    )
    parser.add_argument("--home", default="home")
    parser.add_argument("--target", default="target_1_pregrasp")
    parser.add_argument("--bear-map-x", type=float, default=None)
    parser.add_argument("--bear-map-y", type=float, default=None)
    parser.add_argument("--bear-base-x", type=float, default=None)
    parser.add_argument("--bear-base-y", type=float, default=None)
    parser.add_argument(
        "--detection-json",
        default=None,
        help=(
            "YOLO/detection JSON file. Supported fields: x_map/y_map, map_x/map_y, "
            "x_base/y_base, or base_x/base_y. If it contains detections[], the highest "
            "confidence detection is used."
        ),
    )
    parser.add_argument(
        "--use-latest-target",
        action="store_true",
        help="Use latest /task/target_candidates exposed by /api/ros/task/target.",
    )
    parser.add_argument(
        "--standoff",
        type=float,
        default=0.65,
        help="Stop this many meters in front of the detected object.",
    )
    parser.add_argument("--yes", action="store_true", help="Allow robot motion.")
    parser.add_argument("--dry-run", action="store_true", help="Print the sequence without sending commands.")
    parser.add_argument(
        "--nav-only",
        action="store_true",
        help="Only navigate to --target and stop; do not approach, use arm, retreat, or return home.",
    )

    parser.add_argument("--nav-timeout", type=float, default=90.0)
    parser.add_argument("--poll", type=float, default=0.4)
    parser.add_argument("--http-timeout", type=float, default=5.0)

    parser.add_argument("--approach-seconds", type=float, default=0.0)
    parser.add_argument("--approach-linear", type=float, default=0.04)
    parser.add_argument("--retreat-seconds", type=float, default=1.5)
    parser.add_argument("--retreat-linear", type=float, default=0.06)

    parser.add_argument("--arm-move-time", type=float, default=0.8)
    parser.add_argument("--arm-settle", type=float, default=0.3)
    parser.add_argument("--gripper-settle", type=float, default=0.8)

    # Degrees, matching the existing Arm tab UI. Tune these on the robot.
    parser.add_argument("--arm1-pre", type=float, default=120.0)
    parser.add_argument("--arm2-pre", type=float, default=120.0)
    parser.add_argument("--arm1-lift", type=float, default=135.0)
    parser.add_argument("--arm2-lift", type=float, default=95.0)
    parser.add_argument("--arm1-place", type=float, default=120.0)
    parser.add_argument("--arm2-place", type=float, default=120.0)
    parser.add_argument("--arm1-home", type=float, default=120.0)
    parser.add_argument("--arm2-home", type=float, default=120.0)
    parser.add_argument("--gripper-open", type=float, default=240.0)
    parser.add_argument("--gripper-close", type=float, default=168.0)
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    try:
        run(args)
    except KeyboardInterrupt:
        print("\n[object-mvp] interrupted", file=sys.stderr)
        try:
            HostApi(args.host_url, timeout=args.http_timeout).stop()
        except Exception:
            pass
        return 130
    except Exception as exc:  # noqa: BLE001
        print(f"[object-mvp] ERROR: {exc}", file=sys.stderr)
        try:
            if not args.dry_run:
                HostApi(args.host_url, timeout=args.http_timeout).stop()
        except Exception:
            pass
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
