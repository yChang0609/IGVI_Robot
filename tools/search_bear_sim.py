#!/usr/bin/env python3
"""Simulate the competition bear-search flow through the IGVI Host API.

Flow:
  1. Record the current robot pose as the start pose.
  2. Search for the latest bear target.
  3. If no bear is found, wander slowly and keep searching.
  4. When found, navigate to a pregrasp pose in front of the bear.
  5. Pause, then navigate back to the recorded start pose.
"""

from __future__ import annotations

import argparse
import json
import math
import random
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

    def map(self) -> dict[str, Any]:
        return self.request("GET", "/api/ros/map")

    def latest_target(self) -> dict[str, Any]:
        return self.request("GET", "/api/ros/task/target")

    def bridge_target(self, bridge_url: str) -> dict[str, Any]:
        return HostApi(bridge_url, timeout=self.timeout).request("GET", "/api/task/target")

    def nav_status(self) -> dict[str, Any]:
        return self.request("GET", "/api/ros/nav/status")

    def nav_goal(self, x: float, y: float, yaw: float) -> dict[str, Any]:
        return self.request(
            "POST",
            "/api/ros/nav/goal",
            {"x": x, "y": y, "yaw": yaw},
            timeout=5.0,
        )

    def nav_cancel(self) -> dict[str, Any]:
        return self.request("POST", "/api/ros/nav/cancel", {}, timeout=5.0)

    def cmd_vel(self, linear_x: float, angular_z: float) -> None:
        self.request(
            "POST",
            "/api/ros/cmd_vel",
            {"linear_x": linear_x, "angular_z": angular_z},
        )

    def clear_costmap(self, target: str) -> dict[str, Any]:
        return self.request(
            "POST",
            "/api/ros/costmap/clear",
            {"target": target},
            timeout=5.0,
        )

    def stop(self) -> None:
        self.request("POST", "/api/ros/stop", {})


def log(message: str) -> None:
    print(f"[bear-sim] {message}", flush=True)


def require_ok(result: dict[str, Any], action: str) -> None:
    if not result.get("ok"):
        raise HostApiError(f"{action} failed: {result.get('message') or result}")


def wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def pregrasp_goal(
    robot_pose: dict[str, Any],
    object_x: float,
    object_y: float,
    standoff: float,
    lateral_offset: float,
) -> tuple[float, float, float]:
    robot_x = float(robot_pose.get("x", 0.0))
    robot_y = float(robot_pose.get("y", 0.0))
    dx = object_x - robot_x
    dy = object_y - robot_y
    if math.hypot(dx, dy) < 1e-3:
        raise HostApiError("bear position overlaps robot pose; cannot compute pregrasp goal")

    bearing = math.atan2(dy, dx)
    yaw = wrap_angle(bearing - math.atan2(lateral_offset, standoff))
    cos_y = math.cos(yaw)
    sin_y = math.sin(yaw)
    goal_x = object_x - (cos_y * standoff - sin_y * lateral_offset)
    goal_y = object_y - (sin_y * standoff + cos_y * lateral_offset)
    return goal_x, goal_y, yaw


def map_is_free(grid: dict[str, Any], col: int, row: int, clearance_cells: int) -> bool:
    width = int(grid.get("width", 0))
    height = int(grid.get("height", 0))
    data = grid.get("data") or []
    if width <= 0 or height <= 0 or len(data) < width * height:
        return False

    for dy in range(-clearance_cells, clearance_cells + 1):
        for dx in range(-clearance_cells, clearance_cells + 1):
            if dx * dx + dy * dy > clearance_cells * clearance_cells:
                continue
            cx = col + dx
            cy = row + dy
            if cx < 0 or cx >= width or cy < 0 or cy >= height:
                return False
            value = int(data[cy * width + cx])
            if value < 0 or value >= 50:
                return False
    return True


def random_map_goal(api: HostApi, args: argparse.Namespace) -> tuple[float, float, float]:
    grid = api.map()
    if not grid.get("ok"):
        raise HostApiError("map unavailable for random search")

    width = int(grid.get("width", 0))
    height = int(grid.get("height", 0))
    resolution = float(grid.get("resolution", 0.05))
    origin_x = float(grid.get("origin_x", 0.0))
    origin_y = float(grid.get("origin_y", 0.0))
    clearance_cells = max(1, int(math.ceil(args.random_goal_clearance / resolution)))
    if width <= clearance_cells * 2 or height <= clearance_cells * 2:
        raise HostApiError("map too small for random search")

    for _ in range(args.random_goal_attempts):
        col = random.randint(clearance_cells, width - clearance_cells - 1)
        row = random.randint(clearance_cells, height - clearance_cells - 1)
        if not map_is_free(grid, col, row, clearance_cells):
            continue
        x = origin_x + (col + 0.5) * resolution
        y = origin_y + (row + 0.5) * resolution
        yaw = random.uniform(-math.pi, math.pi)
        return x, y, yaw

    raise HostApiError("could not sample a free random goal from the map")


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


def wait_position(
    api: HostApi,
    label: str,
    target_x: float,
    target_y: float,
    tolerance: float,
    timeout_s: float,
    poll_s: float,
) -> None:
    deadline = time.monotonic() + timeout_s
    last_log = 0.0
    while time.monotonic() < deadline:
        pose = api.pose()
        if not pose.get("ok"):
            raise HostApiError(f"{label}: ROS pose is unavailable while waiting for position.")

        current_x = float(pose.get("x", 0.0))
        current_y = float(pose.get("y", 0.0))
        distance = math.hypot(target_x - current_x, target_y - current_y)
        now = time.monotonic()
        if now - last_log >= 1.0:
            log(
                f"{label}: pose x={current_x:.2f} y={current_y:.2f} "
                f"distance={distance:.2f}m"
            )
            last_log = now
        if distance <= tolerance:
            log(f"{label}: position reached within {tolerance:.2f}m")
            return
        time.sleep(poll_s)
    raise TimeoutError(f"{label}: position timeout after {timeout_s:.1f}s")


def target_candidates(payload: dict[str, Any], args: argparse.Namespace) -> list[dict[str, Any]]:
    raw_candidates = payload.get("detections")
    if not isinstance(raw_candidates, list) or not raw_candidates:
        raw_candidates = [payload]

    candidates = []
    for item in raw_candidates:
        if not isinstance(item, dict):
            continue
        if str(item.get("class_name", "")) not in args.target_classes:
            continue
        if float(item.get("confidence", item.get("score", 0.0))) < args.min_confidence:
            continue
        if item.get("x_map") is None or item.get("y_map") is None:
            continue
        candidates.append(item)
    return candidates


def latest_bear(api: HostApi, args: argparse.Namespace) -> dict[str, Any] | None:
    try:
        target = api.latest_target()
    except HostApiError as exc:
        log(f"host target unavailable ({exc}); trying bridge")
        try:
            target = api.bridge_target(args.bridge_url)
        except HostApiError as bridge_exc:
            log(f"bridge target unavailable ({bridge_exc})")
            return None

    if not target.get("ok"):
        return None
    candidates = target_candidates(target, args)
    if not candidates:
        log("no usable bear candidate in latest target")
        return None

    selected = max(candidates, key=lambda item: float(item.get("confidence", item.get("score", 0.0))))
    log(
        "selected bear: "
        f"mode=first-found candidates={len(candidates)} "
        f"class={selected.get('class_name')} conf={float(selected.get('confidence', 0.0)):.2f} "
        f"base=({float(selected.get('x_base', 0.0)):.2f},{float(selected.get('y_base', 0.0)):.2f})"
    )
    return selected


def wander_once(api: HostApi, args: argparse.Namespace, dry_run: bool) -> None:
    linear = random.uniform(0.0, args.wander_linear)
    angular = random.choice([-1.0, 1.0]) * random.uniform(args.wander_angular_min, args.wander_angular)
    seconds = random.uniform(args.wander_seconds_min, args.wander_seconds_max)
    log(f"no bear: wander linear={linear:.3f} angular={angular:.3f} seconds={seconds:.1f}")
    if dry_run:
        return

    end = time.monotonic() + seconds
    try:
        while time.monotonic() < end:
            api.cmd_vel(linear, angular)
            time.sleep(0.1)
    finally:
        api.stop()
        time.sleep(args.search_poll)


def search_random_goal_once(
    api: HostApi,
    args: argparse.Namespace,
    dry_run: bool,
) -> dict[str, Any] | None:
    goal_x, goal_y, goal_yaw = random_map_goal(api, args)
    log(f"no bear: random map goal x={goal_x:.2f} y={goal_y:.2f} yaw={goal_yaw:.2f}")
    if dry_run:
        time.sleep(args.search_poll)
        return None

    try:
        clear_costmaps(api)
        require_ok(api.nav_goal(goal_x, goal_y, goal_yaw), "random search goal")
    except HostApiError as exc:
        log(f"random search goal rejected: {exc}")
        return None

    deadline = time.monotonic() + args.random_goal_timeout
    last_state = ""
    while time.monotonic() < deadline:
        target = latest_bear(api, args)
        if target is not None:
            log("bear found while navigating random map goal")
            try:
                api.nav_cancel()
            except HostApiError as exc:
                log(f"cancel random goal failed: {exc}")
            api.stop()
            return target

        status = api.nav_status()
        state = str(status.get("state") or "idle")
        msg = str(status.get("message") or "")
        if state != last_state:
            log(f"random search: nav state={state} {msg}")
            last_state = state
        if state in FAIL_STATES:
            log(f"random search goal failed: state={state} {msg}")
            api.stop()
            return None

        pose = api.pose()
        if pose.get("ok"):
            distance = math.hypot(
                goal_x - float(pose.get("x", 0.0)),
                goal_y - float(pose.get("y", 0.0)),
            )
            if distance <= args.position_tolerance:
                log("random search: goal reached")
                api.stop()
                return None
        time.sleep(args.search_poll)

    log(f"random search goal timeout after {args.random_goal_timeout:.1f}s")
    try:
        api.nav_cancel()
    except HostApiError:
        pass
    api.stop()
    return None


def clear_costmaps(api: HostApi) -> None:
    for target in ("local", "global"):
        try:
            result = api.clear_costmap(target)
        except HostApiError as exc:
            log(f"clear {target} costmap failed: {exc}")
            continue
        log(f"clear {target} costmap: {result.get('message', result)}")


def retreat(api: HostApi, label: str, args: argparse.Namespace) -> None:
    if args.retreat_seconds <= 0.0:
        return
    log(
        f"{label}: slow reverse linear={-abs(args.retreat_linear):.3f} "
        f"seconds={args.retreat_seconds:.1f}"
    )
    end = time.monotonic() + args.retreat_seconds
    try:
        while time.monotonic() < end:
            api.cmd_vel(-abs(args.retreat_linear), 0.0)
            time.sleep(0.1)
    finally:
        api.stop()
        time.sleep(0.3)


def restore_heading(api: HostApi, target_yaw: float, args: argparse.Namespace) -> None:
    log(f"restore car heading: target yaw={target_yaw:.2f}")
    deadline = time.monotonic() + args.heading_timeout
    try:
        while time.monotonic() < deadline:
            pose = api.pose()
            if not pose.get("ok"):
                raise HostApiError("ROS pose is unavailable while restoring heading.")

            current_yaw = float(pose.get("yaw", 0.0))
            yaw_error = wrap_angle(target_yaw - current_yaw)
            log(f"restore car heading: current yaw={current_yaw:.2f} error={yaw_error:.2f}")
            if abs(yaw_error) <= args.heading_tolerance:
                log("restore car heading: aligned")
                return

            angular_z = clamp(
                args.heading_angular_sign * args.heading_gain * yaw_error,
                -args.heading_max_angular,
                args.heading_max_angular,
            )
            api.cmd_vel(0.0, angular_z)
            time.sleep(args.heading_poll)
    finally:
        api.stop()
        time.sleep(0.2)

    raise TimeoutError(f"restore heading timeout after {args.heading_timeout:.1f}s")


def center_bear(api: HostApi, args: argparse.Namespace) -> None:
    log("center bear: start slow visual/base-frame correction")
    deadline = time.monotonic() + args.center_timeout
    stable_since: float | None = None
    try:
        while time.monotonic() < deadline:
            target = latest_bear(api, args)
            if target is None:
                api.stop()
                stable_since = None
                time.sleep(args.center_poll)
                continue

            x_base = float(target.get("x_base", 0.0))
            y_base = float(target.get("y_base", 0.0))
            lateral_error = y_base - args.center_lateral_offset
            distance_error = x_base - args.center_standoff

            log(
                "center bear: "
                f"x_base={x_base:.2f} y_base={y_base:.2f} "
                f"lat_err={lateral_error:.2f} dist_err={distance_error:.2f}"
            )

            aligned = (
                abs(lateral_error) <= args.center_lateral_tolerance
                and abs(distance_error) <= args.center_distance_tolerance
            )
            now = time.monotonic()
            if aligned:
                api.stop()
                if stable_since is None:
                    stable_since = now
                if now - stable_since >= args.center_stable_time:
                    log("center bear: aligned")
                    return
                time.sleep(args.center_poll)
                continue

            stable_since = None
            linear_x = 0.0
            if abs(distance_error) > args.center_distance_tolerance:
                linear_x = clamp(
                    args.center_linear_gain * distance_error,
                    -args.center_max_linear,
                    args.center_max_linear,
                )
            angular_z = clamp(
                args.center_angular_sign * args.center_angular_gain * lateral_error,
                -args.center_max_angular,
                args.center_max_angular,
            )
            api.cmd_vel(linear_x, angular_z)
            time.sleep(args.center_poll)
    finally:
        api.stop()
        time.sleep(0.2)

    raise TimeoutError(f"center bear timeout after {args.center_timeout:.1f}s")


def navigate_to_pose_with_retry(
    api: HostApi,
    label: str,
    x: float,
    y: float,
    yaw: float,
    args: argparse.Namespace,
    *,
    retreat_on_retry: bool = False,
) -> None:
    last_error: Exception | None = None
    attempts = max(1, args.return_retries + 1)
    for attempt in range(1, attempts + 1):
        if attempt > 1:
            log(f"{label}: retry {attempt}/{attempts}")
        try:
            clear_costmaps(api)
            require_ok(api.nav_goal(x, y, yaw), label)
            wait_nav(api, label, args.nav_timeout, args.poll)
            wait_position(api, label, x, y, args.position_tolerance, args.nav_timeout, args.poll)
            return
        except (HostApiError, TimeoutError) as exc:
            last_error = exc
            log(f"{label}: attempt {attempt} failed: {exc}")
            api.stop()
            if attempt < attempts and retreat_on_retry:
                retreat(api, f"{label} recovery", args)

    assert last_error is not None
    raise last_error


def run(args: argparse.Namespace) -> None:
    if not args.yes and not args.dry_run:
        raise SystemExit("Refusing to move robot without --yes. Use --dry-run first.")

    random.seed(args.seed)
    api = HostApi(args.host_url, timeout=args.http_timeout)

    health = api.health()
    require_ok(health, "host health")

    nav = api.nav_status()
    if not nav.get("server_ready"):
        raise HostApiError("Nav2 action server is not ready.")

    start_pose = api.pose()
    if not start_pose.get("ok"):
        raise HostApiError("ROS pose is unavailable. Start bridge/localization first.")
    start_x = float(start_pose.get("x", 0.0))
    start_y = float(start_pose.get("y", 0.0))
    start_yaw = float(start_pose.get("yaw", 0.0))
    log(f"record start pose: x={start_x:.2f} y={start_y:.2f} yaw={start_yaw:.2f}")

    deadline = time.monotonic() + args.search_timeout
    target: dict[str, Any] | None = None
    while time.monotonic() < deadline:
        target = latest_bear(api, args)
        if target is not None:
            break
        if args.random_map_search:
            try:
                target = search_random_goal_once(api, args, args.dry_run)
            except HostApiError as exc:
                log(f"random map search unavailable ({exc}); falling back to wander")
                wander_once(api, args, args.dry_run)
            if target is not None:
                break
        else:
            wander_once(api, args, args.dry_run)

    if target is None:
        raise TimeoutError(f"bear not found within {args.search_timeout:.1f}s")

    pose_now = api.pose()
    object_x = float(target["x_map"])
    object_y = float(target["y_map"])
    goal_x, goal_y, goal_yaw = pregrasp_goal(
        pose_now,
        object_x,
        object_y,
        args.standoff,
        args.target_lateral_offset,
    )
    log(
        "bear found target: "
        f"class={target.get('class_name')} conf={float(target.get('confidence', 0.0)):.2f} "
        f"map=({object_x:.2f},{object_y:.2f}) "
        f"goal=({goal_x:.2f},{goal_y:.2f},{goal_yaw:.2f})"
    )

    if not args.dry_run:
        require_ok(api.nav_goal(goal_x, goal_y, goal_yaw), "go bear pregrasp")
        wait_nav(api, "go bear pregrasp", args.nav_timeout, args.poll)
        wait_position(
            api,
            "go bear pregrasp",
            goal_x,
            goal_y,
            args.position_tolerance,
            args.nav_timeout,
            args.poll,
        )
        api.stop()
        if not args.skip_pregrasp_heading_restore:
            restore_heading(api, goal_yaw, args)
        if not args.skip_center_bear:
            center_bear(api, args)

    log(f"pause near bear for {args.found_wait:.1f}s")
    if not args.dry_run:
        time.sleep(args.found_wait)

    log(f"return to recorded start pose: x={start_x:.2f} y={start_y:.2f} yaw={start_yaw:.2f}")
    if not args.dry_run:
        if not args.skip_retreat_before_return:
            retreat(api, "return start pose precheck", args)
        navigate_to_pose_with_retry(
            api,
            "return start pose",
            start_x,
            start_y,
            start_yaw,
            args,
            retreat_on_retry=True,
        )
        api.stop()
        if not args.skip_heading_restore:
            restore_heading(api, start_yaw, args)

    log("done")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Record current pose, search bear, approach, pause, and return.")
    parser.add_argument("--host-url", default="http://127.0.0.1:8770")
    parser.add_argument("--bridge-url", default="http://127.0.0.1:8771")
    parser.add_argument("--yes", action="store_true", help="Allow robot motion.")
    parser.add_argument("--dry-run", action="store_true", help="Print the flow without sending motion commands.")
    parser.add_argument("--http-timeout", type=float, default=5.0)
    parser.add_argument("--poll", type=float, default=0.4)
    parser.add_argument("--nav-timeout", type=float, default=90.0)
    parser.add_argument(
        "--position-tolerance",
        type=float,
        default=0.20,
        help=(
            "Distance in meters that counts as physically reaching a target. "
            "Needed because Nav2 is planning-only in this robot stack."
        ),
    )

    parser.add_argument("--standoff", type=float, default=0.03)
    parser.add_argument("--target-lateral-offset", type=float, default=0.0)
    parser.add_argument("--min-confidence", type=float, default=0.25)
    parser.add_argument("--found-wait", type=float, default=10.0)
    parser.add_argument("--search-timeout", type=float, default=60.0)
    parser.add_argument("--search-poll", type=float, default=0.3)
    parser.add_argument("--return-retries", type=int, default=2)
    parser.add_argument("--retreat-linear", type=float, default=0.05)
    parser.add_argument("--retreat-seconds", type=float, default=1.5)
    parser.add_argument(
        "--skip-retreat-before-return",
        action="store_true",
        help="Do not back away from the bear before planning the return path.",
    )
    parser.add_argument(
        "--target-classes",
        nargs="+",
        default=["xiong", "xiong_qiao", "bear"],
        help="Class names accepted as the bear target.",
    )

    parser.add_argument("--wander-linear", type=float, default=0.025)
    parser.add_argument("--wander-angular", type=float, default=0.35)
    parser.add_argument("--wander-angular-min", type=float, default=0.12)
    parser.add_argument("--wander-seconds-min", type=float, default=0.8)
    parser.add_argument("--wander-seconds-max", type=float, default=1.8)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument(
        "--random-map-search",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="When no bear is visible, navigate to random free goals inside the SLAM map.",
    )
    parser.add_argument("--random-goal-clearance", type=float, default=0.28)
    parser.add_argument("--random-goal-attempts", type=int, default=300)
    parser.add_argument("--random-goal-timeout", type=float, default=3.0)

    parser.add_argument(
        "--skip-heading-restore",
        action="store_true",
        help="Skip the final in-place yaw correction after returning to the start pose.",
    )
    parser.add_argument(
        "--skip-pregrasp-heading-restore",
        action="store_true",
        help="Skip the in-place yaw correction after reaching the bear pregrasp pose.",
    )
    parser.add_argument(
        "--skip-center-bear",
        action="store_true",
        help="Skip the final slow correction that centers the bear in front of the robot.",
    )
    parser.add_argument("--heading-tolerance", type=float, default=0.08)
    parser.add_argument("--heading-timeout", type=float, default=20.0)
    parser.add_argument("--heading-poll", type=float, default=0.15)
    parser.add_argument("--heading-gain", type=float, default=0.8)
    parser.add_argument("--heading-max-angular", type=float, default=0.9)
    parser.add_argument(
        "--heading-angular-sign",
        type=float,
        choices=[-1.0, 1.0],
        default=1.0,
        help="Flip to -1 if final heading correction rotates the wrong way.",
    )
    parser.add_argument("--center-standoff", type=float, default=0.30)
    parser.add_argument("--center-lateral-offset", type=float, default=0.0)
    parser.add_argument("--center-lateral-tolerance", type=float, default=0.04)
    parser.add_argument("--center-distance-tolerance", type=float, default=0.06)
    parser.add_argument("--center-stable-time", type=float, default=0.5)
    parser.add_argument("--center-timeout", type=float, default=20.0)
    parser.add_argument("--center-poll", type=float, default=0.15)
    parser.add_argument("--center-linear-gain", type=float, default=0.35)
    parser.add_argument("--center-angular-gain", type=float, default=1.2)
    parser.add_argument("--center-max-linear", type=float, default=0.04)
    parser.add_argument("--center-max-angular", type=float, default=0.16)
    parser.add_argument(
        "--center-angular-sign",
        type=float,
        choices=[-1.0, 1.0],
        default=-1.0,
        help="Flip to 1 if bear-centering turns away from the bear.",
    )
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    try:
        run(args)
    except KeyboardInterrupt:
        print("\n[bear-sim] interrupted", file=sys.stderr)
        try:
            HostApi(args.host_url, timeout=args.http_timeout).stop()
        except Exception:
            pass
        return 130
    except Exception as exc:  # noqa: BLE001
        print(f"[bear-sim] ERROR: {exc}", file=sys.stderr)
        try:
            HostApi(args.host_url, timeout=args.http_timeout).stop()
        except Exception:
            pass
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
