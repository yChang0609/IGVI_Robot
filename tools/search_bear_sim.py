#!/usr/bin/env python3
"""Find all visible bears, visit each one, wait, and return home.

The script expects the host API to expose /api/ros/task/target. That payload
may contain either one selected detection or a "detections" list. All valid
bear detections with map coordinates are recorded before the robot starts
visiting them, so later camera visibility changes do not lose the targets.
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
    bridge_url: str = "http://127.0.0.1:8771"
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
        try:
            return self.request("GET", "/api/ros/task/target")
        except HostApiError as exc:
            if not str(exc).startswith("404:"):
                raise
            return HostApi(self.bridge_url, self.bridge_url, self.timeout).request(
                "GET",
                "/api/task/target",
            )

    def nav_status(self) -> dict[str, Any]:
        return self.request("GET", "/api/ros/nav/status")

    def nav_goal(self, x: float, y: float, yaw: float) -> dict[str, Any]:
        return self.request(
            "POST",
            "/api/ros/nav/goal",
            {"x": x, "y": y, "yaw": yaw},
            timeout=8.0,
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


def target_candidates(
    payload: dict[str, Any],
    args: argparse.Namespace,
    *,
    require_map: bool = True,
    require_base: bool = False,
) -> list[dict[str, Any]]:
    age = payload.get("age_s")
    if age is not None and float(age) > args.max_target_age:
        return []

    raw_candidates = payload.get("detections")
    if not isinstance(raw_candidates, list) or not raw_candidates:
        raw_candidates = [payload]

    candidates: list[dict[str, Any]] = []
    for item in raw_candidates:
        if not isinstance(item, dict):
            continue
        if str(item.get("class_name", "")) not in args.target_classes:
            continue
        if float(item.get("confidence", item.get("score", 0.0))) < args.min_confidence:
            continue
        if require_map and (item.get("x_map") is None or item.get("y_map") is None):
            continue
        if require_base and (item.get("x_base") is None or item.get("y_base") is None):
            continue
        candidates.append(item)
    return candidates


def dedupe_bears(candidates: list[dict[str, Any]], min_sep: float) -> list[dict[str, Any]]:
    ordered = sorted(
        candidates,
        key=lambda item: float(item.get("confidence", item.get("score", 0.0))),
        reverse=True,
    )
    unique: list[dict[str, Any]] = []
    for candidate in ordered:
        x = float(candidate["x_map"])
        y = float(candidate["y_map"])
        if any(math.hypot(x - float(old["x_map"]), y - float(old["y_map"])) < min_sep for old in unique):
            continue
        unique.append(dict(candidate))
    return unique


def dedupe_bears_by_base(candidates: list[dict[str, Any]], min_sep: float) -> list[dict[str, Any]]:
    ordered = sorted(
        candidates,
        key=lambda item: float(item.get("confidence", item.get("score", 0.0))),
        reverse=True,
    )
    unique: list[dict[str, Any]] = []
    for candidate in ordered:
        x = float(candidate["x_base"])
        y = float(candidate["y_base"])
        if any(math.hypot(x - float(old["x_base"]), y - float(old["y_base"])) < min_sep for old in unique):
            continue
        unique.append(dict(candidate))
    return unique


def add_new_bears(
    known: list[dict[str, Any]],
    candidates: list[dict[str, Any]],
    args: argparse.Namespace,
    *,
    source: str,
) -> int:
    added = 0
    for candidate in dedupe_bears(candidates, args.bear_dedupe_distance):
        x = float(candidate["x_map"])
        y = float(candidate["y_map"])
        if any(math.hypot(x - float(old["x_map"]), y - float(old["y_map"])) < args.bear_dedupe_distance for old in known):
            continue
        known.append(dict(candidate))
        added += 1
        log(
            f"recorded new bear from {source}: "
            f"class={candidate.get('class_name')} "
            f"conf={float(candidate.get('confidence', 0.0)):.2f} "
            f"map=({x:.2f},{y:.2f})"
        )
    return added


def scan_new_bears(
    api: HostApi,
    known: list[dict[str, Any]],
    args: argparse.Namespace,
    *,
    source: str,
) -> int:
    try:
        target = api.latest_target()
    except HostApiError as exc:
        log(f"scan new bears failed from {source}: {exc}")
        return 0
    if not target.get("ok"):
        return 0
    return add_new_bears(known, target_candidates(target, args), args, source=source)


def collect_all_bears(api: HostApi, args: argparse.Namespace) -> list[dict[str, Any]]:
    deadline = time.monotonic() + args.search_timeout
    last_log = 0.0
    while time.monotonic() < deadline:
        target = api.latest_target()
        if target.get("ok"):
            candidates = target_candidates(target, args)
            bears = dedupe_bears(candidates, args.bear_dedupe_distance)
            if bears:
                log(f"recorded {len(bears)} bear(s)")
                for i, bear in enumerate(bears, start=1):
                    log(
                        f"bear {i}: class={bear.get('class_name')} "
                        f"conf={float(bear.get('confidence', 0.0)):.2f} "
                        f"map=({float(bear['x_map']):.2f},{float(bear['y_map']):.2f})"
                    )
                return bears
            base_candidates = target_candidates(target, args, require_map=False, require_base=True)
            base_bears = dedupe_bears_by_base(base_candidates, args.bear_dedupe_distance)
            if base_bears:
                log(f"recorded {len(base_bears)} bear(s) from camera/base without map")
                for i, bear in enumerate(base_bears, start=1):
                    log(
                        f"bear {i}: class={bear.get('class_name')} "
                        f"conf={float(bear.get('confidence', bear.get('score', 0.0))):.2f} "
                        f"base=({float(bear['x_base']):.2f},{float(bear['y_base']):.2f})"
                    )
                return base_bears
            now = time.monotonic()
            if now - last_log >= 1.0:
                age = target.get("age_s")
                if age is not None and float(age) > args.max_target_age:
                    log(f"record: target stale age={float(age):.1f}s > max={args.max_target_age:.1f}s")
                else:
                    log("record: no usable bear target yet")
                last_log = now
        else:
            log(str(target.get("message") or "no bear target yet"))
        time.sleep(args.search_poll)
    raise TimeoutError(f"no bear targets within {args.search_timeout:.1f}s")


def refresh_bear_pose(api: HostApi, bear: dict[str, Any], args: argparse.Namespace) -> dict[str, Any]:
    try:
        target = api.latest_target()
    except HostApiError as exc:
        log(f"refresh bear pose failed: {exc}")
        return bear
    if not target.get("ok"):
        return bear

    candidates = target_candidates(target, args)
    if not candidates:
        return bear

    old_x = float(bear["x_map"])
    old_y = float(bear["y_map"])
    nearest = min(
        candidates,
        key=lambda item: math.hypot(float(item["x_map"]) - old_x, float(item["y_map"]) - old_y),
    )
    distance = math.hypot(float(nearest["x_map"]) - old_x, float(nearest["y_map"]) - old_y)
    if distance > args.bear_update_distance:
        return bear

    updated = dict(nearest)
    log(
        "refresh bear pose: "
        f"old=({old_x:.2f},{old_y:.2f}) "
        f"new=({float(updated['x_map']):.2f},{float(updated['y_map']):.2f}) "
        f"delta={distance:.2f}m"
    )
    return updated


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


def clear_costmaps(api: HostApi) -> None:
    for target in ("local", "global"):
        try:
            result = api.clear_costmap(target)
        except HostApiError as exc:
            log(f"clear {target} costmap failed: {exc}")
            continue
        log(f"clear {target} costmap: {result.get('message', result)}")


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
    known_bears: list[dict[str, Any]] | None = None,
    args: argparse.Namespace | None = None,
) -> None:
    deadline = time.monotonic() + timeout_s
    last_log = 0.0
    while time.monotonic() < deadline:
        if known_bears is not None and args is not None:
            scan_new_bears(api, known_bears, args, source=label)
        pose = api.pose()
        if not pose.get("ok"):
            raise HostApiError(f"{label}: ROS pose unavailable while waiting for position")
        distance = math.hypot(
            target_x - float(pose.get("x", 0.0)),
            target_y - float(pose.get("y", 0.0)),
        )
        now = time.monotonic()
        if now - last_log >= 1.0:
            log(f"{label}: distance={distance:.2f}m")
            last_log = now
        if distance <= tolerance:
            log(f"{label}: position reached within {tolerance:.2f}m")
            return
        time.sleep(poll_s)
    raise TimeoutError(f"{label}: position timeout after {timeout_s:.1f}s")


def navigate_to_pose(
    api: HostApi,
    label: str,
    x: float,
    y: float,
    yaw: float,
    args: argparse.Namespace,
    known_bears: list[dict[str, Any]] | None = None,
) -> None:
    clear_costmaps(api)
    require_ok(api.nav_goal(x, y, yaw), label)
    wait_nav(api, label, args.nav_timeout, args.poll)
    wait_position(
        api,
        label,
        x,
        y,
        args.position_tolerance,
        args.nav_timeout,
        args.poll,
        known_bears,
        args,
    )


def navigate_to_pose_with_retry(
    api: HostApi,
    label: str,
    x: float,
    y: float,
    yaw: float,
    args: argparse.Namespace,
    *,
    retreat_on_retry: bool = False,
    known_bears: list[dict[str, Any]] | None = None,
) -> None:
    last_error: Exception | None = None
    attempts = max(1, args.return_retries + 1)
    for attempt in range(1, attempts + 1):
        if attempt > 1:
            log(f"{label}: retry {attempt}/{attempts}")
        try:
            navigate_to_pose(api, label, x, y, yaw, args, known_bears)
            return
        except (HostApiError, TimeoutError) as exc:
            last_error = exc
            log(f"{label}: attempt {attempt} failed: {exc}")
            api.stop()
            if attempt < attempts and retreat_on_retry:
                retreat(api, f"{label} recovery", args)
    assert last_error is not None
    raise last_error


def restore_heading(api: HostApi, target_yaw: float, args: argparse.Namespace) -> None:
    log(f"restore heading: target yaw={target_yaw:.2f}")
    deadline = time.monotonic() + args.heading_timeout
    try:
        while time.monotonic() < deadline:
            pose = api.pose()
            if not pose.get("ok"):
                raise HostApiError("ROS pose unavailable while restoring heading")
            current_yaw = float(pose.get("yaw", 0.0))
            error = wrap_angle(target_yaw - current_yaw)
            log(f"restore heading: current={current_yaw:.2f} error={error:.2f}")
            if abs(error) <= args.heading_tolerance:
                log("restore heading: aligned")
                return
            angular_z = clamp(
                args.heading_angular_sign * args.heading_gain * error,
                -args.heading_max_angular,
                args.heading_max_angular,
            )
            api.cmd_vel(0.0, angular_z)
            time.sleep(args.heading_poll)
    finally:
        api.stop()
        time.sleep(0.2)
    raise TimeoutError(f"restore heading timeout after {args.heading_timeout:.1f}s")


def drive_to_pose_direct(
    api: HostApi,
    label: str,
    target_x: float,
    target_y: float,
    args: argparse.Namespace,
) -> None:
    log(f"{label}: direct drive fallback to x={target_x:.2f} y={target_y:.2f}")
    deadline = time.monotonic() + args.direct_return_timeout
    last_log = 0.0
    try:
        while time.monotonic() < deadline:
            pose = api.pose()
            if not pose.get("ok"):
                raise HostApiError(f"{label}: ROS pose unavailable while direct driving")

            current_x = float(pose.get("x", 0.0))
            current_y = float(pose.get("y", 0.0))
            current_yaw = float(pose.get("yaw", 0.0))
            dx = target_x - current_x
            dy = target_y - current_y
            distance = math.hypot(dx, dy)
            now = time.monotonic()
            if now - last_log >= 1.0:
                log(f"{label}: direct distance={distance:.2f}m")
                last_log = now
            if distance <= args.direct_return_tolerance:
                log(f"{label}: direct position reached within {args.direct_return_tolerance:.2f}m")
                return

            target_heading = math.atan2(dy, dx)
            heading_error = wrap_angle(target_heading - current_yaw)
            linear_x = 0.0
            if abs(heading_error) <= args.direct_return_turn_in_place_angle:
                linear_x = args.direct_return_linear_gain * distance
                if args.direct_return_max_linear > 0.0:
                    linear_x = clamp(linear_x, 0.0, args.direct_return_max_linear)
            angular_z = args.heading_angular_sign * args.direct_return_angular_gain * heading_error
            if args.direct_return_max_angular > 0.0:
                angular_z = clamp(
                    angular_z,
                    -args.direct_return_max_angular,
                    args.direct_return_max_angular,
                )
            api.cmd_vel(linear_x, angular_z)
            time.sleep(args.direct_return_poll)
    finally:
        api.stop()
        time.sleep(0.2)
    raise TimeoutError(f"{label}: direct drive timeout after {args.direct_return_timeout:.1f}s")


def return_home(api: HostApi, index: int, start: dict[str, float], args: argparse.Namespace) -> None:
    label = f"return home from bear {index}"
    try:
        navigate_to_pose_with_retry(
            api,
            label,
            float(start["x"]),
            float(start["y"]),
            float(start["yaw"]),
            args,
            retreat_on_retry=True,
            known_bears=None,
        )
    except (HostApiError, TimeoutError) as exc:
        if not args.direct_return_on_nav_fail:
            raise
        log(f"{label}: Nav2 failed, direct return fallback: {exc}")
        drive_to_pose_direct(api, label, float(start["x"]), float(start["y"]), args)

    api.stop()
    if not args.skip_heading_restore:
        restore_heading(api, float(start["yaw"]), args)


def center_bear(api: HostApi, args: argparse.Namespace) -> None:
    if args.skip_center_bear:
        return
    log("center bear: start")
    deadline = time.monotonic() + args.center_timeout
    stable_since: float | None = None
    lateral_stable_since: float | None = None
    forward_drive_time = 0.0
    last_motion_time = time.monotonic()
    last_no_candidate_log = 0.0
    try:
        while time.monotonic() < deadline:
            target = api.latest_target()
            candidates = (
                target_candidates(target, args, require_map=False, require_base=True)
                if target.get("ok")
                else []
            )
            if not candidates:
                api.stop()
                stable_since = None
                now = time.monotonic()
                if now - last_no_candidate_log >= 1.0:
                    age = target.get("age_s")
                    if age is not None and float(age) > args.max_target_age:
                        log(
                            f"center bear: target stale age={float(age):.1f}s "
                            f"> max={args.max_target_age:.1f}s"
                        )
                    elif target.get("ok"):
                        log("center bear: no fresh target with x_base/y_base")
                    else:
                        log(f"center bear: {target.get('message') or 'no target'}")
                    last_no_candidate_log = now
                time.sleep(args.center_poll)
                continue
            bear = max(candidates, key=lambda item: float(item.get("confidence", 0.0)))
            x_base = float(bear.get("x_base", 0.0))
            y_base = float(bear.get("y_base", 0.0))
            depth_m = bear.get("depth_m")
            lateral_error = y_base - args.center_lateral_offset
            distance_error = x_base - args.center_standoff
            log(
                f"center bear: x_base={x_base:.2f} y_base={y_base:.2f} "
                f"lat_err={lateral_error:.2f} dist_err={distance_error:.2f}"
            )

            aligned = (
                abs(lateral_error) <= args.center_lateral_tolerance
                and abs(distance_error) <= args.center_distance_tolerance
            )
            now = time.monotonic()
            if (
                args.center_touch_depth > 0.0
                and depth_m is not None
                and float(depth_m) <= args.center_touch_depth
            ):
                api.stop()
                log(f"touch bear (depth fallback, depth={float(depth_m):.2f}m)")
                return
            lateral_aligned = abs(lateral_error) <= args.center_lateral_tolerance
            if lateral_aligned:
                if lateral_stable_since is None:
                    lateral_stable_since = now
            else:
                lateral_stable_since = None
            if aligned:
                api.stop()
                if stable_since is None:
                    stable_since = now
                if now - stable_since >= args.center_stable_time:
                    log("touch bear")
                    return
                time.sleep(args.center_poll)
                continue
            if (
                args.center_touch_after_forward_time > 0.0
                and (
                    args.center_touch_forward_max_x <= 0.0
                    or x_base <= args.center_touch_forward_max_x
                )
                and lateral_stable_since is not None
                and now - lateral_stable_since >= args.center_stable_time
                and forward_drive_time >= args.center_touch_after_forward_time
            ):
                api.stop()
                log(
                    "touch bear "
                    f"(center fallback, x_base={x_base:.2f}, "
                    f"forward_time={forward_drive_time:.1f}s)"
                )
                return

            stable_since = None
            linear_x = 0.0
            if abs(distance_error) > args.center_distance_tolerance:
                linear_x = args.center_linear_gain * distance_error
                if args.center_max_linear > 0.0:
                    linear_x = clamp(
                        linear_x,
                        -args.center_max_linear,
                        args.center_max_linear,
                    )
            angular_z = clamp(
                args.center_angular_sign * args.center_angular_gain * lateral_error,
                -args.center_max_angular,
                args.center_max_angular,
            )
            motion_now = time.monotonic()
            if linear_x > 0.0:
                forward_drive_time += min(motion_now - last_motion_time, args.center_poll)
            last_motion_time = motion_now
            api.cmd_vel(linear_x, angular_z)
            time.sleep(args.center_poll)
    finally:
        api.stop()
        time.sleep(0.2)
    log(f"center bear: timeout after {args.center_timeout:.1f}s; continue with current pose")


def retreat(api: HostApi, label: str, args: argparse.Namespace) -> None:
    if args.retreat_seconds <= 0:
        return
    log(f"{label}: reverse {args.retreat_seconds:.1f}s at {-abs(args.retreat_linear):.2f}m/s")
    end = time.monotonic() + args.retreat_seconds
    try:
        while time.monotonic() < end:
            api.cmd_vel(-abs(args.retreat_linear), 0.0)
            time.sleep(0.1)
    finally:
        api.stop()
        time.sleep(0.3)


def visit_bear(
    api: HostApi,
    index: int,
    bear: dict[str, Any],
    start: dict[str, float],
    args: argparse.Namespace,
    known_bears: list[dict[str, Any]],
) -> None:
    if bear.get("x_map") is not None and bear.get("y_map") is not None:
        bear = refresh_bear_pose(api, bear, args)
    if (
        args.prefer_visual_approach
        and bear.get("x_base") is not None
        and 0.0 < float(bear.get("x_base", 0.0)) <= args.visual_approach_range
    ):
        log(
            f"visit bear {index}: visible in camera, using slow visual approach "
            f"base=({float(bear.get('x_base', 0.0)):.2f},{float(bear.get('y_base', 0.0)):.2f})"
        )
        center_bear(api, args)
        log(f"wait in front of bear {index}: {args.found_wait:.1f}s")
        wait_until = time.monotonic() + args.found_wait
        while time.monotonic() < wait_until:
            time.sleep(args.search_poll)
        if not args.skip_retreat_before_return:
            retreat(api, f"return from bear {index}", args)
        return_home(api, index, start, args)
        return

    if bear.get("x_map") is None or bear.get("y_map") is None:
        raise HostApiError("bear has no map pose and is not close enough for visual approach")
    object_x = float(bear["x_map"])
    object_y = float(bear["y_map"])
    pose = api.pose()
    goal_x, goal_y, goal_yaw = pregrasp_goal(
        pose,
        object_x,
        object_y,
        args.standoff,
        args.target_lateral_offset,
    )
    log(
        f"visit bear {index}: object=({object_x:.2f},{object_y:.2f}) "
        f"goal=({goal_x:.2f},{goal_y:.2f},{goal_yaw:.2f})"
    )
    navigate_to_pose_with_retry(
        api,
        f"go bear {index}",
        goal_x,
        goal_y,
        goal_yaw,
        args,
        known_bears=None,
    )
    api.stop()
    if not args.skip_pregrasp_heading_restore:
        restore_heading(api, goal_yaw, args)
    center_bear(api, args)

    log(f"wait in front of bear {index}: {args.found_wait:.1f}s")
    wait_until = time.monotonic() + args.found_wait
    while time.monotonic() < wait_until:
        time.sleep(args.search_poll)

    if not args.skip_retreat_before_return:
        retreat(api, f"return from bear {index}", args)
    return_home(api, index, start, args)


def run(args: argparse.Namespace) -> None:
    if not args.yes and not args.dry_run:
        raise SystemExit("Refusing to move robot without --yes. Use --dry-run first.")

    random.seed(args.seed)
    api = HostApi(args.host_url, bridge_url=args.bridge_url, timeout=args.http_timeout)
    require_ok(api.health(), "host health")
    if not api.nav_status().get("server_ready"):
        raise HostApiError("Nav2 action server is not ready")

    start_pose = api.pose()
    if not start_pose.get("ok"):
        raise HostApiError("ROS pose unavailable. Start bridge/localization first.")
    start = {
        "x": float(start_pose.get("x", 0.0)),
        "y": float(start_pose.get("y", 0.0)),
        "yaw": float(start_pose.get("yaw", 0.0)),
    }
    log(f"record start pose: x={start['x']:.2f} y={start['y']:.2f} yaw={start['yaw']:.2f}")

    bears = collect_all_bears(api, args)
    if args.dry_run:
        log("dry run complete after recording bears")
        return

    index = 0
    while index < len(bears):
        bear = bears[index]
        visit_bear(api, index + 1, bear, start, args, bears)
        index += 1

    log("done")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Record all visible bears, visit each, wait 5s, and return home.")
    parser.add_argument("--host-url", default="http://127.0.0.1:8770")
    parser.add_argument("--bridge-url", default="http://127.0.0.1:8771")
    parser.add_argument("--yes", action="store_true", help="Allow robot motion.")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--http-timeout", type=float, default=5.0)
    parser.add_argument("--poll", type=float, default=0.4)
    parser.add_argument("--nav-timeout", type=float, default=90.0)
    parser.add_argument("--position-tolerance", type=float, default=0.20)

    parser.add_argument("--standoff", type=float, default=0.15)
    parser.add_argument(
        "--prefer-visual-approach",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Use slow x_base/y_base visual servo instead of Nav2 when the bear is already visible.",
    )
    parser.add_argument("--visual-approach-range", type=float, default=1.5)
    parser.add_argument("--target-lateral-offset", type=float, default=0.0)
    parser.add_argument("--min-confidence", type=float, default=0.0)
    parser.add_argument("--found-wait", type=float, default=5.0)
    parser.add_argument("--search-timeout", type=float, default=10.0)
    parser.add_argument("--search-poll", type=float, default=0.3)
    parser.add_argument("--max-target-age", type=float, default=6.0)
    parser.add_argument("--bear-dedupe-distance", type=float, default=0.25)
    parser.add_argument("--bear-update-distance", type=float, default=0.60)
    parser.add_argument(
        "--target-classes",
        nargs="+",
        default=["xiong", "xiong_qiao", "bear"],
    )

    parser.add_argument("--return-retries", type=int, default=2)
    parser.add_argument("--direct-return-on-nav-fail", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--direct-return-tolerance", type=float, default=0.08)
    parser.add_argument("--direct-return-timeout", type=float, default=45.0)
    parser.add_argument("--direct-return-poll", type=float, default=0.1)
    parser.add_argument("--direct-return-linear-gain", type=float, default=0.4)
    parser.add_argument("--direct-return-angular-gain", type=float, default=1.2)
    parser.add_argument(
        "--direct-return-max-linear",
        type=float,
        default=0.0,
        help="Maximum direct-return linear speed. 0 means no script clamp; motion_arbiter still applies robot limits.",
    )
    parser.add_argument(
        "--direct-return-max-angular",
        type=float,
        default=0.0,
        help="Maximum direct-return angular speed. 0 means no script clamp; motion_arbiter still applies robot limits.",
    )
    parser.add_argument("--direct-return-turn-in-place-angle", type=float, default=0.45)
    parser.add_argument("--retreat-linear", type=float, default=0.05)
    parser.add_argument("--retreat-seconds", type=float, default=1.5)
    parser.add_argument("--skip-retreat-before-return", action="store_true")

    parser.add_argument("--skip-heading-restore", action="store_true")
    parser.add_argument("--skip-pregrasp-heading-restore", action="store_true")
    parser.add_argument("--heading-tolerance", type=float, default=0.05)
    parser.add_argument("--heading-timeout", type=float, default=15.0)
    parser.add_argument("--heading-poll", type=float, default=0.15)
    parser.add_argument("--heading-gain", type=float, default=0.8)
    parser.add_argument("--heading-max-angular", type=float, default=0.25)
    parser.add_argument("--heading-angular-sign", type=float, choices=[-1.0, 1.0], default=1.0)

    parser.add_argument("--skip-center-bear", action="store_true")
    parser.add_argument("--center-standoff", type=float, default=0.15)
    parser.add_argument("--center-lateral-offset", type=float, default=0.0)
    parser.add_argument("--center-lateral-tolerance", type=float, default=0.04)
    parser.add_argument("--center-distance-tolerance", type=float, default=0.03)
    parser.add_argument("--center-stable-time", type=float, default=0.5)
    parser.add_argument(
        "--center-touch-depth",
        type=float,
        default=0.2,
        help="Stop as touch when detection depth is this close. 0 disables.",
    )
    parser.add_argument(
        "--center-touch-after-forward-time",
        type=float,
        default=1.5,
        help="If lateral centering is stable and distance looks unreliable, stop after this much forward drive time. 0 disables.",
    )
    parser.add_argument(
        "--center-touch-forward-max-x",
        type=float,
        default=0.35,
        help="Only allow forward-time touch fallback when x_base is this close. 0 disables this guard.",
    )
    parser.add_argument("--center-timeout", type=float, default=12.0)
    parser.add_argument("--center-poll", type=float, default=0.15)
    parser.add_argument("--center-linear-gain", type=float, default=0.35)
    parser.add_argument("--center-angular-gain", type=float, default=1.2)
    parser.add_argument(
        "--center-max-linear",
        type=float,
        default=0.0,
        help="Maximum visual-approach linear speed. 0 means no script clamp; motion_arbiter still applies robot limits.",
    )
    parser.add_argument("--center-max-angular", type=float, default=0.16)
    parser.add_argument("--center-angular-sign", type=float, choices=[-1.0, 1.0], default=1.0)
    parser.add_argument("--seed", type=int, default=None)
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    try:
        run(args)
    except KeyboardInterrupt:
        print("\n[bear-sim] interrupted", file=sys.stderr)
        try:
            HostApi(args.host_url, bridge_url=args.bridge_url, timeout=args.http_timeout).stop()
        except Exception:
            pass
        return 130
    except Exception as exc:  # noqa: BLE001
        print(f"[bear-sim] ERROR: {exc}", file=sys.stderr)
        try:
            HostApi(args.host_url, bridge_url=args.bridge_url, timeout=args.http_timeout).stop()
        except Exception:
            pass
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
