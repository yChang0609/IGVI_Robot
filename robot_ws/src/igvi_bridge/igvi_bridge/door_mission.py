from __future__ import annotations

import threading
import time
from typing import Any, Callable


Waypoint = dict[str, float]

# How long to wait, after Nav2 reports SUCCEEDED, for motion_arbiter to enter
# PATH_TRACKING before we assume the robot was already at the goal and skip
# straight to opening. The plan-only Nav2 stack returns SUCCESS as soon as the
# path is computed; the actual drive happens asynchronously in motion_arbiter.
_NO_TRACKING_GRACE_SEC = 2.5

# Hard cap on how long the driving phase may take before we give up.
_DRIVING_TIMEOUT_SEC = 60.0


class DoorMissionCoordinator:
    """Coordinates the door task while keeping bridge_node as a transport layer.

    The task remains the same public behavior as before: navigate to a named
    waypoint, then run the open_door action when navigation succeeds.

    The "navigating" phase covers Nav2 planning; once Nav2 reports SUCCEEDED
    (which in the plan-only stack means "path computed", not "drive complete"),
    we enter "driving" and wait for motion_arbiter to finish executing before
    starting open_door.
    """

    def __init__(
        self,
        *,
        get_waypoint: Callable[[str], Waypoint | None],
        send_nav_goal: Callable[[float, float, float], tuple[bool, str, Any]],
        cancel_nav_goal: Callable[[], tuple[bool, str]],
        snapshot_nav: Callable[[], dict[str, Any]],
        send_open_door_goal: Callable[[float], tuple[bool, str]],
        cancel_open_door_goal: Callable[[], tuple[bool, str]],
        snapshot_open_door: Callable[[], dict[str, Any]],
    ) -> None:
        self._get_waypoint = get_waypoint
        self._send_nav_goal = send_nav_goal
        self._cancel_nav_goal = cancel_nav_goal
        self._snapshot_nav = snapshot_nav
        self._send_open_door_goal = send_open_door_goal
        self._cancel_open_door_goal = cancel_open_door_goal
        self._snapshot_open_door = snapshot_open_door

        self._lock = threading.Lock()
        self._state: dict[str, Any] = {
            "active": False,
            "phase": "idle",  # idle|navigating|driving|opening|succeeded|failed|canceled
            "waypoint": "",
            "ready_distance_m": 0.0,
            "nav_goal_token": None,
            "message": "",
            # Driving-phase bookkeeping (see on_motion_state).
            "driving_started_monotonic": 0.0,
            "saw_path_tracking": False,
        }

    def start(
        self, waypoint: str = "door_approach", ready_distance_m: float = 0.0
    ) -> tuple[bool, str]:
        wp_name = (waypoint or "door_approach").strip()
        with self._lock:
            if self._state["active"]:
                return False, f"door mission already active (phase={self._state['phase']})"

        wp = self._get_waypoint(wp_name)
        if wp is None:
            return False, f"no waypoint named '{wp_name}'"

        # Stage before dispatch so a fast nav result observes the mission.
        with self._lock:
            self._state.update(
                active=True,
                phase="navigating",
                waypoint=wp_name,
                ready_distance_m=float(ready_distance_m),
                nav_goal_token=None,
                message=f"navigating to '{wp_name}'",
            )

        ok, msg, nav_goal_token = self._send_nav_goal(wp["x"], wp["y"], wp["yaw"])
        if not ok:
            with self._lock:
                self._state.update(
                    active=False,
                    phase="failed",
                    nav_goal_token=None,
                    message=f"nav dispatch failed: {msg}",
                )
            return False, f"door mission failed to start nav: {msg}"
        with self._lock:
            self._state["nav_goal_token"] = nav_goal_token
        return True, f"door mission started: navigating to '{wp_name}'"

    def cancel(self) -> tuple[bool, str]:
        with self._lock:
            if not self._state["active"]:
                return False, "no active door mission"
            phase = self._state["phase"]

        if phase == "navigating":
            _ok, msg = self._cancel_nav_goal()
        elif phase == "driving":
            # Nav2 already finished planning; the drive lives in motion_arbiter.
            # Cancel by canceling the (now-completed) nav goal AND publishing a
            # zero twist so the arbiter exits PATH_TRACKING — but that's the
            # bridge's job. Here we just mark the mission inactive; bridge has
            # its own /motion/cmd publisher for stop.
            _ok, msg = self._cancel_nav_goal()
        elif phase == "opening":
            _ok, msg = self._cancel_open_door_goal()
        else:
            msg = f"nothing to cancel in phase '{phase}'"

        with self._lock:
            self._state.update(
                active=False,
                phase="canceled",
                nav_goal_token=None,
                message=f"canceled in phase '{phase}': {msg}",
            )
        return True, f"door mission canceled in phase '{phase}'"

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            mission = dict(self._state)
        mission.pop("nav_goal_token", None)
        mission.pop("driving_started_monotonic", None)
        mission.pop("saw_path_tracking", None)
        mission["nav"] = self._snapshot_nav()
        mission["open_door"] = self._snapshot_open_door()
        return mission

    def on_nav_result(self, nav_goal_token: Any, nav_state: str, nav_message: str) -> None:
        with self._lock:
            if not self._state["active"] or self._state["phase"] != "navigating":
                return
            if self._state.get("nav_goal_token") != nav_goal_token:
                return
            wp = self._state["waypoint"]

        if nav_state == "succeeded":
            # The plan-only Nav2 stack returns SUCCEEDED as soon as the path is
            # computed — the actual drive runs asynchronously in motion_arbiter.
            # Transition to "driving" and let on_motion_state() advance to
            # "opening" once the robot finishes executing the path.
            with self._lock:
                self._state.update(
                    phase="driving",
                    message=f"plan to '{wp}' computed; waiting for drive",
                    driving_started_monotonic=time.monotonic(),
                    saw_path_tracking=False,
                )
        elif nav_state == "canceled":
            with self._lock:
                self._state.update(
                    active=False,
                    phase="canceled",
                    nav_goal_token=None,
                    message=f"nav canceled before reaching '{wp}'",
                )
        else:
            with self._lock:
                self._state.update(
                    active=False,
                    phase="failed",
                    nav_goal_token=None,
                    message=f"nav {nav_state} before reaching '{wp}': {nav_message}",
                )

    def on_motion_state(self, motion_state: str) -> None:
        """Advance the driving phase based on motion_arbiter's /motion/state.

        Called for every /motion/state message. The first time we see
        path_tracking we record that the drive started; once we see idle
        afterwards, the drive is complete and we fire open_door. If we never
        see path_tracking within _NO_TRACKING_GRACE_SEC, we assume the robot
        was already on goal and proceed anyway.
        """
        state = (motion_state or "").strip().lower()
        with self._lock:
            if not self._state["active"] or self._state["phase"] != "driving":
                return
            ready = float(self._state["ready_distance_m"])
            wp = self._state["waypoint"]
            saw_tracking = bool(self._state["saw_path_tracking"])
            started = float(self._state["driving_started_monotonic"])
            elapsed = time.monotonic() - started

            if state == "path_tracking":
                self._state["saw_path_tracking"] = True
                return

            if state != "idle":
                # override / manual / other — not a completion signal.
                if elapsed > _DRIVING_TIMEOUT_SEC:
                    self._state.update(
                        active=False,
                        phase="failed",
                        nav_goal_token=None,
                        message=f"driving to '{wp}' timed out after {elapsed:.1f}s",
                    )
                return

            # state == "idle"
            drive_complete = saw_tracking
            already_on_goal = (not saw_tracking) and elapsed >= _NO_TRACKING_GRACE_SEC
            if not (drive_complete or already_on_goal):
                if elapsed > _DRIVING_TIMEOUT_SEC:
                    self._state.update(
                        active=False,
                        phase="failed",
                        nav_goal_token=None,
                        message=f"driving to '{wp}' timed out after {elapsed:.1f}s",
                    )
                return

            reason = "drive complete" if drive_complete else "already on goal"

        ok, send_msg = self._send_open_door_goal(ready)
        if not ok:
            with self._lock:
                self._state.update(
                    active=False,
                    phase="failed",
                    nav_goal_token=None,
                    message=f"reached '{wp}' ({reason}) but open_door failed to start: {send_msg}",
                )
            return
        with self._lock:
            self._state.update(
                phase="opening",
                message=f"reached '{wp}' ({reason}); running open_door (ready={ready:.2f}m)",
            )

    def on_open_door_result(self, door_state: str, door_message: str) -> None:
        with self._lock:
            if not self._state["active"] or self._state["phase"] != "opening":
                return
            wp = self._state["waypoint"]

        if door_state == "succeeded":
            phase = "succeeded"
            message = f"door mission complete (via '{wp}'): {door_message}"
        elif door_state == "canceled":
            phase = "canceled"
            message = f"open_door canceled at '{wp}'"
        else:
            phase = "failed"
            message = f"open_door {door_state} at '{wp}': {door_message}"

        with self._lock:
            self._state.update(active=False, phase=phase, nav_goal_token=None, message=message)
