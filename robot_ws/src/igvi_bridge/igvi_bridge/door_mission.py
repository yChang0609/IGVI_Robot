from __future__ import annotations

import threading
from typing import Any, Callable


Waypoint = dict[str, float]


class DoorMissionCoordinator:
    """Coordinates the door task while keeping bridge_node as a transport layer.

    The task remains the same public behavior as before: navigate to a named
    waypoint, then run the open_door action when navigation succeeds.
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
            "phase": "idle",  # idle|navigating|opening|succeeded|failed|canceled
            "waypoint": "",
            "ready_distance_m": 0.0,
            "nav_goal_token": None,
            "message": "",
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
        mission["nav"] = self._snapshot_nav()
        mission["open_door"] = self._snapshot_open_door()
        return mission

    def on_nav_result(self, nav_goal_token: Any, nav_state: str, nav_message: str) -> None:
        with self._lock:
            if not self._state["active"] or self._state["phase"] != "navigating":
                return
            if self._state.get("nav_goal_token") != nav_goal_token:
                return
            ready = float(self._state["ready_distance_m"])
            wp = self._state["waypoint"]

        if nav_state == "succeeded":
            ok, send_msg = self._send_open_door_goal(ready)
            if not ok:
                with self._lock:
                    self._state.update(
                        active=False,
                        phase="failed",
                        nav_goal_token=None,
                        message=f"reached '{wp}' but open_door failed to start: {send_msg}",
                    )
                return
            with self._lock:
                self._state.update(
                    phase="opening",
                    message=f"reached '{wp}'; running open_door (ready={ready:.2f}m)",
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
