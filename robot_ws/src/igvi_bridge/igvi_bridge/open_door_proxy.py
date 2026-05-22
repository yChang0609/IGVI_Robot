from __future__ import annotations

import threading
from typing import Any

from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient

try:
    from wildbot_grasp.action import OpenDoor  # type: ignore

    _OPEN_DOOR_AVAILABLE = True
except ImportError:  # pragma: no cover
    OpenDoor = None  # type: ignore
    _OPEN_DOOR_AVAILABLE = False


class OpenDoorProxy:
    def __init__(self, node: Any) -> None:
        self._node = node
        self._lock = threading.Lock()
        self._state: str = "idle" if _OPEN_DOOR_AVAILABLE else "unavailable"
        self._stage: str = ""
        self._message: str = (
            "" if _OPEN_DOOR_AVAILABLE else "wildbot_grasp not installed in bridge image"
        )
        self._progress: float = 0.0
        self._goal_handle = None
        self._client = ActionClient(node, OpenDoor, "open_door") if _OPEN_DOOR_AVAILABLE else None

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "available": _OPEN_DOOR_AVAILABLE,
                "state": self._state,
                "stage": self._stage,
                "message": self._message,
                "progress": self._progress,
            }

    def start(self, ready_distance_m: float = 0.0) -> tuple[bool, str]:
        if not _OPEN_DOOR_AVAILABLE or self._client is None:
            return False, "wildbot_grasp action types not installed in bridge image"
        client = self._client
        if not client.server_is_ready():
            if not client.wait_for_server(timeout_sec=2.0):
                self._set_state(
                    "unavailable", "", "open_door action server not running — check wildbot_grasp"
                )
                return False, "open_door action server not running"

        goal_msg = OpenDoor.Goal()
        goal_msg.ready_distance_m = float(ready_distance_m)

        self._set_state("sending", "", "goal dispatched", progress=0.0)
        future = client.send_goal_async(goal_msg, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_response)
        return True, f"open_door dispatched (ready_distance_m={ready_distance_m})"

    def cancel(self) -> tuple[bool, str]:
        with self._lock:
            handle = self._goal_handle
        if handle is None:
            return False, "no active open_door goal"
        handle.cancel_goal_async()
        self._set_state("cancelling", "", "cancel requested")
        return True, "cancel requested"

    def _on_feedback(self, msg: Any) -> None:
        fb = getattr(msg, "feedback", None)
        if fb is None:
            return
        self._set_state(
            "running",
            stage=str(getattr(fb, "stage", "")),
            message=str(getattr(fb, "detail", "")),
            progress=float(getattr(fb, "progress", 0.0)),
        )

    def _on_goal_response(self, future: Any) -> None:
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self._set_state("error", "", f"send_goal failed: {exc}")
            return
        if not handle.accepted:
            self._set_state("rejected", "", "goal rejected by server")
            return
        with self._lock:
            self._goal_handle = handle
        self._set_state("running", "", "goal accepted")
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future: Any) -> None:
        try:
            wrapped = future.result()
            result = wrapped.result
            status = wrapped.status
        except Exception as exc:  # noqa: BLE001
            self._set_state("error", "", f"result fetch failed: {exc}")
            return
        with self._lock:
            self._goal_handle = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._set_state(
                "succeeded",
                "complete",
                str(getattr(result, "message", "")) or "door opened",
                progress=1.0,
            )
        elif status == GoalStatus.STATUS_CANCELED:
            self._set_state("canceled", "", "goal canceled")
        else:
            self._set_state(
                "aborted",
                "",
                str(getattr(result, "message", "")) or f"status={status}",
            )

    def _set_state(
        self,
        state: str,
        stage: str = "",
        message: str = "",
        progress: float | None = None,
    ) -> None:
        with self._lock:
            self._state = state
            if stage:
                self._stage = stage
            self._message = message
            if progress is not None:
                self._progress = float(progress)
