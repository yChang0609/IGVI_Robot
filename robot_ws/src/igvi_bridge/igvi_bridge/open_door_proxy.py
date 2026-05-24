from __future__ import annotations

import threading
from typing import Any, Callable

from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from std_msgs.msg import Empty
from std_srvs.srv import Trigger

try:
    from wildbot_grasp.action import OpenDoor  # type: ignore
    _OPEN_DOOR_AVAILABLE = True
except ImportError:  # pragma: no cover
    OpenDoor = None  # type: ignore
    _OPEN_DOOR_AVAILABLE = False


class OpenDoorProxy:
    """Bridge-side adapter for the open_door action and debug services."""

    def __init__(
        self,
        node: Any,
        *,
        on_result: Callable[[str, str], None],
    ) -> None:
        self._node = node
        self._on_result = on_result
        self._lock = threading.Lock()
        self._state: str = "idle" if _OPEN_DOOR_AVAILABLE else "unavailable"
        self._stage: str = ""
        self._message: str = (
            "" if _OPEN_DOOR_AVAILABLE else "wildbot_grasp not installed in bridge image"
        )
        self._progress: float = 0.0
        self._goal_handle = None
        self._client = ActionClient(node, OpenDoor, "open_door") if _OPEN_DOOR_AVAILABLE else None
        self._clear_path_pub = node.create_publisher(Empty, "/motion/clear_path", 10)

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "available": _OPEN_DOOR_AVAILABLE,
                "state": self._state,
                "stage": self._stage,
                "message": self._message,
                "progress": self._progress,
            }

    def send_goal(self, ready_distance_m: float = 0.0) -> tuple[bool, str]:
        if not _OPEN_DOOR_AVAILABLE or self._client is None:
            return False, "wildbot_grasp action types not installed in bridge image"
        client = self._client
        if not client.server_is_ready():
            if not client.wait_for_server(timeout_sec=2.0):
                self._set_state(
                    "unavailable", "", "open_door action server not running — check wildbot_grasp"
                )
                return False, "open_door action server not running"

        # Drop stale path tracking before open_door takes manual motion control.
        self._clear_path_pub.publish(Empty())

        goal_msg = OpenDoor.Goal()
        goal_msg.ready_distance_m = float(ready_distance_m)

        self._set_state("sending", "", "goal dispatched", progress=0.0)
        future = client.send_goal_async(goal_msg, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_response)
        return True, f"open_door dispatched (ready_distance_m={ready_distance_m})"

    def cancel_goal(self) -> tuple[bool, str]:
        with self._lock:
            handle = self._goal_handle
        if handle is None:
            return False, "no active open_door goal"
        handle.cancel_goal_async()
        self._set_state("cancelling", "", "cancel requested")
        return True, "cancel requested"

    def save_poses(self) -> tuple[bool, str]:
        return self._call_trigger("/open_door_server/save_poses", "save_poses", timeout=4.0)

    def call_step(self, step: str) -> tuple[bool, str]:
        allowed = {"run_press", "run_push", "go_home"}
        if step not in allowed:
            return False, f"unknown open_door step '{step}'"
        return self._call_trigger(f"/open_door_server/{step}", step, timeout=30.0)

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
        handle.get_result_async().add_done_callback(self._on_result_future)

    def _on_result_future(self, future: Any) -> None:
        try:
            wrapped = future.result()
            result = wrapped.result
            status = wrapped.status
        except Exception as exc:  # noqa: BLE001
            self._set_state("error", "", f"result fetch failed: {exc}")
            return
        with self._lock:
            self._goal_handle = None
        result_msg = str(getattr(result, "message", ""))
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._set_state("succeeded", "complete", result_msg or "door opened", progress=1.0)
            door_state = "succeeded"
        elif status == GoalStatus.STATUS_CANCELED:
            self._set_state("canceled", "", "goal canceled")
            door_state = "canceled"
        else:
            self._set_state("aborted", "", result_msg or f"status={status}")
            door_state = "aborted"
        self._on_result(door_state, result_msg)

    def _call_trigger(self, service_name: str, label: str, timeout: float) -> tuple[bool, str]:
        client = self._node.create_client(Trigger, service_name)
        try:
            if not client.wait_for_service(timeout_sec=2.0):
                return False, f"{service_name} not available — is open_door_server running?"
            done = threading.Event()
            outcome: dict[str, Any] = {"ok": False, "message": f"{label} timed out"}
            future = client.call_async(Trigger.Request())

            def _finished(_future: Any) -> None:
                try:
                    resp = _future.result()
                    outcome["ok"] = bool(resp.success)
                    outcome["message"] = str(resp.message)
                except Exception as exc:  # noqa: BLE001
                    outcome["message"] = f"{label} failed: {exc}"
                finally:
                    done.set()

            future.add_done_callback(_finished)
            done.wait(timeout=timeout)
            return bool(outcome["ok"]), str(outcome["message"])
        finally:
            self._node.destroy_client(client)

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
