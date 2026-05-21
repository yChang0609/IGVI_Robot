"""wildbot_grasp GrabObject action client wrapper.

Wraps the existing /grab_object action server (wildbot_grasp package).
Returns (success, object_grasped) or raises on failure / interrupt.
"""
from __future__ import annotations

import time

from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node

from igvi_task_core.fsm import TaskFSM

try:
    from wildbot_grasp.action import GrabObject
    _HAS_GRASP = True
except ImportError:
    _HAS_GRASP = False


class Grabber:
    def __init__(self, node: Node) -> None:
        self._node = node
        self._active_handle = None
        if _HAS_GRASP:
            self._client = ActionClient(node, GrabObject, "grab_object")
        else:
            self._client = None
            node.get_logger().warn("wildbot_grasp not found — grab actions will be simulated")

    def execute(
        self,
        object_label: str,
        distance_m: float,
        fsm: TaskFSM,
    ) -> tuple[bool, bool]:
        """Send a GrabObject goal and wait for result.

        Returns (success, object_grasped).
        Raises InterruptedError if FSM is interrupted.
        Raises RuntimeError on action failure.
        """
        if self._client is None:
            self._node.get_logger().warn(f"[grab] simulated: {object_label} @ {distance_m:.2f}m")
            fsm.sleep(2.0)
            return True, True

        if not self._client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("grab_object action server unavailable")

        goal = GrabObject.Goal()
        goal.object_label = object_label
        goal.distance_m = float(distance_m)

        send_future = self._client.send_goal_async(goal)
        _wait_future(send_future, fsm)

        handle = send_future.result()
        if not handle.accepted:
            raise RuntimeError("grab_object goal rejected")
        self._active_handle = handle

        result_future = handle.get_result_async()
        try:
            _wait_future(result_future, fsm)
        except InterruptedError:
            handle.cancel_goal_async()
            raise

        self._active_handle = None
        res = result_future.result()
        if res.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"GrabObject action failed (status={res.status})")

        r = res.result
        return bool(r.success), bool(r.object_grasped)

    def cancel(self) -> None:
        if self._active_handle is not None:
            self._active_handle.cancel_goal_async()


def _wait_future(future, fsm: TaskFSM) -> None:
    while not future.done():
        fsm.check()
        time.sleep(0.05)
