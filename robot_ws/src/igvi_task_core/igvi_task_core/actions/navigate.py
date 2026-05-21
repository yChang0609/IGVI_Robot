"""Nav2 NavigateToPose action client wrapper.

Calls send_goal_async / get_result_async from the task thread while the
MultiThreadedExecutor spins in the main thread — safe because rclpy futures
are thread-safe by design.

On interrupt: cancels the active goal before re-raising InterruptedError.
"""
from __future__ import annotations

import math
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

from igvi_task_core.fsm import TaskFSM


class Navigator:
    def __init__(self, node: Node) -> None:
        self._node = node
        self._client = ActionClient(node, NavigateToPose, "navigate_to_pose")
        self._active_handle = None

    def go_to(self, x: float, y: float, yaw: float, fsm: TaskFSM) -> None:
        """Navigate to (x, y, yaw) in the map frame.

        Blocks until navigation succeeds or raises on failure / interrupt.
        """
        if not self._client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("Nav2 action server unavailable")

        goal = NavigateToPose.Goal()
        goal.pose = _make_pose(x, y, yaw)

        send_future = self._client.send_goal_async(goal)
        self._wait_future(send_future, fsm)

        handle = send_future.result()
        if not handle.accepted:
            raise RuntimeError("Nav2 rejected goal")
        self._active_handle = handle

        result_future = handle.get_result_async()
        try:
            self._wait_future(result_future, fsm)
        except InterruptedError:
            handle.cancel_goal_async()
            raise

        self._active_handle = None
        status = result_future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"Navigation failed (status={status})")

    def cancel(self) -> None:
        if self._active_handle is not None:
            self._active_handle.cancel_goal_async()

    @staticmethod
    def _wait_future(future, fsm: TaskFSM) -> None:
        while not future.done():
            fsm.check()
            time.sleep(0.05)


def _make_pose(x: float, y: float, yaw: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    q = _yaw_to_quat(yaw)
    pose.pose.orientation = q
    return pose


def _yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q
