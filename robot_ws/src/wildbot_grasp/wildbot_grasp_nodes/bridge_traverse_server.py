#!/usr/bin/env python3
import math
import threading

import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.executors import MultiThreadedExecutor

from wildbot_grasp.action import BridgeTraverse
from wildbot_grasp_nodes.retrieve_base import RetrieveBase


class BridgeTraverseServer(RetrieveBase):
    _feedback_class = BridgeTraverse.Feedback

    def __init__(self):
        super().__init__("bridge_traverse_server")
        self._goal_lock = threading.Lock()
        self._active_goal = False

        self.action_server = ActionServer(
            self,
            BridgeTraverse,
            "bridge_traverse",
            execute_callback=self.execute_callback,
            callback_group=self.callback_group,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.get_logger().info("Ready: /bridge_traverse")

    def goal_callback(self, goal_request):
        with self._goal_lock:
            if self._active_goal:
                self.get_logger().warn("Rejecting goal: another traverse is already active")
                return GoalResponse.REJECT
            self._active_goal = True

        self.get_logger().info(
            f"Bridge traverse goal bridge=({goal_request.bridge_pose_x:.2f}, "
            f"{goal_request.bridge_pose_y:.2f}, {goal_request.bridge_pose_yaw:.2f})"
        )
        return GoalResponse.ACCEPT

    def _finish_goal(self) -> None:
        with self._goal_lock:
            self._active_goal = False

    def _home_pose_from_goal(self, goal):
        if goal.home_pose_valid:
            return self.make_pose(goal.home_pose_x, goal.home_pose_y, goal.home_pose_yaw)

        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            return None
        x, y, yaw = robot_pose
        goal.home_pose_x = float(x)
        goal.home_pose_y = float(y)
        goal.home_pose_yaw = float(yaw)
        return self.make_pose(x, y, yaw)

    def execute_callback(self, goal_handle):
        result = BridgeTraverse.Result()
        goal = goal_handle.request

        try:
            home_pose = self._home_pose_from_goal(goal)
            if home_pose is None:
                result.success = False
                result.message = "Could not determine home pose"
                goal_handle.abort()
                return result

            self.get_logger().info(
                f"return home set to ({goal.home_pose_x:.2f}, "
                f"{goal.home_pose_y:.2f}, {goal.home_pose_yaw:.2f})"
            )

            # Arrive at bridge center facing the travel direction from home to
            # bridge. If home was unavailable in the request, use the stored yaw.
            approach_yaw = goal.bridge_pose_yaw
            if goal.home_pose_valid:
                approach_yaw = math.atan2(
                    goal.bridge_pose_y - goal.home_pose_y,
                    goal.bridge_pose_x - goal.home_pose_x,
                )
            bridge_pose = self.make_pose(goal.bridge_pose_x, goal.bridge_pose_y, approach_yaw)
            ok, message = self.navigate_to_pose(goal_handle, bridge_pose, "to_bridge_center", 0.45)
            if not ok:
                result.success = False
                result.message = message
                goal_handle.canceled() if goal_handle.is_cancel_requested else goal_handle.abort()
                return result

            # Directly leave the bridge. Return via-points force the same
            # deterministic down-bridge route used by the full retrieve mission.
            self.clear_costmaps()
            via_x = list(goal.return_path_x)
            via_y = list(goal.return_path_y)
            via_yaw = list(goal.return_path_yaw)
            return_legs = []
            for index in range(len(via_x)):
                px, py = via_x[index], via_y[index]
                stored_yaw = via_yaw[index] if index < len(via_yaw) else 0.0
                if abs(stored_yaw) > 1e-6:
                    yaw = stored_yaw
                else:
                    nx, ny = (
                        (via_x[index + 1], via_y[index + 1])
                        if index + 1 < len(via_x)
                        else (goal.home_pose_x, goal.home_pose_y)
                    )
                    yaw = math.atan2(ny - py, nx - px)
                return_legs.append((f"return_via_{index + 1}", self.make_pose(px, py, yaw)))
            return_legs.append(("returning_home", home_pose))

            total = len(return_legs)
            for index, (stage, pose) in enumerate(return_legs):
                progress = 0.45 + 0.50 * ((index + 1) / total)
                ok, message = self.navigate_to_pose(goal_handle, pose, stage, progress)
                if not ok:
                    result.success = False
                    result.message = message
                    goal_handle.canceled() if goal_handle.is_cancel_requested else goal_handle.abort()
                    return result

            self.clear_costmaps()
            result.success = True
            result.message = "Bridge traverse complete: reached bridge center and returned home"
            goal_handle.succeed()
            return result
        finally:
            self._finish_goal()


def main(args=None):
    rclpy.init(args=args)
    node = BridgeTraverseServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
