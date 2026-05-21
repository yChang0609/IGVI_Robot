#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.executors import MultiThreadedExecutor

from wildbot_grasp.action import SearchAndRetrieve
from wildbot_grasp_nodes.retrieve_base import RetrieveBase


class SearchRetrieveServer(RetrieveBase):
    _feedback_class = SearchAndRetrieve.Feedback

    def __init__(self):
        super().__init__(
            "search_retrieve_server",
            standoff_distance=0.25,
            visual_servo_kp=0.005,
            visual_servo_timeout=15.0,
        )
        self.action_server = ActionServer(
            self,
            SearchAndRetrieve,
            "search_retrieve",
            execute_callback=self.execute_callback,
            callback_group=self.callback_group,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.get_logger().info("Ready: /search_retrieve")

    def goal_callback(self, goal_request):
        self.get_logger().info(
            f"Received goal to search and retrieve target: {goal_request.target_id}"
        )
        return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle):
        result = SearchAndRetrieve.Result()
        goal = goal_handle.request
        target_id = goal.target_id

        # 1. Find target in semantic memory by id
        self.publish_feedback(goal_handle, "searching", 0.05, f"Looking for {target_id} in semantic memory")
        target = self.find_target_by_id(target_id, timeout_sec=5.0)
        if target is None:
            result.success = False
            result.message = f"Target {target_id} not found in memory"
            goal_handle.abort()
            return result

        # 2. Evaluate approach paths and pick the shortest
        target_name = target["class_name"]
        self.publish_feedback(goal_handle, "evaluating", 0.1, f"Found {target_name}. Evaluating approach paths.")
        approach_pose, approach_detail = self.choose_approach_pose(target["position"])
        if approach_pose is None:
            result.success = False
            result.message = approach_detail
            goal_handle.abort()
            return result

        # 3. Navigate to standoff pose
        ok, message = self.navigate_to_pose(goal_handle, approach_pose, "navigating", 0.3)
        if not ok:
            result.success = False
            result.message = message
            goal_handle.canceled() if goal_handle.is_cancel_requested else goal_handle.abort()
            return result

        # 4. Grasp via GrabObjectServer
        self.publish_feedback(goal_handle, "grabbing", 0.7, f"Grabbing {target_name}")
        ok, message = self.call_grab_object(goal_handle, target_name)
        if not ok:
            result.success = False
            result.message = message
            goal_handle.canceled() if goal_handle.is_cancel_requested else goal_handle.abort()
            return result

        # 5. Return to caller-specified home pose
        self.publish_feedback(goal_handle, "returning", 0.9, "Returning to specified home pose")
        home_pose = self.make_pose(goal.home_pose_x, goal.home_pose_y, goal.home_pose_yaw)
        ok, message = self.navigate_to_pose(goal_handle, home_pose, "returning", 0.9)
        if not ok:
            result.success = False
            result.message = message
            goal_handle.canceled() if goal_handle.is_cancel_requested else goal_handle.abort()
            return result

        # 6. Release
        self.publish_feedback(goal_handle, "releasing", 0.98, "Releasing object")
        ok, message = self.release_arm(goal_handle)
        if not ok:
            result.success = False
            result.message = message
            goal_handle.canceled() if goal_handle.is_cancel_requested else goal_handle.abort()
            return result

        result.success = True
        result.message = "Successfully retrieved object and returned"
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = SearchRetrieveServer()
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
