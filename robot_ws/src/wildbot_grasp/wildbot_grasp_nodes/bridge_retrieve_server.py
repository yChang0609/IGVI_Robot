#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.executors import MultiThreadedExecutor

from wildbot_grasp.action import BridgeRetrieve
from wildbot_grasp_nodes.retrieve_base import RetrieveBase


class BridgeRetrieveServer(RetrieveBase):
    _feedback_class = BridgeRetrieve.Feedback

    def __init__(self):
        super().__init__(
            "bridge_retrieve_server",
            standoff_distance=0.22,
            visual_servo_kp=0.002,
            visual_servo_timeout=12.0,
        )
        self.declare_parameter("target_class", "xiong_qiao")
        self.declare_parameter("search_memory_timeout", 8.0)

        self.action_server = ActionServer(
            self,
            BridgeRetrieve,
            "bridge_retrieve",
            execute_callback=self.execute_callback,
            callback_group=self.callback_group,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.get_logger().info("Ready: /bridge_retrieve")

    def goal_callback(self, goal_request):
        target = goal_request.target_class or self.get_parameter("target_class").value
        self.get_logger().info(
            f"Bridge retrieve goal target={target} bridge=({goal_request.bridge_pose_x:.2f}, "
            f"{goal_request.bridge_pose_y:.2f}, {goal_request.bridge_pose_yaw:.2f})"
        )
        return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle):
        result = BridgeRetrieve.Result()
        goal = goal_handle.request
        target_class = goal.target_class or str(self.get_parameter("target_class").value)

        # Record starting pose before leaving, so we can return here after grasping.
        home_pose_tuple = self.get_robot_pose()
        if home_pose_tuple is None:
            result.success = False
            result.message = "Could not read starting pose"
            goal_handle.abort()
            return result
        home_pose = self.make_pose(*home_pose_tuple)

        # 1. Navigate to bridge center
        bridge_pose = self.make_pose(goal.bridge_pose_x, goal.bridge_pose_y, goal.bridge_pose_yaw)
        ok, message = self.navigate_to_pose(goal_handle, bridge_pose, "to_bridge_center", 0.15)
        if not ok:
            result.success = False
            result.message = message
            goal_handle.canceled() if goal_handle.is_cancel_requested else goal_handle.abort()
            return result

        # 2. Find target in semantic memory
        self.publish_feedback(goal_handle, "searching_target", 0.35, f"Looking for {target_class}")
        target = self.find_target_from_memory(
            target_class, float(self.get_parameter("search_memory_timeout").value)
        )
        if target is None:
            result.success = False
            result.message = f"Target class {target_class} not found in semantic memory"
            goal_handle.abort()
            return result

        # 3. Choose and navigate to approach pose
        approach_pose, approach_detail = self.choose_approach_pose(target["position"])
        if approach_pose is None:
            result.success = False
            result.message = approach_detail
            goal_handle.abort()
            return result

        ok, message = self.navigate_to_pose(goal_handle, approach_pose, "approaching_target", 0.50)
        if not ok:
            result.success = False
            result.message = message
            goal_handle.canceled() if goal_handle.is_cancel_requested else goal_handle.abort()
            return result

        # 4. Visual alignment
        self.publish_feedback(goal_handle, "aligning", 0.62, "Centering target with YOLO bbox")
        _, align_detail = self.visual_align(goal_handle, target_class)
        self.publish_feedback(goal_handle, "aligning", 0.68, align_detail)

        if goal_handle.is_cancel_requested:
            result.success = False
            result.message = "mission canceled"
            goal_handle.canceled()
            return result

        # 5. Grasp via GrabObjectServer
        self.publish_feedback(goal_handle, "grasping", 0.72, f"Closing gripper on {target_class}")
        ok, message = self.call_grab_object(goal_handle, target_class)
        if not ok:
            result.success = False
            result.message = message
            goal_handle.canceled() if goal_handle.is_cancel_requested else goal_handle.abort()
            return result

        # 6. Return home
        ok, message = self.navigate_to_pose(goal_handle, home_pose, "returning_home", 0.88)
        if not ok:
            result.success = False
            result.message = message
            goal_handle.canceled() if goal_handle.is_cancel_requested else goal_handle.abort()
            return result

        # 7. Release
        self.publish_feedback(goal_handle, "releasing", 0.97, "Releasing object at start pose")
        ok, message = self.release_arm(goal_handle)
        if not ok:
            result.success = False
            result.message = message
            goal_handle.canceled() if goal_handle.is_cancel_requested else goal_handle.abort()
            return result

        result.success = True
        result.message = (
            f"Bridge mission complete: reached bridge center, acquired {target_class}, "
            "returned to starting pose and released"
        )
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = BridgeRetrieveServer()
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
