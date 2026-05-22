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
        )
        self.declare_parameter("target_class", "xiong_qiao")

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

        # 1. Navigate to bridge center (gets the robot near the bear/wall)
        bridge_pose = self.make_pose(goal.bridge_pose_x, goal.bridge_pose_y, goal.bridge_pose_yaw)
        ok, message = self.navigate_to_pose(goal_handle, bridge_pose, "to_bridge_center", 0.15)
        if not ok:
            result.success = False
            result.message = message
            goal_handle.canceled() if goal_handle.is_cancel_requested else goal_handle.abort()
            return result

        # 2. Locate the target. If a remembered point of this class sits near the
        # bridge, just turn to face it; otherwise scan-rotate until the bear shows
        # up in YOLO (CW 45deg -> CCW 90deg -> keep rotating).
        bridge_point = (goal.bridge_pose_x, goal.bridge_pose_y)
        mem = self.find_memory_near(
            target_class, bridge_point, float(self.get_parameter("bridge_memory_radius_m").value)
        )
        if mem is not None:
            pos = mem["position"]
            self.publish_feedback(goal_handle, "facing_target", 0.45,
                                  f"Memory point near bridge; facing {target_class}")
            ok, message = self.face_point(goal_handle, (pos["x"], pos["y"]))
        else:
            self.publish_feedback(goal_handle, "scanning", 0.45,
                                  f"No memory near bridge; scanning for {target_class}")
            ok, message = self.scan_for_target(goal_handle, target_class)
        if not ok:
            result.success = False
            result.message = message
            goal_handle.canceled() if goal_handle.is_cancel_requested else goal_handle.abort()
            return result

        # 3. Grasp state: YOLO centering + approach + grab (shared with search_retrieve)
        self.publish_feedback(goal_handle, "grasping", 0.62, f"Approaching and grabbing {target_class}")
        ok, message = self.approach_and_grab(goal_handle, target_class)
        if not ok:
            result.success = False
            result.message = message
            goal_handle.canceled() if goal_handle.is_cancel_requested else goal_handle.abort()
            return result

        # 4. Return home
        ok, message = self.navigate_to_pose(goal_handle, home_pose, "returning_home", 0.88)
        if not ok:
            result.success = False
            result.message = message
            goal_handle.canceled() if goal_handle.is_cancel_requested else goal_handle.abort()
            return result

        # 5. Release
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
