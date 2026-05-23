#!/usr/bin/env python3
import math

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

        home_pose = self.make_pose(goal.home_pose_x, goal.home_pose_y, goal.home_pose_yaw)
        self.get_logger().info(
            f"return home set to ({goal.home_pose_x:.2f}, "
            f"{goal.home_pose_y:.2f}, {goal.home_pose_yaw:.2f})"
        )

        # A door reference point is required: it decides the scan rotation sense
        # (and is the fallback direction when the target isn't in memory yet).
        if not goal.door_ref_valid:
            result.success = False
            result.message = "No door reference point set — set a door_ref point first"
            goal_handle.abort()
            return result

        # 1. Navigate to bridge center. Ignore the waypoint's stored yaw: arrive
        #    facing the travel direction from home to the bridge point, so the
        #    scan step starts from a natural heading. Fall back to the stored yaw
        #    if no home pose is available.
        approach_yaw = goal.bridge_pose_yaw
        if goal.home_pose_valid:
            approach_yaw = math.atan2(
                goal.bridge_pose_y - goal.home_pose_y,
                goal.bridge_pose_x - goal.home_pose_x,
            )
        bridge_pose = self.make_pose(goal.bridge_pose_x, goal.bridge_pose_y, approach_yaw)
        ok, message = self.navigate_to_pose(goal_handle, bridge_pose, "to_bridge_center", 0.15)
        if not ok:
            result.success = False
            result.message = message
            goal_handle.canceled() if goal_handle.is_cancel_requested else goal_handle.abort()
            return result

        # 2. Locate the target on the map. If semantic memory doesn't know where
        #    the bear is yet, do a stepped scan: rotate to face the door ref, then
        #    a fixed step further in the same sense, pausing at each stop so YOLO /
        #    semantic memory can register it. Abort if both views find nothing.
        target_obj = self.find_target_from_memory(target_class, timeout_sec=1.5)
        if target_obj is None:
            self.publish_feedback(goal_handle, "scanning", 0.4,
                                  f"Stepped scan toward door ref for {target_class}")
            target_obj, message = self.scan_door_steps_for_target(
                goal_handle, target_class, (goal.door_ref_x, goal.door_ref_y))
            if target_obj is None:
                result.success = False
                result.message = message
                goal_handle.canceled() if goal_handle.is_cancel_requested else goal_handle.abort()
                return result

        # Face the bear's map position, then hand off to centering.
        pos = target_obj["position"]
        self.publish_feedback(
            goal_handle, "facing", 0.5,
            f"Facing {target_class} at map ({pos['x']:.2f}, {pos['y']:.2f})",
        )
        ok, message = self.face_point(goal_handle, (pos["x"], pos["y"]))
        if not ok:
            result.success = False
            result.message = message
            goal_handle.canceled() if goal_handle.is_cancel_requested else goal_handle.abort()
            return result

        # 3. Center: rotate in place to align bear with image center
        self.publish_feedback(goal_handle, "centering", 0.55,
                              f"Centering on {target_class}")
        ok, message = self.center_on_target(goal_handle, target_class)
        if not ok:
            result.success = False
            result.message = message
            goal_handle.canceled() if goal_handle.is_cancel_requested else goal_handle.abort()
            return result

        # 4. Approach and grab
        self.publish_feedback(goal_handle, "grasping", 0.65,
                              f"Approaching and grabbing {target_class}")
        ok, message = self.approach_and_grab(goal_handle, target_class)
        if not ok:
            result.success = False
            result.message = message
            goal_handle.canceled() if goal_handle.is_cancel_requested else goal_handle.abort()
            return result

        # 5. Return home
        self.clear_costmaps()  # bear is now held — clear its pre-grasp marks before navigating
        ok, message = self.navigate_to_pose(goal_handle, home_pose, "returning_home", 0.88)
        if not ok:
            result.success = False
            result.message = message
            goal_handle.canceled() if goal_handle.is_cancel_requested else goal_handle.abort()
            return result

        # 6. Release
        self.publish_feedback(goal_handle, "releasing", 0.97, "Releasing object at start pose")
        ok, message = self.release_arm(goal_handle)
        if not ok:
            result.success = False
            result.message = message
            goal_handle.canceled() if goal_handle.is_cancel_requested else goal_handle.abort()
            return result

        self.clear_costmaps()  # clear marks left by the bear during carry
        result.success = True
        result.message = (
            f"Bridge mission complete: reached bridge center, acquired {target_class}, "
            "returned to home pose and released"
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
