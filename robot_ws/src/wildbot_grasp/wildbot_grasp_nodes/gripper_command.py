import sys
import time

import rclpy
from rclpy.node import Node

from .motion import ArmCommander


COMMAND_TO_POSE = {
    "open": "place_pose_deg",
    "place": "place_pose_deg",
    "release": "place_pose_deg",
    "close": "grasp_pose_deg",
    "grasp": "grasp_pose_deg",
}


class GripperCommand(Node):
    def __init__(self):
        super().__init__("gripper_command")
        self.arm = ArmCommander(self)

    def run(self, command: str) -> int:
        pose_name = COMMAND_TO_POSE.get(command)
        if pose_name is None:
            self.get_logger().error("usage: ros2 run wildbot_grasp gripper_command open|close")
            return 2

        # Give DDS a short moment to match /arm_controller/joint_trajectory.
        time.sleep(0.5)
        self.arm.send_named(command, pose_name)
        self.get_logger().info(f"sent {command} using {pose_name}")
        return 0


def main(args=None):
    rclpy.init(args=args)
    node = GripperCommand()
    try:
        command = sys.argv[1].lower() if len(sys.argv) > 1 else ""
        exit_code = node.run(command)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)
