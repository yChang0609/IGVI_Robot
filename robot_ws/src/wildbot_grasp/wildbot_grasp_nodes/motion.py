import math
import time
from typing import Sequence

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


ARM_JOINTS = ["arm_1_joint", "arm_2_joint", "gripper_joint"]


def degrees_to_radians(values: Sequence[float]) -> list[float]:
    return [math.radians(float(value)) for value in values]


def make_arm_trajectory(positions: Sequence[float], duration_sec: float = 1.0) -> JointTrajectory:
    msg = JointTrajectory()
    msg.joint_names = list(ARM_JOINTS)

    point = JointTrajectoryPoint()
    point.positions = [float(value) for value in positions]
    whole_seconds = int(duration_sec)
    point.time_from_start.sec = whole_seconds
    point.time_from_start.nanosec = int((duration_sec - whole_seconds) * 1_000_000_000)
    msg.points.append(point)
    return msg


class ArmCommander:
    def __init__(self, node):
        self.node = node
        self._declare_parameters()
        topic = self.node.get_parameter("arm_topic").value
        self.publisher = self.node.create_publisher(JointTrajectory, topic, 10)

    def _declare_parameters(self):
        self.node.declare_parameter("arm_topic", "/arm_controller/joint_trajectory")
        self.node.declare_parameter("move_duration_sec", 1.0)
        self.node.declare_parameter("settle_sec", 0.5)
        self.node.declare_parameter("grasp_pose_deg", [167.0, 75.0, 170.6])
        self.node.declare_parameter("place_pose_deg", [120.0, 75.0, 239.0])

    def pose_deg(self, name: str) -> list[float]:
        return [float(value) for value in self.node.get_parameter(name).value]

    def motion_wait_sec(self) -> float:
        duration = float(self.node.get_parameter("move_duration_sec").value)
        settle = float(self.node.get_parameter("settle_sec").value)
        return duration + settle

    def wait_for_subscriber(self, timeout_sec: float = 2.0):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self.publisher.get_subscription_count() > 0:
                return True
            time.sleep(0.05)
        self.node.get_logger().warning(
            "no subscriber matched on arm trajectory topic before publish"
        )
        return False

    def publish_degrees(self, stage: str, positions_deg: Sequence[float]) -> list[float]:
        positions_rad = degrees_to_radians(positions_deg)
        duration = float(self.node.get_parameter("move_duration_sec").value)
        self.node.get_logger().info(
            f"publishing {stage}: deg={[round(v, 2) for v in positions_deg]} "
            f"rad={[round(v, 4) for v in positions_rad]}"
        )
        self.wait_for_subscriber()
        self.publisher.publish(make_arm_trajectory(positions_rad, duration))
        return positions_rad

    def wait_after_publish(self):
        time.sleep(self.motion_wait_sec())

    def send_degrees(self, stage: str, positions_deg: Sequence[float]) -> list[float]:
        positions_rad = self.publish_degrees(stage, positions_deg)
        self.wait_after_publish()
        return positions_rad

    def publish_named(self, stage: str, pose_parameter: str) -> list[float]:
        return self.publish_degrees(stage, self.pose_deg(pose_parameter))

    def send_named(self, stage: str, pose_parameter: str) -> list[float]:
        return self.send_degrees(stage, self.pose_deg(pose_parameter))
