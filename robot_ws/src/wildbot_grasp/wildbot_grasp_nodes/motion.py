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
        self.node.declare_parameter("arm_topic", "/arm_safeguard/target_trajectory")
        self.node.declare_parameter("move_duration_sec", 1.0)
        self.node.declare_parameter("settle_sec", 0.5)
        self.node.declare_parameter("grasp_pose_deg", [167.0, 75.0, 130.0])
        self.node.declare_parameter("place_pose_deg", [120.0, 75.0, 239.0])
        self.node.declare_parameter("carry_pose_deg", [190.0, 0.0, 130.0])
        # Per-pose duration overrides; <= 0 means use move_duration_sec
        self.node.declare_parameter("grasp_pose_duration_sec", -1.0)
        self.node.declare_parameter("place_pose_duration_sec", -1.0)
        self.node.declare_parameter("carry_pose_duration_sec", -1.0)
        self.node.declare_parameter("home_pose_duration_sec", -1.0)
        
        import os
        env_home = os.environ.get("ARM_HOME_POSE_DEG")
        if env_home:
            try:
                default_home = [float(x.strip()) for x in env_home.split(",")]
            except Exception:
                default_home = [190.0, 0.0, 240.0]
        else:
            default_home = [190.0, 0.0, 240.0]
            
        self.node.declare_parameter("home_pose_deg", default_home)

    def pose_deg(self, name: str) -> list[float]:
        return [float(value) for value in self.node.get_parameter(name).value]

    def duration_for_pose(self, pose_parameter: str) -> float:
        specific = pose_parameter.replace("_deg", "_duration_sec")
        try:
            val = float(self.node.get_parameter(specific).value)
            if val > 0:
                return val
        except Exception:
            pass
        return float(self.node.get_parameter("move_duration_sec").value)

    def motion_wait_sec(self, pose_parameter: str | None = None) -> float:
        duration = self.duration_for_pose(pose_parameter) if pose_parameter else float(self.node.get_parameter("move_duration_sec").value)
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

    def publish_degrees(self, stage: str, positions_deg: Sequence[float], duration_sec: float | None = None) -> list[float]:
        positions_rad = degrees_to_radians(positions_deg)
        if duration_sec is None:
            duration_sec = float(self.node.get_parameter("move_duration_sec").value)
        self.node.get_logger().info(
            f"publishing {stage}: deg={[round(v, 2) for v in positions_deg]} "
            f"rad={[round(v, 4) for v in positions_rad]} duration={duration_sec:.3f}s"
        )
        self.wait_for_subscriber()
        self.publisher.publish(make_arm_trajectory(positions_rad, duration_sec))
        return positions_rad

    def wait_after_publish(self, duration_sec: float | None = None):
        settle = float(self.node.get_parameter("settle_sec").value)
        if duration_sec is None:
            duration_sec = float(self.node.get_parameter("move_duration_sec").value)
        time.sleep(duration_sec + settle)

    def send_degrees(self, stage: str, positions_deg: Sequence[float], duration_sec: float | None = None) -> list[float]:
        positions_rad = self.publish_degrees(stage, positions_deg, duration_sec)
        self.wait_after_publish(duration_sec)
        return positions_rad

    def publish_named(self, stage: str, pose_parameter: str) -> list[float]:
        duration_sec = self.duration_for_pose(pose_parameter)
        return self.publish_degrees(stage, self.pose_deg(pose_parameter), duration_sec)

    def send_named(self, stage: str, pose_parameter: str) -> list[float]:
        duration_sec = self.duration_for_pose(pose_parameter)
        return self.send_degrees(stage, self.pose_deg(pose_parameter), duration_sec)
