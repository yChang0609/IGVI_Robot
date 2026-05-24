import json
import math
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from wildbot_grasp.action import GrabObject


@dataclass
class TargetDetection:
    label: str
    score: float
    center_x: float
    center_y: float
    size_x: float
    size_y: float
    image_width: float | None
    image_height: float | None
    distance_m: float | None
    stamp_monotonic: float


class TaskState(str, Enum):
    IDLE = "idle"
    WAITING_FOR_BEAR = "waiting_for_bear"
    ALIGNING = "aligning"
    GRASPING = "grasping"
    BACKING_UP = "backing_up"
    REALIGNING = "realigning"
    SUCCEEDED = "succeeded"
    FAILED = "failed"


class BearGraspTaskNode(Node):
    """Detect a bear, drive into grasp range, and call /grab_object."""

    def __init__(self):
        super().__init__("bear_grasp_task_node")
        self.callback_group = ReentrantCallbackGroup()
        self._declare_parameters()

        self.state = TaskState.IDLE
        self.active = bool(self.get_parameter("auto_start").value)
        self.latest_detection: TargetDetection | None = None
        self.grasp_retry_count = 0
        self.goal_in_flight = False
        self.in_grasp_range = False
        self.last_state_publish = 0.0
        self.last_detection_log = 0.0

        detection_topic = str(self.get_parameter("detection_topic").value)
        cmd_topic = str(self.get_parameter("cmd_vel_topic").value)
        state_topic = str(self.get_parameter("state_topic").value)
        self.create_subscription(String, detection_topic, self._on_detections, 10, callback_group=self.callback_group)
        self.cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.state_pub = self.create_publisher(String, state_topic, 10)
        self.grab_client = ActionClient(
            self,
            GrabObject,
            str(self.get_parameter("grab_action_name").value),
            callback_group=self.callback_group,
        )
        self.create_service(Trigger, "~/start", self._on_start, callback_group=self.callback_group)
        self.create_service(Trigger, "~/stop", self._on_stop, callback_group=self.callback_group)
        self.create_service(Trigger, "~/status", self._on_status, callback_group=self.callback_group)
        self.timer = self.create_timer(
            1.0 / float(self.get_parameter("control_rate_hz").value),
            self._tick,
            callback_group=self.callback_group,
        )

        if self.active:
            self._set_state(TaskState.WAITING_FOR_BEAR, "auto_start enabled")
        else:
            self._set_state(TaskState.IDLE, "waiting for ~/start")
        self.get_logger().info(
            f"Bear grasp task ready: detections={detection_topic} cmd={cmd_topic} "
            f"target_max_distance={self.get_parameter('target_max_distance_m').value}m"
        )

    def _declare_parameters(self):
        self.declare_parameter("auto_start", False)
        self.declare_parameter("detection_topic", "/detections_json")
        self.declare_parameter("cmd_vel_topic", "/motion/cmd")
        self.declare_parameter("state_topic", "/bear_grasp/state")
        self.declare_parameter("grab_action_name", "grab_object")
        self.declare_parameter("target_labels", ["bear", "teddy bear", "xiong", "熊","xiong_qiao"])
        self.declare_parameter("min_confidence", 0.35)
        self.declare_parameter("target_max_distance_m", 0.24)
        # Once the target enters the grasp band, depth noise must exceed this
        # margin (in metres) past target_max before we drop back to approaching.
        # This stops the robot from bouncing in/out of the narrow band and never grasping.
        self.declare_parameter("distance_hysteresis_m", 0.04)
        self.declare_parameter("require_valid_distance", True)
        self.declare_parameter("max_detection_age_sec", 0.7)
        self.declare_parameter("control_rate_hz", 10.0)
        self.declare_parameter("fallback_image_width", 1280.0)
        self.declare_parameter("fallback_image_height", 720.0)
        self.declare_parameter("target_center_x_px", -1.0)
        self.declare_parameter("max_angular_z", 0.28)
        self.declare_parameter("angular_kp", 0.0022)
        self.declare_parameter("max_linear_x", 0.06)
        self.declare_parameter("min_linear_x", 0.025)
        self.declare_parameter("linear_kp", 0.35)
        self.declare_parameter("backup_linear_x", -0.05)
        self.declare_parameter("backup_duration_sec", 0.8)
        self.declare_parameter("retry_pause_sec", 0.4)
        self.declare_parameter("max_task_retries", 0)

    def _on_start(self, _request, response):
        self.active = True
        self.grasp_retry_count = 0
        self.goal_in_flight = False
        self.in_grasp_range = False
        self._set_state(TaskState.WAITING_FOR_BEAR, "manual start")
        response.success = True
        response.message = "bear grasp task started"
        return response

    def _on_stop(self, _request, response):
        self.active = False
        self.goal_in_flight = False
        self.in_grasp_range = False
        self._publish_stop()
        self._set_state(TaskState.IDLE, "manual stop")
        response.success = True
        response.message = "bear grasp task stopped"
        return response

    def _on_status(self, _request, response):
        response.success = True
        response.message = self._status_message(self.latest_detection)
        return response

    def _on_detections(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warning(f"invalid detection JSON: {exc}")
            return

        image_width = self._first_number(payload, ["image_width", "width"])
        image_height = self._first_number(payload, ["image_height", "height"])
        best = self._select_target(payload, image_width, image_height)
        if best is None:
            return
        self.latest_detection = best
        now = time.monotonic()
        if now - self.last_detection_log > 1.0:
            self.last_detection_log = now
            self.get_logger().info(
                f"target {best.label} score={best.score:.2f} center=({best.center_x:.1f},{best.center_y:.1f}) "
                f"distance={best.distance_m}m"
            )

    def _select_target(self, payload: dict[str, Any], image_width: float | None, image_height: float | None) -> TargetDetection | None:
        detections = payload.get("detections") or []
        labels = {str(label).lower() for label in self.get_parameter("target_labels").value}
        min_conf = float(self.get_parameter("min_confidence").value)
        best: TargetDetection | None = None

        for item in detections:
            if not isinstance(item, dict):
                continue
            label = str(item.get("class_name") or item.get("label") or item.get("class_id") or "")
            if labels and label.lower() not in labels:
                continue
            score = float(item.get("score") or item.get("confidence") or 0.0)
            if score < min_conf:
                continue
            bbox = item.get("bbox") or {}
            if not isinstance(bbox, dict):
                continue
            center_x = self._number(bbox.get("center_x"))
            center_y = self._number(bbox.get("center_y"))
            size_x = self._number(bbox.get("size_x"))
            size_y = self._number(bbox.get("size_y"))
            if center_x is None or center_y is None or size_x is None or size_y is None:
                continue
            candidate = TargetDetection(
                label=label,
                score=score,
                center_x=center_x,
                center_y=center_y,
                size_x=size_x,
                size_y=size_y,
                image_width=image_width,
                image_height=image_height,
                distance_m=self._distance_from_detection(item),
                stamp_monotonic=time.monotonic(),
            )
            if best is None or self._target_priority(candidate) > self._target_priority(best):
                best = candidate
        return best

    def _target_priority(self, detection: TargetDetection):
        # A target without depth cannot enter the grasp distance gate.
        # Prefer usable depth over a high-score edge detection with depth=None.
        has_distance = 1 if detection.distance_m is not None else 0
        center_error = abs(self._center_error_px(detection))
        return (has_distance, -center_error, detection.score)

    def _distance_from_detection(self, item: dict[str, Any]) -> float | None:
        if item.get("depth_valid") is False:
            return None
        for key in ("depth_m", "distance_m", "distance"):
            value = self._number(item.get(key))
            if value is not None and math.isfinite(value) and value > 0.0:
                return value
        return None

    def _tick(self):
        self._publish_state_periodic()
        if not self.active or self.goal_in_flight:
            return
        if self.state in (TaskState.IDLE, TaskState.SUCCEEDED, TaskState.FAILED,
                          TaskState.BACKING_UP, TaskState.REALIGNING):
            return

        detection = self._fresh_detection()
        if detection is None:
            self.in_grasp_range = False
            self._publish_stop()
            self._set_state(TaskState.WAITING_FOR_BEAR, "waiting for fresh target detection")
            return
        if not self._distance_ready(detection):
            self._drive_toward_distance(detection)
            self._set_state(TaskState.ALIGNING, "approaching target distance")
            return
        # 距離一到就直接夾取：不再要求置中對齊或穩定保持。
        self._publish_stop()
        self.goal_in_flight = True
        self._set_state(TaskState.GRASPING, f"target in range for {detection.label}; grabbing")
        self._send_grab_goal(detection)

    def _fresh_detection(self) -> TargetDetection | None:
        detection = self.latest_detection
        if detection is None:
            return None
        if time.monotonic() - detection.stamp_monotonic > float(self.get_parameter("max_detection_age_sec").value):
            return None
        return detection

    def _distance_ready(self, detection: TargetDetection) -> bool:
        max_distance = float(self.get_parameter("target_max_distance_m").value)
        hysteresis = max(0.0, float(self.get_parameter("distance_hysteresis_m").value))

        if detection.distance_m is None:
            # Depth often drops out at very close range. If we already reached the
            # grasp range, stay latched and proceed to grasp instead of stalling.
            if self.in_grasp_range:
                return True
            return not bool(self.get_parameter("require_valid_distance").value)

        if self.in_grasp_range:
            # Only fall back to approaching if the target is clearly too far again.
            if detection.distance_m > max_distance + hysteresis:
                self.in_grasp_range = False
                self.get_logger().info(
                    f"target receded to {detection.distance_m:.3f}m (> {max_distance + hysteresis:.3f}m); "
                    "re-approaching"
                )
                return False
            return True

        # Close enough means ready: target_max is the only active distance gate.
        if detection.distance_m <= max_distance:
            self.in_grasp_range = True
            self.get_logger().info(
                f"reached grasp range at {detection.distance_m:.3f}m (<= {max_distance:.3f}m); grabbing"
            )
            return True
        return False

    def _drive_toward_distance(self, detection: TargetDetection):
        if detection.distance_m is None:
            self._publish_stop()
            return
        max_distance = float(self.get_parameter("target_max_distance_m").value)
        distance_error = max(0.0, detection.distance_m - max_distance)
        twist = self._centering_twist(detection)
        linear = distance_error * float(self.get_parameter("linear_kp").value)
        max_linear = float(self.get_parameter("max_linear_x").value)
        min_linear = max(0.0, float(self.get_parameter("min_linear_x").value))
        if distance_error > 0.0:
            linear = max(min_linear, linear)
        twist.linear.x = max(-max_linear, min(max_linear, linear))
        self.cmd_pub.publish(twist)

    def _centering_twist(self, detection: TargetDetection) -> Twist:
        error_px = self._center_error_px(detection)
        angular = -error_px * float(self.get_parameter("angular_kp").value)
        max_angular = float(self.get_parameter("max_angular_z").value)
        twist = Twist()
        twist.angular.z = max(-max_angular, min(max_angular, angular))
        return twist

    def _center_error_px(self, detection: TargetDetection) -> float:
        target_center_x = float(self.get_parameter("target_center_x_px").value)
        if target_center_x < 0.0:
            if detection.image_width is None or detection.image_width <= 0.0:
                target_center_x = float(self.get_parameter("fallback_image_width").value) / 2.0
            else:
                target_center_x = detection.image_width / 2.0
        return detection.center_x - target_center_x

    def _send_grab_goal(self, detection: TargetDetection):
        if not self.grab_client.wait_for_server(timeout_sec=0.2):
            self.goal_in_flight = False
            self._set_state(TaskState.WAITING_FOR_BEAR, "waiting for /grab_object action server")
            return
        goal = GrabObject.Goal()
        goal.object_label = detection.label
        goal.distance_m = float(detection.distance_m or 0.0)
        self.goal_in_flight = True
        self._set_state(TaskState.GRASPING, f"sending grab goal for {detection.label}")
        future = self.grab_client.send_goal_async(goal, feedback_callback=self._on_grab_feedback)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.goal_in_flight = False
            self._retry_or_fail("grab goal rejected")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_grab_result)

    def _on_grab_feedback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"grab feedback stage={feedback.stage} progress={feedback.progress:.2f} detail={feedback.detail}")

    def _on_grab_result(self, future):
        self.goal_in_flight = False
        result = future.result().result
        if result.success and result.object_grasped:
            self._publish_stop()
            self._set_state(TaskState.SUCCEEDED, result.message)
            return
        self._retry_or_fail(result.message)

    def _retry_or_fail(self, reason: str):
        max_retries = int(self.get_parameter("max_task_retries").value)
        if max_retries > 0 and self.grasp_retry_count >= max_retries:
            self.active = False
            self.in_grasp_range = False
            self._publish_stop()
            self._set_state(TaskState.FAILED, f"grasp failed after {self.grasp_retry_count} retries: {reason}")
            return

        self.grasp_retry_count += 1
        self.in_grasp_range = False
        self._set_state(TaskState.BACKING_UP, f"grasp failed; backing up before retry {self.grasp_retry_count}: {reason}")
        self._back_up_before_retry()
        pause = float(self.get_parameter("retry_pause_sec").value)
        self._set_state(TaskState.REALIGNING, f"retry {self.grasp_retry_count}: {reason}")
        if pause > 0.0:
            time.sleep(pause)
        self._set_state(TaskState.WAITING_FOR_BEAR, "retrying after backup and realignment pause")

    def _back_up_before_retry(self):
        twist = Twist()
        twist.linear.x = float(self.get_parameter("backup_linear_x").value)
        deadline = time.monotonic() + max(0.0, float(self.get_parameter("backup_duration_sec").value))
        while rclpy.ok() and time.monotonic() < deadline:
            self.cmd_pub.publish(twist)
            time.sleep(0.05)
        self._publish_stop()

    def _publish_stop(self):
        self.cmd_pub.publish(Twist())

    def _set_state(self, state: TaskState, detail: str):
        if self.state != state:
            self.get_logger().info(f"state {self.state.value} -> {state.value}: {detail}")
        elif detail:
            self.get_logger().debug(f"state {state.value}: {detail}")
        self.state = state
        self._publish_state(detail)

    def _publish_state_periodic(self):
        now = time.monotonic()
        if now - self.last_state_publish > 1.0:
            self._publish_state("")

    def _publish_state(self, detail: str):
        msg = String()
        msg.data = json.dumps({
            "state": self.state.value,
            "active": self.active,
            "retry_count": self.grasp_retry_count,
            "detail": detail,
            "detection": self._detection_payload(self.latest_detection),
        })
        self.state_pub.publish(msg)
        self.last_state_publish = time.monotonic()

    def _detection_payload(self, detection: TargetDetection | None):
        if detection is None:
            return None
        return {
            "label": detection.label,
            "score": detection.score,
            "center_x": detection.center_x,
            "center_y": detection.center_y,
            "center_error_px": self._center_error_px(detection),
            "distance_m": detection.distance_m,
            "age_sec": time.monotonic() - detection.stamp_monotonic,
        }

    def _status_message(self, detection: TargetDetection | None) -> str:
        return json.dumps({
            "state": self.state.value,
            "active": self.active,
            "retry_count": self.grasp_retry_count,
            "detection": self._detection_payload(detection),
        })

    def _first_number(self, payload: dict[str, Any], keys: list[str]) -> float | None:
        for key in keys:
            value = self._number(payload.get(key))
            if value is not None:
                return value
        return None

    def _number(self, value: Any) -> float | None:
        try:
            if value is None:
                return None
            return float(value)
        except (TypeError, ValueError):
            return None


def main(args=None):
    rclpy.init(args=args)
    node = BearGraspTaskNode()
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
