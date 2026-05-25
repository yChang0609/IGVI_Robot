import math
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from wildbot_grasp.action import GrabObject

from .motion import ArmCommander


class GrabObjectServer(Node):
    def __init__(self):
        super().__init__("grab_object_server")
        self.declare_parameter("grasp_detect_min_error_deg", 2.5)
        self.declare_parameter("grasp_detect_min_close_motion_deg", 5.0)
        self.declare_parameter("grasp_detect_settle_sec", 1.0)
        self.declare_parameter("grasp_check_timeout_sec", 1.5)
        self.declare_parameter("max_grasp_attempts", 2)
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("temperature_topic", "/arm_joint_temperatures")
        self.declare_parameter("gripper_temperature_index", 2)
        self.declare_parameter("gripper_max_start_temp_c", 68.0)
        self.declare_parameter("gripper_resume_temp_c", 65.0)
        self.declare_parameter("initial_pose_on_start", True)
        self.declare_parameter("initial_pose_delay_sec", 1.0)
        self.declare_parameter("release_after_grasp", False)
        # Per-action trajectory duration (overrides move_duration_sec for each step).
        # Tune these to balance speed vs. smoothness for each grab phase.
        self.declare_parameter("open_duration_sec", 0.8)   # 張爪移到抓取準備位 0.8
        self.declare_parameter("close_duration_sec", 0.3)  # 閉爪夾物 1.2
        self.declare_parameter("carry_duration_sec", 0.5)  # 夾住後移到搬運位 1.5
        self.declare_parameter("home_duration_sec", 0.3)   # 回 home（失敗/完成）1.0

        self.latest_joint_state = None
        self.latest_temperatures = None
        self.callback_group = ReentrantCallbackGroup()
        self.arm = ArmCommander(self)
        self.create_subscription(
            JointState,
            self.get_parameter("joint_states_topic").value,
            self.joint_state_callback,
            10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Float64MultiArray,
            self.get_parameter("temperature_topic").value,
            self.temperature_callback,
            10,
            callback_group=self.callback_group,
        )
        self.action_server = ActionServer(
            self,
            GrabObject,
            "grab_object",
            execute_callback=self.execute_callback,
            callback_group=self.callback_group,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.initial_pose_timer = None
        if bool(self.get_parameter("initial_pose_on_start").value):
            delay = max(0.1, float(self.get_parameter("initial_pose_delay_sec").value))
            self.initial_pose_timer = self.create_timer(
                delay,
                self.publish_initial_pose_once,
                callback_group=self.callback_group,
            )
        self.get_logger().info("Ready: /grab_object")

    def publish_initial_pose_once(self):
        if self.initial_pose_timer is not None:
            self.initial_pose_timer.cancel()
            self.initial_pose_timer = None
        self.get_logger().info("moving arm to initial/home pose")
        self.arm.send_named("initial_home_pose", "home_pose_deg",
                            float(self.get_parameter("home_duration_sec").value))

    def joint_state_callback(self, msg):
        self.latest_joint_state = msg

    def temperature_callback(self, msg):
        self.latest_temperatures = [float(value) for value in msg.data]

    def gripper_temperature_c(self):
        temperatures = self.latest_temperatures
        if temperatures is None:
            return None
        index = int(self.get_parameter("gripper_temperature_index").value)
        if index < 0 or index >= len(temperatures):
            return None
        return temperatures[index]

    def check_temperature_safe(self):
        temperature = self.gripper_temperature_c()
        max_start = float(self.get_parameter("gripper_max_start_temp_c").value)
        resume = float(self.get_parameter("gripper_resume_temp_c").value)
        if temperature is None or math.isnan(temperature):
            return True, "gripper temperature unavailable; continuing"
        if temperature >= max_start:
            return (
                False,
                f"gripper overheat: {temperature:.1f}C >= {max_start:.1f}C; "
                f"wait until <= {resume:.1f}C before grasping",
            )
        return True, f"gripper temperature ok: {temperature:.1f}C < {max_start:.1f}C"

    def goal_callback(self, goal_request):
        self.get_logger().info(
            f"accepted grasp goal object={goal_request.object_label} "
            f"distance={goal_request.distance_m:.3f}m"
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle):
        return CancelResponse.ACCEPT

    def publish_feedback(self, goal_handle, stage: str, progress: float, detail: str):
        feedback = GrabObject.Feedback()
        feedback.stage = stage
        feedback.progress = float(progress)
        feedback.detail = detail
        goal_handle.publish_feedback(feedback)
        self.get_logger().info(f"{stage}: {detail}")

    def current_gripper_position(self):
        msg = self.latest_joint_state
        if msg is None:
            return None
        try:
            index = list(msg.name).index("gripper_joint")
        except ValueError:
            return None
        if index >= len(msg.position):
            return None
        return float(msg.position[index])

    def gripper_position_detail(self, label: str):
        position = self.current_gripper_position()
        if position is None:
            return f"{label}: gripper_joint unavailable"
        return f"{label}: gripper={math.degrees(position):.1f}deg ({position:.4f}rad)"

    def current_joint_snapshot(self):
        msg = self.latest_joint_state
        if msg is None:
            return None
        names = list(msg.name)
        try:
            gripper_index = names.index("gripper_joint")
        except ValueError:
            return None
        if gripper_index >= len(msg.position):
            return None

        positions = [float(value) for value in msg.position]
        velocities = [float(value) for value in msg.velocity]
        efforts = [float(value) for value in msg.effort]
        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1_000_000_000.0
        return {
            "stamp_sec": stamp_sec,
            "names": names,
            "positions": positions,
            "velocities": velocities,
            "efforts": efforts,
            "gripper_rad": positions[gripper_index],
        }

    def format_joint_values(self, names, values, unit: str = ""):
        if not values:
            return "[]"
        parts = []
        for index, value in enumerate(values):
            name = names[index] if index < len(names) else f"joint_{index}"
            if math.isnan(value):
                formatted = "nan"
            else:
                formatted = f"{value:.3f}"
            parts.append(f"{name}={formatted}{unit}")
        return "[" + ", ".join(parts) + "]"

    def min_gripper_sample_detail(
        self,
        label: str,
        target_gripper_rad: float,
        open_gripper_rad: float | None,
        duration_sec: float,
        sample_period_sec: float = 0.05,
    ):
        start = time.monotonic()
        deadline = start + duration_sec
        sample_count = 0
        unique_sample_count = 0
        seen_stamps = set()
        first_gripper_rad = None
        last_gripper_rad = None
        min_snapshot = None
        min_elapsed = None

        while rclpy.ok():
            now = time.monotonic()
            if now > deadline:
                break
            snapshot = self.current_joint_snapshot()
            if snapshot is not None:
                sample_count += 1
                stamp_key = snapshot["stamp_sec"]
                if stamp_key in seen_stamps:
                    time.sleep(sample_period_sec)
                    continue
                seen_stamps.add(stamp_key)
                unique_sample_count += 1
                gripper_rad = snapshot["gripper_rad"]
                if first_gripper_rad is None:
                    first_gripper_rad = gripper_rad
                last_gripper_rad = gripper_rad
                if min_snapshot is None or gripper_rad < min_snapshot["gripper_rad"]:
                    min_snapshot = snapshot
                    min_elapsed = now - start
            time.sleep(sample_period_sec)

        if min_snapshot is None:
            detail = (
                f"{label}: no gripper_joint samples during {duration_sec:.3f}s "
                f"sample_period={sample_period_sec:.3f}s"
            )
            self.get_logger().info(f"gripper min sample: {detail}")
            return detail

        names = min_snapshot["names"]
        positions_rad = min_snapshot["positions"]
        positions_deg = [math.degrees(value) for value in positions_rad]
        min_gripper_rad = min_snapshot["gripper_rad"]
        min_error = min_gripper_rad - target_gripper_rad
        min_close_motion = None if open_gripper_rad is None else open_gripper_rad - min_gripper_rad
        temperature = self.gripper_temperature_c()
        detail = (
            f"{label}: sample_count={sample_count} unique_samples={unique_sample_count} "
            f"duration={duration_sec:.3f}s sample_period={sample_period_sec:.3f}s "
            f"min_elapsed={min_elapsed:.3f}s joint_stamp={min_snapshot['stamp_sec']:.6f} "
            f"target={math.degrees(target_gripper_rad):.3f}deg/{target_gripper_rad:.6f}rad "
            f"open={None if open_gripper_rad is None else math.degrees(open_gripper_rad):}deg/{open_gripper_rad}rad "
            f"first={None if first_gripper_rad is None else math.degrees(first_gripper_rad):}deg/{first_gripper_rad}rad "
            f"last={None if last_gripper_rad is None else math.degrees(last_gripper_rad):}deg/{last_gripper_rad}rad "
            f"min={math.degrees(min_gripper_rad):.3f}deg/{min_gripper_rad:.6f}rad "
            f"min_error={math.degrees(min_error):+.3f}deg/{min_error:+.6f}rad "
            f"min_close_motion={None if min_close_motion is None else math.degrees(min_close_motion):}deg/{min_close_motion}rad "
            f"temperature={temperature}C "
            f"positions_deg={self.format_joint_values(names, positions_deg, 'deg')} "
            f"positions_rad={self.format_joint_values(names, positions_rad, 'rad')} "
            f"velocities={self.format_joint_values(names, min_snapshot['velocities'])} "
            f"efforts={self.format_joint_values(names, min_snapshot['efforts'])}"
        )
        self.get_logger().info(f"gripper min sample: {detail}")
        return detail

    def wait_for_gripper_position(self, timeout_sec: float):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and rclpy.ok():
            position = self.current_gripper_position()
            if position is not None:
                return position
            time.sleep(0.05)
        return None

    def detect_grasp(self, target_gripper_rad: float, open_gripper_rad: float | None = None):
        settle = float(self.get_parameter("grasp_detect_settle_sec").value)
        if settle > 0.0:
            time.sleep(settle)
        timeout = float(self.get_parameter("grasp_check_timeout_sec").value)
        actual = self.wait_for_gripper_position(timeout)
        if actual is None:
            return False, "no gripper_joint feedback available"

        min_error = math.radians(float(self.get_parameter("grasp_detect_min_error_deg").value))
        min_close_motion = math.radians(float(self.get_parameter("grasp_detect_min_close_motion_deg").value))
        error = actual - target_gripper_rad
        close_motion = None if open_gripper_rad is None else open_gripper_rad - actual
        has_blocking_error = error >= min_error
        has_closed_motion = close_motion is None or close_motion >= min_close_motion
        grasped = has_blocking_error and has_closed_motion

        if grasped:
            decision = "grasped"
        elif not has_closed_motion:
            decision = "not_grasped: gripper stayed near open position"
        else:
            decision = "not_grasped: gripper reached close target"

        if close_motion is None:
            motion_detail = "open=unknown close_motion=unknown"
        else:
            motion_detail = (
                f"open={math.degrees(open_gripper_rad):.1f}deg "
                f"close_motion={math.degrees(close_motion):+.1f}deg "
                f"min_close_motion={math.degrees(min_close_motion):.1f}deg"
            )

        open_gripper_deg = None if open_gripper_rad is None else math.degrees(open_gripper_rad)
        close_motion_deg = None if close_motion is None else math.degrees(close_motion)
        variable_detail = (
            f"settle={settle:.3f}s "
            f"timeout={timeout:.3f}s "
            f"target_gripper_rad={target_gripper_rad:.6f} "
            f"target_gripper_deg={math.degrees(target_gripper_rad):.3f} "
            f"open_gripper_rad={open_gripper_rad} "
            f"open_gripper_deg={open_gripper_deg} "
            f"actual_rad={actual:.6f} "
            f"actual_deg={math.degrees(actual):.3f} "
            f"min_error_rad={min_error:.6f} "
            f"min_error_deg={math.degrees(min_error):.3f} "
            f"min_close_motion_rad={min_close_motion:.6f} "
            f"min_close_motion_deg={math.degrees(min_close_motion):.3f} "
            f"error_rad={error:.6f} "
            f"error_deg={math.degrees(error):+.3f} "
            f"close_motion_rad={close_motion} "
            f"close_motion_deg={close_motion_deg} "
            f"has_blocking_error={has_blocking_error} "
            f"has_closed_motion={has_closed_motion} "
            f"grasped={grasped} "
            f"decision={decision}"
        )
        self.get_logger().info(f"detect_grasp variables: {variable_detail}")

        detail = (
            f"gripper close_target={math.degrees(target_gripper_rad):.1f}deg "
            f"actual={math.degrees(actual):.1f}deg "
            f"error={math.degrees(error):+.1f}deg "
            f"threshold={math.degrees(min_error):.1f}deg "
            f"{motion_detail} "
            f"decision={decision}; vars: {variable_detail}"
        )
        return grasped, detail

    def open_at_grasp_pose_deg(self):
        grasp_pose = self.arm.pose_deg("grasp_pose_deg")
        place_pose = self.arm.pose_deg("place_pose_deg")
        return [grasp_pose[0], grasp_pose[1], place_pose[2]]

    def execute_callback(self, goal_handle):
        result = GrabObject.Result()
        goal = goal_handle.request
        label = goal.object_label or "object"
        max_attempts = max(1, int(self.get_parameter("max_grasp_attempts").value))
        grasp_detail = "not checked"
        attempt_details = []
        object_grasped = False
        safety_abort = False

        temperature_safe, temperature_detail = self.check_temperature_safe()
        if not temperature_safe:
            self.publish_feedback(goal_handle, "safety_abort", 0.0, temperature_detail)
            goal_handle.abort()
            result.success = False
            result.object_grasped = False
            result.message = temperature_detail
            return result

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.success = False
            result.object_grasped = False
            result.message = "grasp canceled before motion"
            return result

        for attempt in range(1, max_attempts + 1):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.object_grasped = object_grasped
                result.message = f"grasp canceled before attempt {attempt}"
                return result

            temperature_safe, temperature_detail = self.check_temperature_safe()
            if not temperature_safe:
                self.publish_feedback(goal_handle, "safety_abort", 0.0, temperature_detail)
                grasp_detail = temperature_detail
                attempt_details.append(f"attempt {attempt}/{max_attempts} safety check: {temperature_detail}")
                safety_abort = True
                break

            open_dur = float(self.get_parameter("open_duration_sec").value)
            close_dur = float(self.get_parameter("close_duration_sec").value)

            detail = f"attempt {attempt}/{max_attempts}: opening gripper at grasp pose ({temperature_detail})"
            self.publish_feedback(goal_handle, "open_for_grasp", 0.15, detail)
            self.arm.send_degrees("open_for_grasp", self.open_at_grasp_pose_deg(), open_dur)
            open_gripper_rad = self.wait_for_gripper_position(float(self.get_parameter("grasp_check_timeout_sec").value))
            self.publish_feedback(
                goal_handle,
                "gripper_angle",
                0.25,
                f"attempt {attempt}/{max_attempts}: {self.gripper_position_detail('after open_for_grasp')}",
            )

            detail = f"attempt {attempt}/{max_attempts}: closing gripper for {label}"
            self.publish_feedback(goal_handle, "close_gripper", 0.35, detail)
            grasp_pose_rad = self.arm.publish_named(f"close_gripper_attempt_{attempt}", "grasp_pose_deg", close_dur)
            min_detail = self.min_gripper_sample_detail(
                f"attempt {attempt}/{max_attempts} during close_gripper",
                grasp_pose_rad[2],
                open_gripper_rad,
                self.arm.motion_wait_sec(close_dur),
            )
            self.publish_feedback(goal_handle, "gripper_min_angle", 0.43, min_detail)

            self.publish_feedback(
                goal_handle,
                "gripper_angle",
                0.45,
                f"attempt {attempt}/{max_attempts}: {self.gripper_position_detail('after close command')}",
            )
            settle = float(self.get_parameter("grasp_detect_settle_sec").value)
            self.publish_feedback(
                goal_handle,
                "check_grasp",
                0.5,
                f"attempt {attempt}/{max_attempts}: waiting {settle:.1f}s then checking gripper close target error",
            )
            grasp_detected, grasp_detail = self.detect_grasp(grasp_pose_rad[2], open_gripper_rad)
            attempt_details.append(f"attempt {attempt}/{max_attempts} grasp check: {grasp_detail}")

            if not grasp_detected:
                if attempt < max_attempts:
                    self.publish_feedback(goal_handle, "retry", 0.58, f"not grasped; retrying. {attempt_details[-1]}")
                continue

            self.publish_feedback(
                goal_handle,
                "move_to_carry_holding",
                0.72,
                "grasp detected; moving to carry pose while keeping gripper closed",
            )
            carry_dur = float(self.get_parameter("carry_duration_sec").value)
            carry_pose_rad = self.arm.send_named("move_to_carry_holding", "carry_pose_deg", carry_dur)
            self.publish_feedback(
                goal_handle,
                "gripper_angle",
                0.8,
                f"attempt {attempt}/{max_attempts}: {self.gripper_position_detail('after move_to_carry_holding')}",
            )

            self.publish_feedback(goal_handle, "verify_at_carry", 0.84, "checking object is still held in carry pose")
            object_grasped, carry_detail = self.detect_grasp(carry_pose_rad[2], open_gripper_rad)
            attempt_details.append(f"attempt {attempt}/{max_attempts} carry check: {carry_detail}")
            grasp_detail = carry_detail

            if object_grasped:
                if bool(self.get_parameter("release_after_grasp").value):
                    home_dur = float(self.get_parameter("home_duration_sec").value)
                    self.publish_feedback(goal_handle, "release_at_place", 0.92, "object still held; opening gripper at place pose")
                    self.arm.send_named("release_at_place", "place_pose_deg", home_dur)
                    self.publish_feedback(goal_handle, "return_home_after_release", 0.97, "object released; returning arm to home pose")
                    self.arm.send_named("return_home_after_release", "home_pose_deg", home_dur)
                else:
                    self.publish_feedback(goal_handle, "hold_for_navigation", 0.97, "object held; staying in carry pose for navigation")
                break

            if attempt < max_attempts:
                self.publish_feedback(goal_handle, "retry", 0.88, f"object was not held at carry pose; retrying. {attempt_details[-1]}")

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.success = False
            result.object_grasped = object_grasped
            result.message = "grasp canceled after check"
            return result

        if not object_grasped and not safety_abort:
            self.publish_feedback(goal_handle, "reset_to_home", 0.9, grasp_detail)
            self.arm.send_named("reset_to_home", "home_pose_deg",
                                float(self.get_parameter("home_duration_sec").value))

        result.success = bool(object_grasped)
        result.object_grasped = bool(object_grasped)
        result.message = (
            f"completed after {len(attempt_details)} attempt(s); object_grasped={object_grasped}; "
            f"{' | '.join(attempt_details)}. "
            "Voltage/current-based detection is not available because kros_car does not publish those signals yet."
        )
        if object_grasped:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = GrabObjectServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
