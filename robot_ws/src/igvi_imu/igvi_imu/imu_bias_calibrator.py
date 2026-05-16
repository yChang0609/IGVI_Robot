from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import SensorDataQoS
from sensor_msgs.msg import Imu
from std_msgs.msg import Empty, String


GRAVITY_MPS2 = 9.80665


class ImuBiasCalibrator(Node):
    """Subtract gyro bias and refine it online while the IMU is stationary."""

    def __init__(self) -> None:
        super().__init__("imu_bias_calibrator")
        self.declare_parameter("gyro_bias", [0.0, 0.0, 0.0])
        self.declare_parameter("online_calibration", True)
        self.declare_parameter("manual_calibration_required", True)
        self.declare_parameter("manual_calibration_duration", 30.0)
        self.declare_parameter("stationary_gyro_threshold", 0.03)
        self.declare_parameter("stationary_accel_tolerance", 0.8)
        self.declare_parameter("stationary_min_duration", 3.0)
        self.declare_parameter("bias_time_constant", 10.0)
        self.declare_parameter("convergence_gyro_threshold", 0.005)
        self.declare_parameter("convergence_min_duration", 5.0)
        self.declare_parameter("publish_state_period", 1.0)
        self.declare_parameter("auto_start_on_boot", True)

        self._bias = self._read_bias_parameter()
        self._online_calibration = bool(self.get_parameter("online_calibration").value)
        self._manual_required = bool(self.get_parameter("manual_calibration_required").value)
        self._manual_duration = max(1.0, float(self.get_parameter("manual_calibration_duration").value))
        self._gyro_threshold = float(self.get_parameter("stationary_gyro_threshold").value)
        self._accel_tolerance = float(self.get_parameter("stationary_accel_tolerance").value)
        self._stationary_min_duration = float(self.get_parameter("stationary_min_duration").value)
        self._bias_time_constant = max(0.1, float(self.get_parameter("bias_time_constant").value))
        self._convergence_gyro_threshold = float(
            self.get_parameter("convergence_gyro_threshold").value
        )
        self._convergence_min_duration = float(self.get_parameter("convergence_min_duration").value)

        self._last_msg_time = None
        self._manual_started_at = None
        self._stationary_since = None
        self._converged_since = None
        self._stationary = False
        self._converged = False
        self._manual_active = not self._manual_required
        self._manual_remaining_s = 0.0
        self._calibration_active = False
        self._stationary_age = 0.0
        self._convergence_age = 0.0
        self._gyro_error = 0.0

        qos = SensorDataQoS()
        self._pub = self.create_publisher(Imu, "imu/out", qos)
        self._state_pub = self.create_publisher(String, "imu/calibration_state", 10)
        self.create_subscription(Imu, "imu/in", self._on_imu, qos)
        self.create_subscription(Empty, "calibration/start", self._on_start_calibration, 10)

        state_period = max(0.1, float(self.get_parameter("publish_state_period").value))
        self.create_timer(state_period, self._publish_state)
        auto_start = bool(self.get_parameter("auto_start_on_boot").value)
        if auto_start and self._manual_required:
            self._manual_started_at = self.get_clock().now()
            self._manual_active = True
            self._manual_remaining_s = self._manual_duration

        self.get_logger().info(
            "imu_bias_calibrator started "
            f"bias=[{self._bias[0]:.6f}, {self._bias[1]:.6f}, {self._bias[2]:.6f}] "
            f"online={self._online_calibration} manual_required={self._manual_required} "
            f"auto_start={auto_start}"
        )

    def _read_bias_parameter(self) -> list[float]:
        value = self.get_parameter("gyro_bias").value
        if not isinstance(value, (list, tuple)) or len(value) != 3:
            self.get_logger().warning("gyro_bias must contain three values; using zeros")
            return [0.0, 0.0, 0.0]
        return [float(value[0]), float(value[1]), float(value[2])]

    def _on_start_calibration(self, _msg: Empty) -> None:
        self._manual_started_at = self.get_clock().now()
        self._manual_active = True
        self._manual_remaining_s = self._manual_duration
        self._converged = False
        self._converged_since = None
        self._convergence_age = 0.0
        self.get_logger().info(
            f"manual IMU calibration window started for {self._manual_duration:.1f}s"
        )

    def _on_imu(self, msg: Imu) -> None:
        now = self.get_clock().now()
        dt = None
        if self._last_msg_time is not None:
            dt = (now - self._last_msg_time).nanoseconds * 1e-9
        self._last_msg_time = now
        self._update_manual_window(now)

        gyro = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        accel_norm = _norm(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        corrected_gyro_norm = _norm(
            gyro[0] - self._bias[0],
            gyro[1] - self._bias[1],
            gyro[2] - self._bias[2],
        )
        stationary_now = (
            corrected_gyro_norm < self._gyro_threshold
            and abs(accel_norm - GRAVITY_MPS2) < self._accel_tolerance
        )

        if stationary_now:
            if self._stationary_since is None:
                self._stationary_since = now
            self._stationary_age = (now - self._stationary_since).nanoseconds * 1e-9
            self._stationary = self._stationary_age >= self._stationary_min_duration
        else:
            self._stationary_since = None
            self._converged_since = None
            self._stationary = False
            self._converged = False
            self._stationary_age = 0.0
            self._convergence_age = 0.0

        self._calibration_active = (
            self._online_calibration
            and self._manual_active
            and self._stationary
        )

        if self._calibration_active and dt is not None and 0.0 < dt < 1.0:
            alpha = min(1.0, dt / self._bias_time_constant)
            for i in range(3):
                self._bias[i] = (1.0 - alpha) * self._bias[i] + alpha * float(gyro[i])

        self._gyro_error = _norm(
            gyro[0] - self._bias[0],
            gyro[1] - self._bias[1],
            gyro[2] - self._bias[2],
        )
        if self._calibration_active and self._gyro_error < self._convergence_gyro_threshold:
            if self._converged_since is None:
                self._converged_since = now
            self._convergence_age = (now - self._converged_since).nanoseconds * 1e-9
            self._converged = self._convergence_age >= self._convergence_min_duration
        else:
            self._converged_since = None
            self._converged = False
            self._convergence_age = 0.0

        self._pub.publish(self._corrected_message(msg))

    def _update_manual_window(self, now) -> None:
        if not self._manual_required:
            self._manual_active = True
            self._manual_remaining_s = 0.0
            return
        if self._manual_started_at is None:
            self._manual_active = False
            self._manual_remaining_s = 0.0
            return

        elapsed = (now - self._manual_started_at).nanoseconds * 1e-9
        remaining = self._manual_duration - elapsed
        self._manual_active = remaining > 0.0
        self._manual_remaining_s = max(0.0, remaining)
        if not self._manual_active:
            self._manual_started_at = None
            self._calibration_active = False

    def _corrected_message(self, msg: Imu) -> Imu:
        out = Imu()
        out.header = msg.header
        out.orientation = msg.orientation
        out.orientation_covariance = msg.orientation_covariance
        out.angular_velocity.x = msg.angular_velocity.x - self._bias[0]
        out.angular_velocity.y = msg.angular_velocity.y - self._bias[1]
        out.angular_velocity.z = msg.angular_velocity.z - self._bias[2]
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        return out

    def _publish_state(self) -> None:
        msg = String()
        state = "idle" if self._manual_required else "moving"
        if not self._online_calibration:
            state = "disabled"
        elif self._converged:
            state = "converged"
        elif self._calibration_active:
            state = "converging"
        elif self._manual_active and self._stationary_since is None:
            state = "waiting"
        elif self._stationary_since is not None:
            state = "stationary"
        elif not self._manual_required:
            state = "moving"
        msg.data = (
            f"state={state} "
            f"stationary={str(self._stationary).lower()} "
            f"converged={str(self._converged).lower()} "
            f"online={str(self._online_calibration).lower()} "
            f"manual_required={str(self._manual_required).lower()} "
            f"manual_active={str(self._manual_active).lower()} "
            f"calibration_active={str(self._calibration_active).lower()} "
            f"manual_remaining_s={self._manual_remaining_s:.1f} "
            f"stationary_age_s={self._stationary_age:.1f} "
            f"convergence_age_s={self._convergence_age:.1f} "
            f"gyro_error_rad_s={self._gyro_error:.6f} "
            f"gyro_bias=[{self._bias[0]:.7f},{self._bias[1]:.7f},{self._bias[2]:.7f}]"
        )
        self._state_pub.publish(msg)


def _norm(x: float, y: float, z: float) -> float:
    return math.sqrt(x * x + y * y + z * z)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ImuBiasCalibrator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
