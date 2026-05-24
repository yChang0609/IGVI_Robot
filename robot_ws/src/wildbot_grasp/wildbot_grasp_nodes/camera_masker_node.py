import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge

class CameraMaskerNode(Node):
    def __init__(self):
        super().__init__('camera_masker_node')

        import os
        # Parameters
        env_home = os.environ.get("ARM_HOME_POSE_DEG")
        if env_home:
            try:
                default_home = [float(x.strip()) for x in env_home.split(",")]
            except Exception:
                default_home = [190.0, 0.0, 240.0]
        else:
            default_home = [190.0, 0.0, 240.0]

        env_tol = os.environ.get("ARM_JOINT_TOLERANCE_DEG")
        default_tol = float(env_tol) if env_tol else 10.0

        env_mask_pct = os.environ.get("ARM_MASK_HEIGHT_PCT")
        default_mask_pct = float(env_mask_pct) if env_mask_pct else 0.50

        self.declare_parameter('home_pose_deg', default_home)
        self.declare_parameter('joint_tolerance_deg', default_tol)
        self.declare_parameter('mask_height_pct', default_mask_pct)
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('rgb_input_topic', '/rgb/image_raw')
        self.declare_parameter('depth_input_topic', '/depth_to_rgb/image_raw')
        self.declare_parameter('rgb_output_topic', '/rgb/image_filtered')
        self.declare_parameter('depth_output_topic', '/depth_to_rgb/image_filtered')

        home_deg = self.get_parameter('home_pose_deg').value
        self.arm_1_home = math.radians(float(home_deg[0]))
        self.arm_2_home = math.radians(float(home_deg[1]))
        self.gripper_home = math.radians(float(home_deg[2]))
        self.tolerance = math.radians(self.get_parameter('joint_tolerance_deg').value)
        self.mask_height_pct = self.get_parameter('mask_height_pct').value

        # Arm state
        self.arm_is_home = True
        self.latest_joint_state_time = 0.0

        # CvBridge
        self.bridge = CvBridge()

        # Subscribers
        self.create_subscription(
            JointState,
            self.get_parameter('joint_states_topic').value,
            self.joint_states_callback,
            10
        )

        self.create_subscription(
            Image,
            self.get_parameter('rgb_input_topic').value,
            self.rgb_callback,
            10
        )

        self.create_subscription(
            Image,
            self.get_parameter('depth_input_topic').value,
            self.depth_callback,
            10
        )

        # Publishers
        self.rgb_pub = self.create_publisher(
            Image,
            self.get_parameter('rgb_output_topic').value,
            10
        )

        self.depth_pub = self.create_publisher(
            Image,
            self.get_parameter('depth_output_topic').value,
            10
        )

        self.get_logger().info(
            f"Camera Masker initialized: home_pose_deg={home_deg} "
            f"mask_height={self.mask_height_pct * 100:.1f}%"
        )

    def joint_states_callback(self, msg: JointState):
        # We need to map the joint names to their values
        joint_map = dict(zip(msg.name, msg.position))

        arm_1 = joint_map.get('arm_1_joint')
        arm_2 = joint_map.get('arm_2_joint')
        gripper = joint_map.get('gripper_joint')

        # If any of the arm joints are missing in the /joint_states message,
        # we assume the arm state has not changed or default to home to be safe.
        if arm_1 is None or arm_2 is None or gripper is None:
            return

        # Check deviations from home pose
        dev_1 = abs(arm_1 - self.arm_1_home)
        dev_2 = abs(arm_2 - self.arm_2_home)
        dev_g = abs(gripper - self.gripper_home)

        is_home = (dev_1 <= self.tolerance) and (dev_2 <= self.tolerance) and (dev_g <= self.tolerance)

        if is_home != self.arm_is_home:
            self.arm_is_home = is_home
            state_str = "HOME (No Mask)" if is_home else "ACTIVE (Masking Enabled)"
            self.get_logger().info(
                f"Arm state transition: {state_str}. "
                f"Joints: arm_1={math.degrees(arm_1):.1f}°, arm_2={math.degrees(arm_2):.1f}°, gripper={math.degrees(gripper):.1f}°"
            )

    def rgb_callback(self, msg: Image):
        if self.arm_is_home:
            # Zero-copy republishing when arm is home
            self.rgb_pub.publish(msg)
            return

        try:
            # We must use "bgr8" or "rgb8" depending on source, but "passthrough" preserves the original
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            h, w = cv_image.shape[:2]
            
            # Mask the lower part
            mask_start_row = int(h * (1.0 - self.mask_height_pct))
            
            # Modifying in place is extremely fast on NumPy arrays
            cv_image[mask_start_row:, :] = 0

            # Convert back to ROS message and preserve header timestamp
            masked_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding=msg.encoding)
            masked_msg.header = msg.header
            self.rgb_pub.publish(masked_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to mask RGB image: {e}")
            self.rgb_pub.publish(msg)

    def depth_callback(self, msg: Image):
        if self.arm_is_home:
            # Zero-copy republishing when arm is home
            self.depth_pub.publish(msg)
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            h, w = cv_image.shape[:2]
            
            mask_start_row = int(h * (1.0 - self.mask_height_pct))
            
            # Set depth to 0 (invalid/missing depth in ROS)
            cv_image[mask_start_row:, :] = 0

            masked_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding=msg.encoding)
            masked_msg.header = msg.header
            self.depth_pub.publish(masked_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to mask depth image: {e}")
            self.depth_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraMaskerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
