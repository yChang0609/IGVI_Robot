import rclpy
from rclpy.node import Node
import threading
import math
from collections import deque
from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
import builtin_interfaces.msg

class ArmSafeguardNode(Node):
    def __init__(self):
        super().__init__('arm_safeguard_node')
        # 1. 訂閱來自 motion.py 的高階指令
        self.target_sub = self.create_subscription(
            JointTrajectory, 
            '/arm_safeguard/target_trajectory', 
            self.target_callback, 
            10
        )
        # 2. 真正發布給硬體馬達的主題
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        
        # 3. 狀態紀錄與防護鎖
        self.action_lock = threading.Lock()
        self.last_cmd_time = self.get_clock().now()
        self.current_duration = 0.0
        self.position_history = deque(maxlen=30) 
        
        # 4. 訂閱馬達真實狀態
        self.state_sub = self.create_subscription(
            JointTrajectoryControllerState,
            '/arm_controller/controller_state',
            self.state_callback,
            10
        )
        # 5. 降溫放鬆計時器
        self.idle_timer = self.create_timer(1, self.idle_monitor_callback)
        self.get_logger().info("Arm Safeguard 啟動：負責防過熱與穩態放鬆")

    def state_callback(self, msg):
        if len(msg.feedback.positions) >= 3:
            self.position_history.append(msg.feedback.positions)

    def target_callback(self, msg: JointTrajectory):
        """收到任何節點的指令，更新時間戳並直接轉發給硬體"""
        with self.action_lock:
            self.last_cmd_time = self.get_clock().now()
            # 取出預期動作時間
            if msg.points:
                t = msg.points[0].time_from_start
                self.current_duration = t.sec + (t.nanosec / 1e9) + 2
            self.arm_pub.publish(msg)

    def idle_monitor_callback(self):
        """閒置超過預期時間，發布平均值放鬆"""
        with self.action_lock:
            dt = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
            if dt > (self.current_duration + 0.5) and self.position_history:
                history_len = len(self.position_history)
                avg_rad = [
                    sum(pos[i] for pos in self.position_history) / history_len
                    for i in range(3)
                ]
                relax_msg = JointTrajectory()
                relax_msg.header.stamp = self.get_clock().now().to_msg()
                relax_msg.joint_names = ['arm_1_joint', 'arm_2_joint', 'gripper_joint'] 
                point = builtin_interfaces.msg.JointTrajectoryPoint()
                point.positions = avg_rad
                point.time_from_start = builtin_interfaces.msg.Duration(sec=0, nanosec=100_000_000)
                relax_msg.points = [point]
                self.arm_pub.publish(relax_msg)

def main():
    rclpy.init()
    node = ArmSafeguardNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': main()