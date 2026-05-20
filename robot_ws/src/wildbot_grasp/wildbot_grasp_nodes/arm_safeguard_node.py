import rclpy
from rclpy.node import Node
import threading

from collections import deque
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
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
        self.position_history = deque(maxlen=100)
        
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
        # 計算距離上次下指令經過了多久
        dt = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        
        # 🌟 關鍵新增：只有當時間大於 current_duration (也就是預期已經走到定點後)，才開始記錄
        if dt > self.current_duration:
            if len(msg.feedback.positions) >= 3:
                self.position_history.append(msg.feedback.positions)

    def target_callback(self, msg: JointTrajectory):
        """收到任何節點的指令，更新時間戳並直接轉發給硬體"""
        with self.action_lock:
            self.last_cmd_time = self.get_clock().now()
            # 取出預期動作時間 (不需要在這裡 +2 了，我們把預期時間還原)
            if msg.points:
                t = msg.points[0].time_from_start
                self.current_duration = t.sec + (t.nanosec / 1e9)
            
            # 🌟 關鍵新增：收到新指令，立刻清空歷史紀錄，避免混到上一次的舊資料
            self.position_history.clear()
            self.arm_pub.publish(msg)

    def idle_monitor_callback(self):
        """閒置超過預期時間，發布平均值放鬆"""
        with self.action_lock:
            dt = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
            
            # 🌟 把你原本加的 2 秒延遲移到這裡：
            # 意思是：抵達目標後，再多等 2 秒鐘蒐集純淨數據，然後才放鬆
            if dt > (self.current_duration + 1.0) and self.position_history:
                history_len = len(self.position_history)
                avg_rad = [
                    sum(pos[i] for pos in self.position_history) / history_len
                    for i in range(3)
                ]
                relax_msg = JointTrajectory()
                relax_msg.header.stamp = self.get_clock().now().to_msg()
                relax_msg.joint_names = ['arm_1_joint', 'arm_2_joint', 'gripper_joint'] 
                point = JointTrajectoryPoint()
                point.positions = avg_rad
                point.time_from_start = builtin_interfaces.msg.Duration(sec=0, nanosec=100_000_000)
                relax_msg.points = [point]
                self.arm_pub.publish(relax_msg)
                
                # 發布放鬆後清空紀錄，避免下一個 0.5 秒重複觸發
                self.position_history.clear()

def main():
    rclpy.init()
    node = ArmSafeguardNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': main()