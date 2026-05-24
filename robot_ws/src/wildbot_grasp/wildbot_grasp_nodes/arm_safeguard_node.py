import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import threading

from collections import deque
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import builtin_interfaces.msg

ARM_JOINT_NAMES = ['arm_1_joint', 'arm_2_joint', 'gripper_joint']

# Latched QoS matching the bridge's /estop publisher so we get the current
# state as soon as we subscribe, even if we start after the bridge.
_ESTOP_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


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
        self.last_measured = None  # latest measured joint positions (for E-stop hold)

        # 4. 訂閱馬達真實狀態
        self.state_sub = self.create_subscription(
            JointTrajectoryControllerState,
            '/arm_controller/controller_state',
            self.state_callback,
            10
        )
        # 5. 降溫放鬆計時器
        self.idle_timer = self.create_timer(1, self.idle_monitor_callback)

        # 6. 緊急停止：收到 /estop=True 時凍結手臂、丟棄新指令，並持續抓住當前位置
        self.estop_engaged = False
        self.estop_sub = self.create_subscription(
            Bool, '/estop', self.estop_callback, _ESTOP_QOS
        )
        self.get_logger().info("Arm Safeguard 啟動：負責防過熱、穩態放鬆與緊急停止")

    def estop_callback(self, msg: Bool):
        engaged = bool(msg.data)
        with self.action_lock:
            self.estop_engaged = engaged
            if engaged and self.last_measured is not None:
                self._publish_hold(self.last_measured)
        self.get_logger().warn(
            "E-STOP：手臂凍結" if engaged else "E-STOP 解除：手臂恢復接受指令"
        )

    def _publish_hold(self, positions):
        """Command the arm to actively hold the given measured position."""
        hold = JointTrajectory()
        hold.header.stamp = self.get_clock().now().to_msg()
        hold.joint_names = list(ARM_JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions[:3]]
        point.time_from_start = builtin_interfaces.msg.Duration(sec=0, nanosec=100_000_000)
        hold.points = [point]
        self.arm_pub.publish(hold)

    def state_callback(self, msg):
        # 隨時記錄最新量測位置，E-stop 凍結時用它來抓住手臂
        if len(msg.feedback.positions) >= 3:
            self.last_measured = list(msg.feedback.positions)

        # 計算距離上次下指令經過了多久
        dt = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9

        # 🌟 關鍵新增：只有當時間大於 current_duration (也就是預期已經走到定點後)，才開始記錄
        if dt > self.current_duration:
            if len(msg.feedback.positions) >= 3:
                self.position_history.append(msg.feedback.positions)

    def target_callback(self, msg: JointTrajectory):
        """收到任何節點的指令，更新時間戳並直接轉發給硬體"""
        with self.action_lock:
            # 緊急停止期間丟棄所有新指令，手臂保持凍結
            if self.estop_engaged:
                return
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
            # 緊急停止期間：持續抓住當前位置，不進入放鬆邏輯
            if self.estop_engaged:
                if self.last_measured is not None:
                    self._publish_hold(self.last_measured)
                return

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