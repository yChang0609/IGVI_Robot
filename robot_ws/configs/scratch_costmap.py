import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import time

class CostmapChecker(Node):
    def __init__(self):
        super().__init__('costmap_checker')
        self.sub = self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self.callback, 1)
        self.got_data = False

    def callback(self, msg):
        if self.got_data: return
        self.got_data = True
        
        width = msg.info.width
        height = msg.info.height
        data = msg.data
        
        lethal = sum(1 for c in data if c == 254)
        inflated = sum(1 for c in data if 0 < c < 254)
        free = sum(1 for c in data if c == 0)
        unknown = sum(1 for c in data if c == -1)
        
        print(f"Costmap Size: {width}x{height}")
        print(f"Lethal obstacles: {lethal}")
        print(f"Inflated/Warning: {inflated}")
        print(f"Free space: {free}")
        print(f"Unknown space: {unknown}")
        
        self.destroy_node()
        rclpy.shutdown()

def main():
    rclpy.init()
    node = CostmapChecker()
    t0 = time.time()
    while rclpy.ok() and not node.got_data and time.time() - t0 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    
    if not node.got_data:
        print("Failed to get costmap within 5 seconds.")

if __name__ == '__main__':
    main()
