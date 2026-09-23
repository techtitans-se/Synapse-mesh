import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import time

class Monitor(Node):
    def __init__(self):
        super().__init__('monitor')
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.count = 0
    def odom_callback(self, msg):
        self.count += 1
        if self.count % 10 == 0:
            print(f"[{time.time()}] x: {msg.pose.pose.position.x:.2f}, y: {msg.pose.pose.position.y:.2f}, twist_x: {msg.twist.twist.linear.x:.2f}, twist_z: {msg.twist.twist.angular.z:.2f}")

rclpy.init()
node = Monitor()
print("Monitoring...")
for i in range(100):
    rclpy.spin_once(node, timeout_sec=0.1)
rclpy.shutdown()
