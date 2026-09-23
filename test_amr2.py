import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time, json

class TestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        self.pub = self.create_publisher(String, '/synapse_amr_2/amr_dispatch', 10)
        self.sub = self.create_subscription(Twist, '/synapse_amr_2/cmd_vel', self.vel_cb, 10)
        self.msgs_received = 0
        self.timer = self.create_timer(1.0, self.publish_msg)

    def publish_msg(self):
        if self.msgs_received == 0:
            msg = String()
            msg.data = json.dumps({"target_load": "box_rack_a1_bay1", "dropoff_location": "QC Station"})
            self.pub.publish(msg)
            self.get_logger().info("Published dispatch to AMR 2")

    def vel_cb(self, msg):
        self.msgs_received += 1
        if self.msgs_received < 5:
            self.get_logger().info(f"AMR 2 moving: v={msg.linear.x}, w={msg.angular.z}")

def main():
    rclpy.init()
    node = TestNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
