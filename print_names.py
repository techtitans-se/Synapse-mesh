import rclpy
from gazebo_msgs.msg import ModelStates

def cb(msg):
    print(msg.name)
    rclpy.shutdown()

rclpy.init()
node = rclpy.create_node('print_names')
sub = node.create_subscription(ModelStates, '/gazebo/model_states', cb, 10)
rclpy.spin(node)
