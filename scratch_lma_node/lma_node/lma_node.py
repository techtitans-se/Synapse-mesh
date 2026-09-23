import rclpy
from rclpy.node import Node
import zenoh
import json
import math
from nav_msgs.msg import Odometry

class LocalityManager(Node):
    def __init__(self):
        super().__init__('lma_node')
        self.get_logger().info("Locality Manager Application (LMA) starting...")
        
        # ROS 2 Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        
        # Zenoh setup
        conf = zenoh.Config()
        self.z_session = zenoh.open(conf)
        
        # State
        self.robot_id = self.declare_parameter('robot_id', 'amr_1').value
        self.proximity_threshold = 6.0 # meters (subscribe if closer than this)
        self.neighbor_table = {} # id -> pose
        self.active_subscriptions = {} # id -> zenoh subscriber
        
        # Zenoh heartbeat publisher (low frequency)
        self.heartbeat_pub = self.z_session.declare_publisher(f"synapse_mesh/{self.robot_id}/heartbeat")
        
        # Zenoh heartbeat subscriber (listen to all robots)
        self.heartbeat_sub = self.z_session.declare_subscriber("synapse_mesh/*/heartbeat", self.heartbeat_callback)
        
        self.timer = self.create_timer(1.0, self.publish_heartbeat)
        self.current_pose = None
        self.get_logger().info(f"LMA Node for {self.robot_id} initialized with Zenoh.")
        
    def odom_callback(self, msg):
        self.current_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y
        }
        
    def publish_heartbeat(self):
        if self.current_pose:
            payload = json.dumps(self.current_pose)
            self.heartbeat_pub.put(payload.encode('utf-8'))
            
    def heartbeat_callback(self, sample):
        key = str(sample.key_expr)
        parts = key.split('/')
        if len(parts) >= 3:
            sender_id = parts[1]
            if sender_id == self.robot_id:
                return # Ignore self
                
            payload = json.loads(sample.payload.to_bytes().decode('utf-8'))
            self.update_neighbor_table(sender_id, payload)
            
    def update_neighbor_table(self, neighbor_id, pose):
        if not self.current_pose: return
        
        dist = math.hypot(self.current_pose['x'] - pose['x'], self.current_pose['y'] - pose['y'])
        
        if dist <= self.proximity_threshold:
            if neighbor_id not in self.neighbor_table:
                self.get_logger().info(f"Neighbor {neighbor_id} entered locality zone (dist: {dist:.2f}m). Subscribing to state.")
                self.neighbor_table[neighbor_id] = pose
                # Dynamically subscribe to high-frequency state
                sub = self.z_session.declare_subscriber(f"synapse_mesh/{neighbor_id}/state", self.state_callback)
                self.active_subscriptions[neighbor_id] = sub
        else:
            if neighbor_id in self.neighbor_table:
                self.get_logger().info(f"Neighbor {neighbor_id} left locality zone (dist: {dist:.2f}m). Unsubscribing.")
                del self.neighbor_table[neighbor_id]
                if neighbor_id in self.active_subscriptions:
                    self.active_subscriptions[neighbor_id].undeclare()
                    del self.active_subscriptions[neighbor_id]
                    
    def state_callback(self, sample):
        # Handle high-frequency state updates from neighbors for ST-Lease and safety
        pass

def main(args=None):
    rclpy.init(args=args)
    node = LocalityManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.z_session.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
