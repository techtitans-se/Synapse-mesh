#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import zenoh
import json
import time
import math

# --- Conflict Zone Definition ---
# Center of warehouse. If robot is within 2.0 meters of (0,0), it's in the aisle.
ZONE_RADIUS = 2.0 

class STLeaseNode(Node):
    def __init__(self):
        super().__init__('st_lease_node')
        
        self.robot_id = self.declare_parameter('robot_id', 'amr_1').value
        self.base_priority = self.declare_parameter('priority', 1.0).value
        
        self.get_logger().info(f"[{self.robot_id}] ST-Lease Autonomous Navigator Starting (Priority: {self.base_priority})")
        
        # --- ROS 2 Interfaces ---
        self.odom_sub = self.create_subscription(Odometry, f'/{self.robot_id}/odom', self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, f'/{self.robot_id}/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, f'/{self.robot_id}/st_lease_status', 10)
        self.bidding_pub = self.create_publisher(String, '/fms/st_lease_bidding', 10)
        
        self.dashboard_cmd_sub = self.create_subscription(String, '/fms/dashboard_cmd', self.dashboard_cmd_callback, 10)
        
        # --- Zenoh Setup (Decentralized Mesh) ---
        conf = zenoh.Config()
        self.z_session = zenoh.open(conf)
        self.lease_req_pub = self.z_session.declare_publisher(f"synapse_mesh/{self.robot_id}/lease_request")
        self.lease_sub = self.z_session.declare_subscriber("synapse_mesh/*/lease_request", self.mesh_callback)
        
        # --- Autonomous State ---
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.target_x = None
        self.target_y = None
        
        self.state = "IDLE"  # IDLE, APPROACHING, WAITING_FOR_LEASE, CROSSING, FAULT_STALLED
        self.request_time = 0.0
        self.lease_granted_time = 0.0
        self.active_leases = {}
        self.epoch = 1
        
        # Scenario Flags
        self.fault_stall_active = False
        
        # Control Loop at 10Hz
        self.create_timer(0.1, self.control_loop)
        
        # Status Publisher for Dashboard at 2Hz
        self.create_timer(0.5, self.publish_status)

    def ui_log(self, msg, is_warn=False):
        if is_warn:
            self.get_logger().warn(msg)
        else:
            self.get_logger().info(msg)
        self.bidding_pub.publish(String(data=f"[{self.robot_id}] {msg}"))

    def publish_status(self):
        status = {
            "robot_id": self.robot_id,
            "state": self.state,
            "x": self.x,
            "y": self.y,
            "target_x": self.target_x,
            "target_y": self.target_y,
            "epoch": self.epoch
        }
        self.status_pub.publish(String(data=json.dumps(status)))

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def dashboard_cmd_callback(self, msg):
        cmd = msg.data
        self.get_logger().info(f"[{self.robot_id}] Received Dashboard Cmd: {cmd}")
        
        if cmd == "RESET_HOME":
            self.state = "IDLE"
            self.target_x = None
            self.target_y = None
            self.fault_stall_active = False
            self.active_leases.clear()
            self._stop()
            
        elif cmd == "FORCE_CROSSING":
            # Set a target across the map based on current position
            if self.x < -1.0:
                self.target_x, self.target_y = 3.0, 0.0
            elif self.x > 1.0:
                self.target_x, self.target_y = -3.0, 0.0
            else:
                self.target_x, self.target_y = 0.0, 3.0
                
            self.state = "APPROACHING"
            self.fault_stall_active = False
            self.get_logger().info(f"[{self.robot_id}] Navigating to {self.target_x}, {self.target_y}")
            
        elif cmd.startswith("GOTO"):
            parts = cmd.split()
            if len(parts) == 4 and parts[1] == self.robot_id:
                try:
                    self.target_x = float(parts[2])
                    self.target_y = float(parts[3])
                    self.state = "APPROACHING"
                    self.fault_stall_active = False
                    self.get_logger().info(f"[{self.robot_id}] Admin assigned new task: GOTO {self.target_x}, {self.target_y}")
                except ValueError:
                    pass
            
        elif cmd == f"FAULT_STALL_{self.robot_id.upper()}":
            self.fault_stall_active = True
            self.get_logger().warn(f"[{self.robot_id}] INJECTED FAULT: Stalling in conflict zone!")

    def mesh_callback(self, sample):
        sender_id = str(sample.key_expr).split('/')[1]
        if sender_id == self.robot_id: return
        
        try:
            req = json.loads(sample.payload.to_bytes().decode('utf-8'))
            
            if req['action'] == "REQUEST":
                self.active_leases[sender_id] = req
            elif req['action'] == "RELEASE":
                if sender_id in self.active_leases:
                    del self.active_leases[sender_id]
        except Exception as e:
            self.get_logger().error(f"Mesh parse error: {e}")

    def broadcast_lease_request(self):
        req = {
            'epoch': self.epoch,
            'action': "REQUEST",
            'priority': self.base_priority,
            'p_eff': self.get_effective_priority(),
            'timestamp': self.request_time
        }
        self.lease_req_pub.put(json.dumps(req).encode('utf-8'))

    def broadcast_lease_release(self):
        req = {
            'epoch': self.epoch,
            'action': "RELEASE"
        }
        self.lease_req_pub.put(json.dumps(req).encode('utf-8'))
        self.epoch += 1

    def get_effective_priority(self):
        if self.state == "WAITING_FOR_LEASE":
            wait_time = time.time() - self.request_time
            return self.base_priority + (wait_time * 0.1) # Priority Aging Rule
        return self.base_priority

    def evaluate_lease(self):
        my_peff = self.get_effective_priority()
        granted = True
        
        for peer_id, req in self.active_leases.items():
            peer_peff = req.get('p_eff', req['priority'])
            
            # Rule 3: Priority Comparison & Tie-Breaker
            if peer_peff > my_peff:
                granted = False
                break
            elif abs(peer_peff - my_peff) < 0.05: # Tie
                if peer_id < self.robot_id: # Deterministic Alphanumeric ID Tie-Breaker
                    granted = False
                    break
        
        return granted

    def _stop(self):
        self.cmd_vel_pub.publish(Twist())

    def _drive_to_target(self):
        if self.fault_stall_active:
            self._stop()
            return

        # Simple waypoint router for cross-shaped map
        waypoint_x, waypoint_y = self.target_x, self.target_y
        
        # If target and current position are on different axes, route through the center (0,0)
        # to avoid clipping walls.
        if abs(self.target_y - self.y) > 1.0 and abs(self.target_x - self.x) > 1.0:
            dist_to_center = math.hypot(self.x, self.y)
            if dist_to_center > 0.5:
                waypoint_x, waypoint_y = 0.0, 0.0

        dx = waypoint_x - self.x
        dy = waypoint_y - self.y
        distance = math.hypot(dx, dy)
        
        if distance < 0.2:
            self._stop()
            self.state = "IDLE"
            self.get_logger().info(f"[{self.robot_id}] Reached destination.")
            return

        target_yaw = math.atan2(dy, dx)
        angle_diff = target_yaw - self.yaw
        
        # Normalize angle
        while angle_diff > math.pi: angle_diff -= 2 * math.pi
        while angle_diff < -math.pi: angle_diff += 2 * math.pi
        
        cmd = Twist()
        if abs(angle_diff) > 0.2:
            # Turn to face target
            cmd.angular.z = max(-0.5, min(0.5, angle_diff * 1.5))
        else:
            # Drive forward while adjusting angle
            cmd.linear.x = 0.3
            cmd.angular.z = angle_diff * 1.0
            
        self.cmd_vel_pub.publish(cmd)

    def control_loop(self):
        if self.state == "IDLE":
            self._stop()
            return
            
        dist_to_center = math.hypot(self.x, self.y)
        
        if self.state == "APPROACHING":
            if dist_to_center < ZONE_RADIUS + 0.3: # Approaching boundary
                self.ui_log("Reached conflict zone. Requesting ST-Lease...")
                self._stop()
                self.state = "WAITING_FOR_LEASE"
                self.request_time = time.time()
                self.broadcast_lease_request()
            else:
                self._drive_to_target()
                
        elif self.state == "WAITING_FOR_LEASE":
            self._stop()
            self.broadcast_lease_request() # Continually broadcast aging priority
            
            if self.evaluate_lease():
                self.ui_log("🏆 LEASE GRANTED. Proceeding into conflict zone.")
                self.state = "CROSSING"
                self.lease_granted_time = time.time()
                
        elif self.state == "CROSSING":
            if dist_to_center > ZONE_RADIUS + 0.5: # Exited the zone with a buffer
                self.ui_log("Exited conflict zone. Releasing lease.")
                self.broadcast_lease_release()
                self.state = "EXITING"
            else:
                # Rule 1 & 12: Lease Expiry
                if time.time() - self.lease_granted_time > 10.0:
                    self.ui_log("⚠️ LEASE EXPIRED! Max occupancy time exceeded!", is_warn=True)
                    # In a real system, the robot would trigger a deadlock recovery.
                    # Here, we release to let others know we are stuck.
                    self.broadcast_lease_release()
                    self.state = "FAULT_STALLED"
                    self._stop()
                else:
                    self._drive_to_target()
                    
        elif self.state == "EXITING":
            self._drive_to_target()
                    
        elif self.state == "FAULT_STALLED":
            self._stop() # Stuck in the middle

    def publish_status(self):
        msg = String()
        data = {
            "state": self.state,
            "priority": self.base_priority,
            "aging": max(0.0, time.time() - self.request_time) if self.state == "WAITING_FOR_LEASE" else 0.0,
            "p_eff": self.get_effective_priority()
        }
        msg.data = json.dumps(data)
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = STLeaseNode()
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
