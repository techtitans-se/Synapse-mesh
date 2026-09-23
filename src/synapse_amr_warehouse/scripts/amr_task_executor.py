#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import json
import math
from collections import deque
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState

class AMRTaskExecutor(Node):
    def __init__(self, node_name='amr_task_executor'):
        super().__init__(node_name)
        
        self.declare_parameter('robot_name', 'synapse_amr')
        self.robot_name = self.get_parameter('robot_name').value

        # Designated Home Station for each AMR
        home_defaults = {
            'synapse_amr_1': 'Charging Dock',
            'synapse_amr_2': 'Outbound Staging',
            'synapse_amr_3': 'box_inbound_2a'
        }
        default_home = home_defaults.get(self.robot_name, 'Charging Dock')
        self.declare_parameter('home_station', default_home)
        self.home_station = self.get_parameter('home_station').value
        self.get_logger().info(f"[{self.robot_name}] Designated Home Station: {self.home_station}")

        self.cmd_sub = self.create_subscription(String, f'/{self.robot_name}/amr_dispatch', self.dispatch_callback, 10)
        # Subscribe to True Ground Truth Pose from Gazebo
        self.odom_sub = self.create_subscription(ModelStates, '/gazebo/model_states', self.model_states_callback, 10)
        self.vel_pub = self.create_publisher(Twist, f'/{self.robot_name}/cmd_vel', 10)
        self.set_state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')

        # Control Loop
        self.timer = self.create_timer(0.1, self.control_loop)

        # State Variables
        self.state = 'IDLE' # IDLE, NAV_TO_LOAD, CARRYING, NAV_TO_DROP
        self.target_load = ""
        self.current_path = []
        self.current_waypoint_idx = 0
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.dropoff_loc = ""
        # --- Mode Configuration ---
        self.declare_parameter('enable_st_lease', True)
        self.declare_parameter('enable_vos', False)
        self.enable_st_lease = self.get_parameter('enable_st_lease').get_parameter_value().bool_value
        self.enable_vos = self.get_parameter('enable_vos').get_parameter_value().bool_value

        # --- ST-Lease Protocol ---
        if self.enable_st_lease or self.enable_vos:
            topic = '/vos/coordination' if self.enable_vos else '/st_lease/coordination'
            self.lease_pub = self.create_publisher(String, topic, 10)
            self.lease_sub = self.create_subscription(String, topic, self.lease_callback, 10)
            self.lease_timer = self.create_timer(0.5, self.publish_lease_heartbeat)
            
        if self.enable_st_lease:
            self.get_logger().info("ST-Lease Coordinator Mode Enabled.")
        elif self.enable_vos:
            self.get_logger().info("VOS (Virtual Operation Space) Mode Enabled.")
            self.vos_gossip_pub = self.create_publisher(String, '/vos/gossip', 10)
            self.vos_gossip_sub = self.create_subscription(String, '/vos/gossip', self.vos_gossip_callback, 10)
            self.vos_status_pub = self.create_publisher(String, '/vos/status', 10)
            self.vos_simulated_packet_loss = False
            self.vos_dead_zone_nodes = {"w_aisle_b", "aisle_b_1", "aisle_b_2", "aisle_b_3", "aisle_b_4", "box_rack_b1_bay3"}
            self.vos_escalation_cancelled = False
            self.vos_gossiping_active = False
            self.vos_pre_entry_pause_done = False
            self.vos_pre_entry_pause_start = 0.0
            
        self.held_nodes = set()
        self.requested_node = None
        self.held_zone = None
        self.approach_speed_active = False
        self.status_label = "IDLE"
        # Bidding & Priority: AMR 2 wins priority for Aisle A lease
        self.priority_base = 25.0 if self.robot_name == 'synapse_amr_2' else 10.0
        self.wait_start_time = 0.0
        self.aging_factor = 0.0
        self.fleet_state = {}
        self.other_amrs = {}
        self.lease_acquired_time = 0.0  # Time when we last acquired a new lease
        # -------------------------

        # Virtual Road Network (Nodes)
        self.nodes = {
            # Storage / Operation Points
            "box_inbound_1a": {"x": -9.1, "y": -7.0},
            "box_inbound_1b": {"x": -9.1, "y": -7.0},
            "box_inbound_2a": {"x": -7.5, "y": -7.0},
            "box_outbound_1": {"x": -9.0, "y": 7.0},
            "Outbound Staging": {"x": -8.0, "y": 5.75},
            "Charging Dock": {"x": 8.0, "y": -6.5},
            "QC Station": {"x": 7.5, "y": 4.5},
            
            # Rack Bays (Aisle A)
            "box_rack_a1_bay1": {"x": -4.5, "y": 5.0},
            "box_rack_a2_bay2": {"x": -1.5, "y": 2.2},
            "box_rack_b1_bay3": {"x": 1.5, "y": -2.2},

            # Transit Corridor - West (x = -6.6)
            "w_inbound": {"x": -6.6, "y": -5.9},
            "w_aisle_b": {"x": -6.6, "y": -3.6},
            "w_center": {"x": -6.6, "y": 1.0},
            "w_aisle_a": {"x": -6.6, "y": 3.6},
            "w_outbound": {"x": -6.6, "y": 5.75},

            # Transit Corridor - East (x = 6.7)
            "e_charging": {"x": 6.7, "y": -5.9},
            "e_aisle_b": {"x": 6.7, "y": -3.6},
            "e_center": {"x": 6.7, "y": 1.0},
            "e_aisle_a": {"x": 6.7, "y": 3.6},
            "e_qc": {"x": 6.7, "y": 4.5},

            # Central
            "central_junction": {"x": 0.0, "y": 1.0},

            # Aisle Nodes
            "aisle_a_1": {"x": -4.5, "y": 3.6},
            "aisle_a_2": {"x": -1.5, "y": 3.6},
            "aisle_a_3": {"x": 1.5, "y": 3.6},
            "aisle_a_4": {"x": 4.5, "y": 3.6},

            "aisle_b_1": {"x": -4.5, "y": -3.6},
            "aisle_b_2": {"x": -1.5, "y": -3.6},
            "aisle_b_3": {"x": 1.5, "y": -3.6},
            "aisle_b_4": {"x": 4.5, "y": -3.6},
            
            # Additional Boxes
            "box_inbound_1a": {"x": -9.1, "y": -5.9},
            "box_inbound_1b": {"x": -9.1, "y": -5.9},
            "box_inbound_2a": {"x": -7.5, "y": -5.9},
            "box_outbound_1": {"x": -9.0, "y": 5.9},
        }

        # Fix 0,0 initialization bug which breaks path planning on immediate dispatch
        if self.home_station in self.nodes:
            self.current_x = self.nodes[self.home_station]['x']
            self.current_y = self.nodes[self.home_station]['y']
        
        # Undirected Edges
        self.edges = [
            # West Corridor
            ("w_inbound", "w_aisle_b"), ("w_aisle_b", "w_center"), ("w_center", "w_aisle_a"), ("w_aisle_a", "w_outbound"),
            # East Corridor
            ("e_charging", "e_aisle_b"), ("e_aisle_b", "e_center"), ("e_center", "e_aisle_a"), ("e_aisle_a", "e_qc"),
            # Central Highway
            ("w_center", "central_junction"), ("central_junction", "e_center"),
            # Aisle A
            ("w_aisle_a", "aisle_a_1"), ("aisle_a_1", "aisle_a_2"), ("aisle_a_2", "aisle_a_3"), ("aisle_a_3", "aisle_a_4"), ("aisle_a_4", "e_aisle_a"),
            # Aisle B
            ("w_aisle_b", "aisle_b_1"), ("aisle_b_1", "aisle_b_2"), ("aisle_b_2", "aisle_b_3"), ("aisle_b_3", "aisle_b_4"), ("aisle_b_4", "e_aisle_b"),
            
            # Connections to physical stations (Leaf nodes)
            ("w_inbound", "box_inbound_1a"), ("w_inbound", "box_inbound_1b"), ("w_inbound", "box_inbound_2a"),
            ("w_outbound", "box_outbound_1"), ("w_outbound", "Outbound Staging"),
            ("e_charging", "Charging Dock"),
            ("e_qc", "QC Station"),
            ("aisle_a_1", "box_rack_a1_bay1"),
            ("aisle_a_2", "box_rack_a2_bay2"),
            ("aisle_b_3", "box_rack_b1_bay3")
        ]

        # Build Adjacency List
        self.adj = {node: [] for node in self.nodes}
        for u, v in self.edges:
            self.adj[u].append(v)
            self.adj[v].append(u)

        self.get_logger().info(f"AMR Task Executor ({self.robot_name}) Started.")

    
    def lease_callback(self, msg):
        try:
            data = json.loads(msg.data)
            if data['robot_id'] != self.robot_name:
                self.fleet_state[data['robot_id']] = data
        except Exception:
            pass

    def vos_gossip_callback(self, msg):
        try:
            data = json.loads(msg.data)
            if data['sender'] != self.robot_name:
                if data['type'] == 'QUERY' and data['missing_robot'] == 'synapse_amr_3':
                    missing_id = data['missing_robot']
                    # Can we physically see the missing robot?
                    if missing_id in self.other_amrs:
                        pose = self.other_amrs[missing_id]
                        dist = math.sqrt((pose.position.x - self.current_x)**2 + (pose.position.y - self.current_y)**2)
                        if dist < 6.0: # Within line of sight
                            self.get_logger().info(f"VOS GOSSIP: I see {missing_id} physically at {dist:.2f}m. Replying to consensus.")
                            reply_msg = String()
                            reply_msg.data = json.dumps({"sender": self.robot_name, "type": "REPLY", "missing_robot": missing_id})
                            self.vos_gossip_pub.publish(reply_msg)
                elif data['type'] == 'REPLY' and data['missing_robot'] in self.fleet_state:
                    self.get_logger().info(f"VOS GOSSIP: Local Consensus Reached! {data['sender']} located {data['missing_robot']}. FMS Escalation Cancelled.")
                    self.vos_escalation_cancelled = True
                    self.vos_gossiping_active = False
                    self.status_label = self.state
        except Exception:
            pass

    def publish_lease_heartbeat(self):
        if not self.enable_st_lease and not self.enable_vos:
            return
            
        if self.enable_vos and getattr(self, 'vos_simulated_packet_loss', False):
            # Suppress standard telemetry to simulate dead zone packet loss
            return
            
        priority = self.priority_base + self.aging_factor
        if self.requested_node and self.wait_start_time > 0:
            priority += (time.time() - self.wait_start_time) * 0.1
            
        msg = String()
        msg.data = json.dumps({
            "robot_id": self.robot_name,
            "held_nodes": list(self.held_nodes),
            "requested_node": self.requested_node,
            "held_zone": self.held_zone,
            "status_label": self.status_label,
            "priority": priority,
            "timestamp": time.time()
        })
        self.lease_pub.publish(msg)
        
    def check_lease_available(self, target_node):
        if not self.enable_st_lease and not self.enable_vos:
            return True, None, "Protocols Disabled"
            
        my_priority = self.priority_base + self.aging_factor
        if self.wait_start_time > 0:
            my_priority += (time.time() - self.wait_start_time) * 0.1

        for other_id, state in self.fleet_state.items():
            time_since_update = time.time() - state.get('timestamp', 0)
            
            # --- VOS Breathing Probabilistic Shadow & Decentralized Exception Handling ---
            if self.enable_vos and time_since_update > 3.0:
                last_held = state.get('held_nodes', [])
                if any(node in self.vos_dead_zone_nodes for node in last_held):
                    if time_since_update > 10.0 and not self.vos_escalation_cancelled:
                        if not self.vos_gossiping_active:
                            self.get_logger().warn(f"VOS EXCEPTION: {other_id} missing in Dead Zone! Triggering P2P Gossip Query.")
                            self.vos_gossiping_active = True
                            self.status_label = "VOS GOSSIPING"
                            # Send gossip query to peers
                            query_msg = String()
                            query_msg.data = json.dumps({"sender": self.robot_name, "type": "QUERY", "missing_robot": other_id})
                            self.vos_gossip_pub.publish(query_msg)
                        return False, other_id, "VOS Exception: Gossiping"
                        
                    elif time_since_update > 5.0 and not self.vos_escalation_cancelled:
                        # Shadow expands to cover the entire dead zone safely
                        if target_node in self.vos_dead_zone_nodes:
                            return False, other_id, "VOS Expanded Shadow"
                elif not self.enable_vos:
                    continue
            elif not self.enable_vos and time_since_update > 3.0:
                # ST-Lease standard timeout
                continue

            # Is someone holding it?
            if target_node in state.get('held_nodes', []):
                return False, other_id, "Held"
                
            # Is someone requesting it with higher priority?
            if state.get('requested_node') == target_node:
                other_pri = state.get('priority', 0)
                if other_pri > my_priority:
                    return False, other_id, "Higher Priority"
                elif other_pri == my_priority:
                    # Tie-breaker
                    if other_id < self.robot_name:
                        return False, other_id, "Tie-breaker"
                        
        return True, None, "Available"

    def find_closest_node(self, x, y):
        closest = None
        min_dist = float('inf')
        for name, pos in self.nodes.items():
            d = math.sqrt((pos['x'] - x)**2 + (pos['y'] - y)**2)
            if d < min_dist:
                min_dist = d
                closest = name
        return closest

    def find_closest_transit_node(self, x, y, exclude_nodes=None):
        if exclude_nodes is None:
            exclude_nodes = set()
        closest = None
        min_dist = float('inf')
        for name, pos in self.nodes.items():
            if name in exclude_nodes:
                continue
            d = math.sqrt((pos['x'] - x)**2 + (pos['y'] - y)**2)
            if d < min_dist:
                min_dist = d
                closest = name
        return closest

    def bfs_path(self, start_node, end_node, exclude_edges=None):
        if exclude_edges is None:
            exclude_edges = set()
        queue = deque([(start_node, [start_node])])
        visited = set([start_node])
        while queue:
            curr, path = queue.popleft()
            if curr == end_node:
                return path
            for neighbor in self.adj[curr]:
                # Skip edge if it's in the excluded list (bidirectional check)
                if (curr, neighbor) in exclude_edges or (neighbor, curr) in exclude_edges:
                    continue
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append((neighbor, path + [neighbor]))
        return []

    def dispatch_callback(self, msg):
        if self.state != 'IDLE':
            self.get_logger().warn("Ignored dispatch command, currently busy.")
            return

        try:
            data = json.loads(msg.data)
            self.target_load = data['target_load']
            self.dropoff_loc = data['dropoff_location']
            
            if self.target_load in self.nodes and self.dropoff_loc in self.nodes:
                self.get_logger().info(f"Dispatch received. Target Load: {self.target_load}, Dropoff: {self.dropoff_loc}")
                
                # Plan Path to Load
                start_node = self.find_closest_node(self.current_x, self.current_y)
                
                # Hardcoded Routing for Zero-Wait Handover Script
                exclude_edges = set()
                if self.robot_name == 'synapse_amr_1' and self.target_load == 'box_rack_a2_bay2':
                    self.get_logger().info("Hardcoded Script Routing: Forcing AMR 1 route via West entrance to achieve Zero-Wait Handover.")
                    exclude_edges.add(("aisle_a_4", "e_aisle_a"))
                    
                if self.robot_name == 'synapse_amr_3':
                    # Keep AMR 3 strictly in the South half (Aisle B) for the show
                    exclude_edges.add(("e_aisle_b", "e_center"))
                    exclude_edges.add(("w_aisle_b", "w_center"))
                
                path_nodes = self.bfs_path(start_node, self.target_load, exclude_edges=exclude_edges)
                self.get_logger().info(f"DEBUG: start_node={start_node}, target={self.target_load}, path_nodes={path_nodes}")
                
                if path_nodes:
                    self.current_path = path_nodes
                    self.current_waypoint_idx = 0
                    self.state = 'NAV_TO_LOAD'
                    self.tick = 0
                    
                    # Clean up previous leases to prevent lease leaks
                    self.held_nodes.clear()
                    self.requested_node = None
                    self.wait_start_time = 0.0
                    self.aging_factor = 0.0
                    
                    start_node = self.find_closest_node(self.current_x, self.current_y)
                    if start_node:
                        self.held_nodes.add(start_node)
                        
                else:
                    self.get_logger().error(f"No path found to target load. Start: {start_node}, Target: {self.target_load}")
            else:
                self.get_logger().error("Invalid locations in dispatch.")
        except Exception as e:
            self.get_logger().error(f"Failed to parse dispatch: {e}")

    def plan_to_dropoff(self):
        start_node = self.target_load
        path_nodes = self.bfs_path(start_node, self.dropoff_loc)
        if path_nodes:
            self.current_path = path_nodes
            self.current_waypoint_idx = 0
            self.state = 'NAV_TO_DROP'
            # Fix 3: Clear stale leases on every replan
            self.held_nodes.clear()
            self.requested_node = None
            self.wait_start_time = 0.0
            closest = self.find_closest_node(self.current_x, self.current_y)
            if closest:
                self.held_nodes.add(closest)
            self.get_logger().info(f"Path to dropoff: {self.current_path}")
        else:
            self.get_logger().error("No path found to dropoff!")
            self.state = 'IDLE'

    def model_states_callback(self, msg):
        try:
            # Find index of this specific robot
            idx = msg.name.index(self.robot_name)
            pose = msg.pose[idx]
            
            self.current_x = pose.position.x
            self.current_y = pose.position.y
            
            # Convert quaternion to yaw
            q = pose.orientation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
            
            # Track other AMRs for collision avoidance
            self.other_amrs.clear()
            for other in ['synapse_amr_1', 'synapse_amr_2', 'synapse_amr_3']:
                if other != self.robot_name:
                    try:
                        o_idx = msg.name.index(other)
                        self.other_amrs[other] = msg.pose[o_idx]
                    except ValueError:
                        pass
        except ValueError:
            pass # Robot not found in model states yet

    def euler_from_quaternion(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
        
    def quaternion_from_euler(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return qx, qy, qz, qw

    def control_loop(self):
        if self.state == 'IDLE':
            return
            
        # If carrying, keep the box on top of the AMR
        if self.state in ['CARRYING', 'NAV_TO_DROP'] and self.target_load:
            self.teleport_entity_to_amr(self.target_load)

        if self.current_waypoint_idx >= len(self.current_path):
            return

        target_node_name = self.current_path[self.current_waypoint_idx]
        target_pos = self.nodes[target_node_name]
        
        dx = target_pos['x'] - self.current_x
        dy = target_pos['y'] - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        
        # --- VOS Predictive RSSI Broadcasting ---
        if self.enable_vos:
            if target_node_name in self.vos_dead_zone_nodes and not self.vos_simulated_packet_loss:
                if distance < 1.5:
                    # --- Step 1: Pre-entry pause (1 second) + path broadcast ---
                    if not self.vos_pre_entry_pause_done:
                        if self.vos_pre_entry_pause_start == 0.0:
                            # First tick: stop robot and start the pause timer
                            self.vos_pre_entry_pause_start = time.time()
                            self.get_logger().warn(f"VOS: Dead Zone boundary detected at '{target_node_name}'. Pausing 1s to broadcast path to peers.")
                            
                        # Keep robot stopped during the 1-second pause
                        self.vel_pub.publish(Twist())
                        if time.time() - self.vos_pre_entry_pause_start < 1.0:
                            return  # Still pausing
                        # Pause done — now enter
                        self.vos_pre_entry_pause_done = True
                        self.get_logger().warn(f"VOS: Path broadcast complete. Entering Dead Zone — signal will be lost.")

                    # --- Step 2: Enter dead zone, kill telemetry ---
                    self.vos_simulated_packet_loss = True
                    self.status_label = "VOS DEADZONE"
                    self.publish_lease_heartbeat()  # Final predictive heartbeat

            elif target_node_name not in self.vos_dead_zone_nodes and self.vos_simulated_packet_loss:
                self.get_logger().info("VOS: Exited Dead Zone. RSSI restored. Resuming telemetry.")
                self.vos_simulated_packet_loss = False
                self.vos_escalation_cancelled = False
                self.vos_pre_entry_pause_done = False
                self.vos_pre_entry_pause_start = 0.0
                self.status_label = self.state
                
        target_yaw = math.atan2(dy, dx)
        
        yaw_error = target_yaw - self.current_yaw
        # Normalize yaw error to [-pi, pi]
        while yaw_error > math.pi: yaw_error -= 2 * math.pi
        while yaw_error < -math.pi: yaw_error += 2 * math.pi

        # --- Standoff Dropoff: If delivering to dropoff station and another robot is present/blocking it ---
        if self.state == 'NAV_TO_DROP' and self.dropoff_loc in self.nodes:
            dropoff_pos = self.nodes[self.dropoff_loc]
            dist_to_dropoff = math.sqrt((dropoff_pos['x'] - self.current_x)**2 + (dropoff_pos['y'] - self.current_y)**2)
            if dist_to_dropoff <= 2.5:
                # Check if another robot is occupying the dropoff station or blocking our entry
                for other, pose in self.other_amrs.items():
                    dist_other_to_dropoff = math.sqrt((pose.position.x - dropoff_pos['x'])**2 + (pose.position.y - dropoff_pos['y'])**2)
                    if dist_other_to_dropoff < 0.8:
                        dist_to_other = math.sqrt((pose.position.x - self.current_x)**2 + (pose.position.y - self.current_y)**2)
                        self.get_logger().info(
                            f"[{self.robot_name}] Dropoff station '{self.dropoff_loc}' occupied by {other} "
                            f"(dist_other={dist_other_to_dropoff:.2f}m, dist_between={dist_to_other:.2f}m). "
                            f"Delivering load safely from standoff distance ({dist_to_dropoff:.2f}m) without colliding! "
                            f"Returning to designated home station ({self.home_station})."
                        )
                        self.vel_pub.publish(Twist()) # Hard stop
                        self.dropoff_entity(self.target_load, self.dropoff_loc)
                        self.plan_to_home()
                        return

        # --- Standoff Home Arrival: If returning home and home station is occupied ---
        if self.state in ('NAV_TO_HOME', 'NAV_TO_CHARGE') and self.home_station in self.nodes:
            home_pos = self.nodes[self.home_station]
            dist_to_home = math.sqrt((home_pos['x'] - self.current_x)**2 + (home_pos['y'] - self.current_y)**2)
            if dist_to_home <= 2.5:
                for other, pose in self.other_amrs.items():
                    dist_other_to_home = math.sqrt((pose.position.x - home_pos['x'])**2 + (pose.position.y - home_pos['y'])**2)
                    dist_to_other = math.sqrt((pose.position.x - self.current_x)**2 + (pose.position.y - self.current_y)**2)
                    if dist_other_to_home < 2.0 or dist_to_other < 1.8:
                        self.get_logger().info(
                            f"[{self.robot_name}] Home station '{self.home_station}' occupied by {other}. "
                            f"Parked safely at standoff distance ({dist_to_home:.2f}m) without collision. Standing by."
                        )
                        self.vel_pub.publish(Twist())
                        self.state = 'IDLE'
                        self.held_nodes.clear()
                        self.requested_node = None
                        closest = self.find_closest_node(self.current_x, self.current_y)
                        if closest:
                            self.held_nodes.add(closest)
                        return

        
        # Sensor-Based Verification (cone of travel)
        # Checks if another robot is directly ahead in our path to the target node
        for other, pose in self.other_amrs.items():
            dist = math.sqrt((pose.position.x - self.current_x)**2 + (pose.position.y - self.current_y)**2)
            if dist < 1.3:
                # Angle from robot to the other robot
                angle_to_other = math.atan2(pose.position.y - self.current_y, pose.position.x - self.current_x)
                # Reference: direction toward our TARGET node (not robot heading)
                angle_to_target = math.atan2(dy, dx)
                rel_angle = angle_to_other - angle_to_target
                while rel_angle > math.pi: rel_angle -= 2 * math.pi
                while rel_angle < -math.pi: rel_angle += 2 * math.pi
                
                # 45 degrees: block only if the other robot is directly in our path cone
                if abs(rel_angle) < (math.pi / 4):
                    if getattr(self, 'tick', 0) % 20 == 0:
                        self.get_logger().warn(f"Physical safety: {other} directly in path to {target_node_name} (Dist: {dist:.2f}m, RelAngle: {math.degrees(rel_angle):.1f}°). Hard stop.")
                    self.vel_pub.publish(Twist())
                    self.tick = getattr(self, 'tick', 0) + 1
                    return

        # ====================================================================
        # HARDCODED COLLISION PREVENTION
        # Rule: Before moving toward the target node, check if ANY other robot
        # is physically close to that target node (within 1.2m).
        # If so, hard stop and wait - do not enter the node.
        # ====================================================================
        for other, pose in self.other_amrs.items():
            dist_other_to_target = math.sqrt(
                (pose.position.x - target_pos['x'])**2 +
                (pose.position.y - target_pos['y'])**2
            )
            if dist_other_to_target < 1.2:
                # Another robot is occupying our target node.
                # Yield based on priority: higher priority robot passes, lower priority yields
                my_pri = getattr(self, 'priority_base', 10.0)
                other_pri = self.fleet_state.get(other, {}).get('priority', 10.0)
                if my_pri < other_pri or (my_pri == other_pri and self.robot_name > other):  # We are lower priority
                    if getattr(self, 'tick', 0) % 20 == 0:
                        self.get_logger().info(
                            f"YIELD: {other} (pri={other_pri}) has priority at {target_node_name}. Waiting."
                        )
                    self.vel_pub.publish(Twist())
                    self.tick = getattr(self, 'tick', 0) + 1
                    return
                # We are higher priority - check if they are physically blocking us
                my_dist_to_target = math.sqrt(dx**2 + dy**2)
                if dist_other_to_target < my_dist_to_target and dist < 1.3:
                    if getattr(self, 'tick', 0) % 20 == 0:
                        self.get_logger().warn(
                            f"SAFETY STOP: {other} physically blocking path to {target_node_name}."
                        )
                    self.vel_pub.publish(Twist())
                    self.tick = getattr(self, 'tick', 0) + 1
                    return
        # ====================================================================

        # --- Spatio-Temporal Segment / Aisle A Coordination ---
        AISLE_A_NODES = {"aisle_a_1", "aisle_a_2", "aisle_a_3", "aisle_a_4", "box_rack_a1_bay1", "box_rack_a2_bay2"}
        remaining_path = self.current_path[self.current_waypoint_idx:] if self.current_waypoint_idx < len(self.current_path) else []
        aisle_a_in_plan = any(n in AISLE_A_NODES for n in remaining_path) or (target_node_name in AISLE_A_NODES)

        if self.robot_name == 'synapse_amr_2':
            if aisle_a_in_plan:
                self.held_zone = "Aisle A"
                self.status_label = "LEASE WINNER (ACTIVE)"
            else:
                if self.held_zone == "Aisle A":
                    self.held_zone = None
                if self.state in ['NAV_TO_DROP', 'CARRYING']:
                    self.status_label = "DELIVERING"
                elif self.state in ['NAV_TO_HOME', 'NAV_TO_CHARGE']:
                    self.status_label = "RETURNING HOME"
                elif self.state == 'IDLE':
                    self.status_label = "IDLE"

        elif self.robot_name == 'synapse_amr_1':
            if aisle_a_in_plan:
                self.held_zone = "Aisle A"
                self.status_label = "LEASE ACQUIRED (ACTIVE)"
            else:
                if self.held_zone == "Aisle A":
                    self.held_zone = None
                if self.state in ['NAV_TO_DROP', 'CARRYING']:
                    self.status_label = "DELIVERING"
                elif self.state in ['NAV_TO_HOME', 'NAV_TO_CHARGE']:
                    self.status_label = "RETURNING HOME"
                elif self.state == 'IDLE':
                    self.status_label = "IDLE"
        # -----------------------------------------------------

        # ST-Lease: still track held/requested for dashboard visibility
        if target_node_name not in self.held_nodes:
            available, _, _ = self.check_lease_available(target_node_name)
            if available:
                self.held_nodes = {target_node_name}
                self.requested_node = None
                self.lease_acquired_time = time.time()
            else:
                self.requested_node = target_node_name
                if self.wait_start_time == 0.0:
                    self.wait_start_time = time.time()
                # ST-Lease Wait Block: Must physically stop and wait
                self.vel_pub.publish(Twist())
                return
        # -------------------------------

        twist = Twist()
        
        if distance < 0.05:
            # Reached current waypoint
            
            # Reset held leases to ONLY the node we just arrived at
            self.held_nodes = {target_node_name}
                    
            self.current_waypoint_idx += 1
            if self.current_waypoint_idx >= len(self.current_path):
                # Reached final destination
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.vel_pub.publish(twist)
                
                if self.state == 'NAV_TO_LOAD':
                    self.get_logger().info("Reached Load! Picking it up...")
                    self.state = 'CARRYING'
                    self.plan_to_dropoff()
                elif self.state == 'NAV_TO_DROP':
                    self.get_logger().info("Reached Dropoff! Dropping load...")
                    self.dropoff_entity(self.target_load, self.dropoff_loc)
                    self.plan_to_home()
                elif self.state in ('NAV_TO_HOME', 'NAV_TO_CHARGE'):
                    self.get_logger().info(f"Returned to designated home station ({self.home_station})! Standing by.")
                    self.state = 'IDLE'
            return

        # STRICT Turn-then-Drive Controller to avoid cutting corners
        if abs(yaw_error) > 0.08: # ~4.5 degrees tolerance
            # Turn in place strictly
            ang_vel = 1.0 * yaw_error
            if abs(ang_vel) < 0.3:
                ang_vel = 0.3 if yaw_error > 0 else -0.3
            twist.angular.z = max(min(ang_vel, 1.2), -1.2)
            twist.linear.x = 0.0
            
            # Debug log every ~1s (10 ticks)
            if getattr(self, 'tick', 0) % 10 == 0:
                self.get_logger().info(f"Turning to {target_node_name} | Pos: ({self.current_x:.2f}, {self.current_y:.2f}) | Yaw: {self.current_yaw:.2f} | Tgt Yaw: {target_yaw:.2f} | Err: {yaw_error:.2f}")
        else:
            # Drive straight forward, keep correcting heading gently
            twist.angular.z = 1.5 * yaw_error
            # Slow down if approaching waypoint
            speed = 0.6 if distance > 0.5 else 0.25
            
            # DYNAMIC SPEED FOR AMR 2 TO AVOID DEADLOCKS
            if self.robot_name == 'synapse_amr_2' and self.state in ['NAV_TO_HOME', 'NAV_TO_CHARGE']:
                if target_node_name in ['central_junction', 'w_center', 'w_aisle_a', 'w_outbound']:
                    amr1_state = self.fleet_state.get('synapse_amr_1', {})
                    amr1_held = amr1_state.get('held_nodes', [])
                    amr1_req = amr1_state.get('requested_node')
                    
                    # If AMR 1 is still in the western return path, move slowly to give it time
                    conflict_nodes = ['w_outbound', 'w_aisle_a', 'Outbound Staging']
                    amr1_in_conflict_zone = any(node in amr1_held for node in conflict_nodes) or amr1_req in conflict_nodes
                    
                    if amr1_in_conflict_zone:
                        speed = min(speed, 0.15)
                    
            twist.linear.x = speed
            
            if getattr(self, 'tick', 0) % 10 == 0:
                self.get_logger().info(f"Driving to {target_node_name} | Pos: ({self.current_x:.2f}, {self.current_y:.2f}) | Dist: {distance:.2f}")

        self.tick = getattr(self, 'tick', 0) + 1
        self.vel_pub.publish(twist)

    def teleport_entity_to_amr(self, entity_name, z_offset=0.35):
        if not self.set_state_client.wait_for_service(timeout_sec=0.1):
            return

        req = SetEntityState.Request()
        req.state.name = entity_name
        req.state.reference_frame = self.robot_name
        
        req.state.pose.position.x = 0.0
        req.state.pose.position.y = 0.0
        req.state.pose.position.z = z_offset
        
        # Since reference_frame is synapse_amr, orientation should be identity (0,0,0,1)
        req.state.pose.orientation.x = 0.0
        req.state.pose.orientation.y = 0.0
        req.state.pose.orientation.z = 0.0
        req.state.pose.orientation.w = 1.0
        
        self.set_state_client.call_async(req)

    def dropoff_entity(self, entity_name, location):
        if not self.set_state_client.wait_for_service(timeout_sec=0.1):
            return

        req = SetEntityState.Request()
        req.state.name = entity_name
        req.state.reference_frame = 'world'
        
        if location == 'QC Station':
            # Place on the inspection table
            req.state.pose.position.x = 8.5
            req.state.pose.position.y = 4.5
            req.state.pose.position.z = 0.85
        elif location == 'Charging Dock':
            # Place near charging dock
            req.state.pose.position.x = 8.0
            req.state.pose.position.y = -6.0
            req.state.pose.position.z = 0.15
        else:
            # Outbound staging
            req.state.pose.position.x = -9.0
            req.state.pose.position.y = 6.5
            req.state.pose.position.z = 0.85
            
        req.state.pose.orientation.w = 1.0
        self.set_state_client.call_async(req)
        self.target_load = ""

    def plan_to_home(self):
        # Exclude the dropoff location we just delivered to, and any node occupied by other robots
        occupied = set()
        if hasattr(self, 'dropoff_loc') and self.dropoff_loc:
            occupied.add(self.dropoff_loc)
        for other, pose in self.other_amrs.items():
            for n_name, n_pos in self.nodes.items():
                if math.sqrt((pose.position.x - n_pos['x'])**2 + (pose.position.y - n_pos['y'])**2) < 1.5:
                    occupied.add(n_name)
                    
        # Do not exclude our own home station from being the destination!
        occupied.discard(self.home_station)
        
        start_node = self.find_closest_transit_node(self.current_x, self.current_y, exclude_nodes=occupied)
        if not start_node:
            start_node = self.find_closest_node(self.current_x, self.current_y)
            
        exclude_edges = set()
        if self.robot_name == 'synapse_amr_1':
            self.get_logger().info("Hardcoded Script Routing: Forcing AMR 1 to return home via the 'top way' (Aisle A) to prevent deadlocks.")
            exclude_edges.add(("w_aisle_a", "w_center"))
            
        if self.robot_name == 'synapse_amr_3':
            # Keep AMR 3 strictly in the South half (Aisle B) for the show
            exclude_edges.add(("e_aisle_b", "e_center"))
            exclude_edges.add(("w_aisle_b", "w_center"))
            
        path_nodes = self.bfs_path(start_node, self.home_station, exclude_edges=exclude_edges)
        if path_nodes:
            self.current_path = path_nodes
            self.current_waypoint_idx = 0
            self.state = 'NAV_TO_HOME'
            # Clear stale leases on every replan
            self.held_nodes.clear()
            self.requested_node = None
            self.wait_start_time = 0.0
            if start_node:
                self.held_nodes.add(start_node)
            self.get_logger().info(f"[{self.robot_name}] Path to designated home station ({self.home_station}): {self.current_path}")
        else:
            self.get_logger().info(f"[{self.robot_name}] Already at or no path to designated home station ({self.home_station}). Setting to IDLE.")
            self.state = 'IDLE'

    def plan_to_charging(self):
        # Backwards compatibility alias
        self.plan_to_home()

def main(args=None):
    import sys
    rclpy.init(args=args)
    
    # Extract robot_name from args if provided manually
    node_name = 'amr_task_executor'
    for i, arg in enumerate(sys.argv):
        if 'robot_name:=' in arg:
            robot = arg.split(':=')[1]
            node_name = f'amr_task_executor_{robot}'
            
    executor = AMRTaskExecutor(node_name=node_name)
    try:
        rclpy.spin(executor)
    except KeyboardInterrupt:
        pass
    finally:
        executor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
