import re

with open("src/synapse_amr_warehouse/scripts/amr_task_executor.py", "r") as f:
    code = f.read()

# 1. Imports
code = code.replace("from geometry_msgs.msg import Twist", "from geometry_msgs.msg import Twist\nimport time")

# 2. Init
init_addition = """
        # --- ST-Lease Protocol ---
        self.lease_pub = self.create_publisher(String, '/st_lease/coordination', 10)
        self.lease_sub = self.create_subscription(String, '/st_lease/coordination', self.lease_callback, 10)
        self.lease_timer = self.create_timer(0.5, self.publish_lease_heartbeat)

        self.held_nodes = set()
        self.requested_node = None
        self.priority_base = 10.0
        self.wait_start_time = 0.0
        self.aging_factor = 0.0
        self.fleet_state = {}
        # -------------------------
"""
code = code.replace("self.other_amrs = {}", init_addition.strip())

# 3. Add lease_callback and publish_lease_heartbeat
lease_funcs = """
    def lease_callback(self, msg):
        try:
            data = json.loads(msg.data)
            if data['robot_id'] != self.robot_name:
                self.fleet_state[data['robot_id']] = data
        except Exception:
            pass

    def publish_lease_heartbeat(self):
        priority = self.priority_base + self.aging_factor
        if self.requested_node and self.wait_start_time > 0:
            priority += (time.time() - self.wait_start_time) * 0.1
            
        msg = String()
        msg.data = json.dumps({
            "robot_id": self.robot_name,
            "held_nodes": list(self.held_nodes),
            "requested_node": self.requested_node,
            "priority": priority,
            "timestamp": time.time()
        })
        self.lease_pub.publish(msg)
        
    def check_lease_available(self, target_node):
        my_priority = self.priority_base + self.aging_factor
        if self.wait_start_time > 0:
            my_priority += (time.time() - self.wait_start_time) * 0.1

        for other_id, state in self.fleet_state.items():
            # Check timeout (3 seconds)
            if time.time() - state.get('timestamp', 0) > 3.0:
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
"""
code = code.replace("def find_closest_node(self, x, y):", lease_funcs + "\n    def find_closest_node(self, x, y):")

# 4. Modify control_loop to use ST-Lease
# Replace the entire collision avoidance block with ST-Lease request block.
old_ca_pattern = r"# --- Decentralized Collision Avoidance ---.*?# ----------------------------------------"
new_st_lease = """
        # --- ST-Lease Protocol Check ---
        # Initialize our held node if empty (we assume we hold our starting closest node)
        if not self.held_nodes and self.current_path:
            start_node = self.find_closest_node(self.current_x, self.current_y)
            if start_node:
                self.held_nodes.add(start_node)

        # We need the lease for target_node_name to proceed physically
        if target_node_name not in self.held_nodes:
            self.requested_node = target_node_name
            if self.wait_start_time == 0.0:
                self.wait_start_time = time.time()
                
            # Check if we can acquire it
            available, blocking_robot, reason = self.check_lease_available(target_node_name)
            
            if not available:
                if getattr(self, 'tick', 0) % 20 == 0:
                    self.get_logger().info(f"ST-Lease: Waiting for {target_node_name} (Blocked by {blocking_robot} - {reason})")
                
                # Check for Deadlock (Circular Dependency)
                if reason == "Held" and blocking_robot in self.fleet_state:
                    blocking_state = self.fleet_state[blocking_robot]
                    blocking_req = blocking_state.get('requested_node')
                    if blocking_req and blocking_req in self.held_nodes:
                        self.get_logger().error(f"DEADLOCK DETECTED with {blocking_robot}! They want {blocking_req}, we want {target_node_name}.")
                        # Tie-breaker determines who retreats
                        my_priority = self.priority_base + self.aging_factor + (time.time() - self.wait_start_time) * 0.1
                        their_priority = blocking_state.get('priority', 0)
                        
                        we_must_yield = False
                        if my_priority < their_priority:
                            we_must_yield = True
                        elif my_priority == their_priority and self.robot_name > blocking_robot:
                            we_must_yield = True
                            
                        if we_must_yield:
                            self.get_logger().error("We have lower priority. Retreating (Dropping path) to break deadlock!")
                            self.state = 'IDLE'
                            self.current_path = []
                            self.requested_node = None
                            self.wait_start_time = 0.0
                            self.aging_factor += 5.0 # Boost priority for next time
                            return
                
                self.vel_pub.publish(Twist()) # Brake
                self.tick = getattr(self, 'tick', 0) + 1
                return
            else:
                # ACQUIRE THE LEASE
                self.get_logger().info(f"ST-Lease: Acquired {target_node_name}!")
                self.held_nodes.add(target_node_name)
                self.requested_node = None
                self.wait_start_time = 0.0
                self.aging_factor = 0.0 # Reset aging
        # -------------------------------
        
        # Sensor-Based Verification (Robustness Add-on 8)
        # Even if we have the lease, verify physical clearance using distance to other AMRs
        for other, pose in self.other_amrs.items():
            dist = math.sqrt((pose.position.x - self.current_x)**2 + (pose.position.y - self.current_y)**2)
            if dist < 1.0:
                angle_to_other = math.atan2(pose.position.y - self.current_y, pose.position.x - self.current_x)
                rel_angle = angle_to_other - self.current_yaw
                while rel_angle > math.pi: rel_angle -= 2 * math.pi
                while rel_angle < -math.pi: rel_angle += 2 * math.pi
                
                if abs(rel_angle) < math.pi/4:
                    if getattr(self, 'tick', 0) % 20 == 0:
                        self.get_logger().warn(f"Physical path blocked by {other} (Dist: {dist:.2f}m)! Braking.")
                    self.vel_pub.publish(Twist())
                    self.tick = getattr(self, 'tick', 0) + 1
                    return
"""

code = re.sub(old_ca_pattern, new_st_lease.strip(), code, flags=re.DOTALL)

# 5. Release lease when reaching a waypoint
old_waypoint_reached = """
        if distance < 0.05:
            # Reached current waypoint
            self.current_waypoint_idx += 1
"""
new_waypoint_reached = """
        if distance < 0.05:
            # Reached current waypoint
            
            # Release previous node lease
            if self.current_waypoint_idx > 0:
                prev_node = self.current_path[self.current_waypoint_idx - 1]
                if prev_node in self.held_nodes:
                    self.held_nodes.remove(prev_node)
                    
            self.current_waypoint_idx += 1
"""
code = code.replace(old_waypoint_reached.strip(), new_waypoint_reached.strip())

with open("src/synapse_amr_warehouse/scripts/amr_task_executor.py", "w") as f:
    f.write(code)

