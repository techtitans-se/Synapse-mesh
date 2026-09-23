import re

with open("src/synapse_amr_warehouse/scripts/amr_task_executor.py", "r") as f:
    code = f.read()

# 1. Extract the sensor block
sensor_block_match = re.search(r"(\s*# Sensor-Based Verification.*?return)", code, flags=re.DOTALL)
if sensor_block_match:
    sensor_block = sensor_block_match.group(1)
    
    # Remove it from its original location
    code = code.replace(sensor_block, "")
    
    # Insert it right after the yaw_error normalization
    insertion_point = r"while yaw_error < -math.pi: yaw_error \+= 2 \* math.pi\n"
    new_code = re.sub(insertion_point, insertion_point + "\n" + sensor_block + "\n", code)
    if new_code != code:
        code = new_code
    else:
        print("Failed to insert sensor block")

# 2. Fix the lease cleanup at waypoint reached
old_waypoint_reached = """
            # Release previous node lease
            if self.current_waypoint_idx > 0:
                prev_node = self.current_path[self.current_waypoint_idx - 1]
                if prev_node in self.held_nodes:
                    self.held_nodes.remove(prev_node)
"""
new_waypoint_reached = """
            # Reset held leases to ONLY the node we just arrived at
            self.held_nodes = {target_node_name}
"""
code = code.replace(old_waypoint_reached, new_waypoint_reached)

with open("src/synapse_amr_warehouse/scripts/amr_task_executor.py", "w") as f:
    f.write(code)

