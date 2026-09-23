# Synapse AMR Warehouse Simulation (ROS 2 Humble + Gazebo)

A simulation of an industrial Autonomous Mobile Robot (AMR) inside a modern warehouse facility in Gazebo Classic with preloaded templates, inventory databases, waypoint stations, and extensible rule engine hooks.

![Synapse AMR Dashboard UI](assets/dashboard.png)

---

## 🚀 Quick Start Commands

From this directory (`/home/akil/Desktop/synapse_mesh`):

### 1. Launch the Full Simulation (Gazebo + RViz + AMR)
```bash
./run_simulation.sh
```
*Spawns the AMR at the Charging Dock inside the warehouse, starts the sensors (360° LiDAR, Depth Camera, IMU, Odometry), activates the 2D warehouse map server, and opens RViz2.*

*To run without RViz or Gazebo GUI:*
```bash
./run_simulation.sh rviz:=false
./run_simulation.sh gui:=false rviz:=false
```

---

### 2. Drive the AMR (Keyboard Teleoperation)
In a new terminal:
```bash
./run_teleop.sh
```
Controls:
- `w` / `s` : Increase / Decrease linear speed (forward/backward)
- `a` / `d` : Steer left / right
- `space` / `x` : Emergency Stop
- `q` : Quit

---

### 3. Run the AMR Rule Engine & Dispatch Framework
In a new terminal:
```bash
./run_rule_engine.sh
```
*Evaluates warehouse rules at 10 Hz, checks front sensor safety distances, tracks battery levels, and dispatches pending pick orders by priority.*

---

## 📂 Preloaded Data & Templates

| File | Purpose | Preloaded Contents |
| :--- | :--- | :--- |
| [`data/inventory_data.json`](file:///home/akil/Desktop/synapse_mesh/src/synapse_amr_warehouse/data/inventory_data.json) | Warehouse Inventory Database | 7 SKUs with rack IDs, pallet IDs, weights, stock counts, priority levels, and target stations |
| [`data/warehouse_stations.yaml`](file:///home/akil/Desktop/synapse_mesh/src/synapse_amr_warehouse/data/warehouse_stations.yaml) | Pre-mapped Waypoint Stations | 15 stations including Charging Dock, Inbound Dock, Outbound Staging, QC Table, and Aisle A/B Storage Bays |
| [`data/amr_specs.json`](file:///home/akil/Desktop/synapse_mesh/src/synapse_amr_warehouse/data/amr_specs.json) | AMR Hardware & Kinematics Profile | Mass (50 kg), Payload (250 kg), Max speeds (1.5 m/s, 1.8 rad/s), Battery specs, and sensor topics |
| [`maps/warehouse_map.yaml`](file:///home/akil/Desktop/synapse_mesh/src/synapse_amr_warehouse/maps/warehouse_map.yaml) | 2D Occupancy Grid Map | 520x400 cell map at 0.05 m/cell matching the exact Gazebo world geometry |

---

## 🤖 Robot Specifications (`synapse_amr`)

- **Chassis**: Industrial low-profile differential drive (0.85m x 0.60m x 0.32m).
- **Wheels**: Dual center traction drive wheels + 4 low-friction caster ball supports for zero-slip stability.
- **Sensors**:
  - `360° LiDAR`: Topic `/scan` (range 0.15m - 20.0m, 15 Hz)
  - `Forward RGB-D Camera`: Topics `/camera/image_raw`, `/camera/depth/image_raw`, `/camera/points`
  - `6-DoF IMU`: Topic `/imu/data` (50 Hz)
  - `Wheel Odometry`: Topic `/odom` (50 Hz) broadcasting `odom` ➔ `base_footprint` TF
- **Actuation**: Listens to standard geometry Twist commands on `/cmd_vel`.

---

## 🏭 Warehouse Layout Elements & Models

- **Industrial Pallet Racks**: Multi-tier heavy-duty storage racks forming Aisle A and Aisle B.
- **Euro-Pallets**: 1200mm x 800mm industrial wooden pallets placed in storage bays and dock zones.
- **Cargo Boxes**: Corrugated parcels with barcode shipping labels stacked on pallets.
- **AMR Autonomous Charging Dock**: Docking station with inductive pads, status beacon, and alignment target.
- **Safety Bollards**: High-contrast safety posts protecting rack corners and docking corridors.

---

## 🧩 Adding Custom Rules

Open [`src/synapse_amr_warehouse/scripts/amr_rule_engine.py`](file:///home/akil/Desktop/synapse_mesh/src/synapse_amr_warehouse/scripts/amr_rule_engine.py) and add your custom logic inside the hook methods:
- `rule_safety_collision_check()`
- `rule_battery_management()`
- `rule_dispatch_by_priority()`
- Or define new methods and register them in `evaluate_rules()`.

---

## 🧭 Spatio-Temporal Lease (ST-Lease)

### Purpose
ST-Lease enables nearby Autonomous Mobile Robots (AMRs) to resolve local resource conflicts, such as narrow aisles, without requiring a round-trip to the centralized Fleet Management System (FMS). The following mechanisms strengthen ST-Lease against communication failures, deadlocks, starvation, stale messages, and robot failures.

### Robustness Add-ons
1. **Lease Expiry / Timeout:** Every ST-Lease has a maximum validity period. If the holder fails to release or renew the lease before expiry, it becomes invalid.
2. **Lease Heartbeat & Renewal:** The lease holder periodically broadcasts its current lease state and estimated remaining traversal time.
3. **Deterministic Tie-Breaker:** If two robots have identical priority scores, a deterministic rule such as the lower unique Robot ID decides the winner.
4. **Anti-Starvation / Priority Aging:** A robot's priority increases with waiting time. `Priority = Task Urgency + Wait Cost + Battery Factor + Aging Factor`
5. **Deadlock Detection & Resolution:** Robots maintain local wait/dependency information. If a circular dependency is detected, a deterministic recovery action is triggered.
6. **Safe-Wait Position Validation:** Before a robot moves to a designated waiting position, it verifies that the position is physically available.
7. **Network-Partition Handling:** Communication failure must never be interpreted as permission to enter a constrained resource. 
8. **Sensor-Based Lease Verification:** An ST-Lease grants coordination permission, not physical clearance.
9. **Lease ID, Version & Epoch:** Each request and lease contains a unique ID and version/epoch to prevent delayed or stale messages.
10. **Duplicate & Reordered Message Protection:** Unique request IDs and state versions prevent incorrect state transitions.
11. **Priority Integrity:** Task urgency is supplied by the FMS and must be validated/authenticated.
12. **Maximum Resource Occupancy Time:** If the robot exceeds the expected traversal duration, it must renew the lease or enter recovery behavior.

### Conflict Handling Matrix

| Conflict | Risk | ST-Lease Response |
| :--- | :--- | :--- |
| Two robots request the same aisle | Both attempt entry | Priority negotiation |
| Equal priority | No clear winner | Deterministic Robot-ID tie-breaker |
| Robot never exits | Other robots wait indefinitely | Lease timeout + recovery |
| Robot is delayed | Lease duration exceeded | Heartbeat + bounded renewal |
| Repeated priority loss | Robot starvation | Priority aging |
| Circular waiting | Deadlock | Dependency detection + deadlock resolution |
| Waiting position occupied | Robot cannot safely yield | Alternate validated waiting position |
| Network failure | Robots cannot coordinate | Conservative fallback + local sensing |
| Stale lease message | Robot acts on old information | Lease ID + version/epoch |
| Duplicate message | Request processed multiple times | Unique request ID |
| False urgency | Robot manipulates priority | FMS-authorized urgency |
| Robot crashes | Lease remains active | Timeout + recovery |
| Unexpected obstacle | Robot cannot exit on time | Lease renewal or recovery |
| Late-joining robot | Missing current lease state | Periodic lease-state advertisement |
| Robot reboot | Previous state is invalid | New session/epoch ID |

### Three-Layer Safety Architecture

1. **FMS / Task Layer**: Provides global task urgency, task priority, and fleet-level objectives.
2. **ST-Lease Coordination Layer**: Handles local resource negotiation, priority calculation, lease timeout, renewal, deadlock detection, and recovery.
3. **On-Robot Safety Layer**: Uses LiDAR, proximity sensors, and collision avoidance to independently determine whether physical movement is safe.

> **CORE PRINCIPLE**: FMS decides what matters globally. ST-Lease decides which robot gets a contested local resource. The on-robot safety controller independently decides whether physical movement is safe. **ST-Lease is therefore a coordination protocol, not a physical safety system.**

---

## 📡 Synapse-Mesh: VOS Architecture Upgrades

The Virtual Occupancy Shadow (VOS) relies on static bounding boxes and reactive dead-zone handling. To ensure fault tolerance, the VOS module has three critical dynamic/predictive upgrades:

### Upgrade 1: The "Breathing" Probabilistic Shadow
- **The Flaw:** Currently, the shadow is a rigid block. Dodging an obstacle pushes a robot out of its shadow, looking "safe" to peers and causing collisions.
- **The Solution:** Transition from deterministic spatial bounds to a Spatiotemporal Gaussian Mixture Model. The shadow behaves like a fading heat map. As time passes without a re-connection, the spatial uncertainty (covariance matrix) expands dynamically, providing a conservative occupancy hint without permanently locking up the grid.

### Upgrade 2: Predictive RSSI Broadcasting
- **The Flaw:** Broadcasting a path prediction exactly when the signal is dying guarantees high packet loss.
- **The Solution:** Implement an Edge-Computed Predictive Geofence using linear regression on a rolling window of RSSI values ($`\Delta RSSI / \Delta t`$). The robot predicts when the signal will drop and broadcasts the VOS message while the signal is still strong.

### Upgrade 3: Decentralized Exception Handling
- **The Flaw:** If a robot misses its time window to exit a dead zone, peers immediately escalate to the centralized Fleet Management System (FMS), creating a bottleneck.
- **The Solution:** Integrate a Peer-to-Peer Gossip Protocol over Zenoh. Robots query immediate neighbors via the LMA to check if anyone has seen the missing robot before escalating to the FMS.

---

## 🧠 Synapse-Mesh: Edge-AI Integration & Kinodynamic Profiling

### Executive Summary
Instead of handling intersections like cars at a stop sign (stop-and-go), Synapse-Mesh replaces binary spatial yielding with **Momentum-Aware Kinodynamic Yielding**. By deploying a lightweight, supervised machine learning regression model at the edge, the system calculates optimal fractional velocity vectors ($`v_{approach}`$) in real-time. This tells the yielding robot to slightly reduce its speed so it seamlessly coasts through the intersection right after the first robot passes, eliminating the static-friction torque penalty and saving massive amounts of battery.

### Technical Architecture: Kinodynamic ST-Lease
When a conflict is detected over the Zenoh mesh, the Edge-AI model evaluates real-time telemetry instead of just issuing a `[STOP]` or `[GO]` command.

**Model Inputs (Features):**
- Distance of Robot A to the conflict zone ($`d_A`$)
- Distance of Robot B to the conflict zone ($`d_B`$)
- Current velocity of the primary robot ($`v_A`$)
- Estimated payload/mass constraints of the yielding robot

**Model Output (Target):**
- A continuous **Target Velocity Profile ($`v_B`$)**. Robot B decelerates to this precise speed, allowing it to trail Robot A safely without hitting 0 m/s.

### Advanced VOS Integration: The "Breathing" Shadow
As an AMR enters an RF dead-zone, the AI model directly couples the spatial expansion rate of the shadow to the velocity profiles of approaching AMRs. As the shadow grows, the AI instructs approaching robots to proportionally reduce their speed, preserving forward momentum while widening the safety buffer until the disconnected AMR re-establishes its link.

### Implementation & Hardware Stack
To ensure the AI does not violate strict `<15ms` latency guarantees, we use task-specific predictive models.
- **Model Architecture:** A lightweight regression model (XGBoost, Random Forest, or compact PyTorch MLP).
- **Training Pipeline:** Supervised learning using a synthetic dataset generated from Gazebo physics simulations.
- **Edge Deployment:** Exported as a standard Python `.whl` file and deployed directly onto the Raspberry Pi 5.
- **Execution:** ST-Lease logic queries the local model instantly, consuming negligible RAM and CPU cycles.

### Why It Works (The Edge Advantage)
1. **Operational Efficiency:** Overcoming static friction requires peak motor torque, draining the battery exponentially. Maintaining continuous motion extends battery duty cycles and throughput.
2. **Deterministic Safety:** The AI operates strictly as a velocity coordinator. On-robot controllers (LiDAR, e-stop) retain absolute authority.
3. **Zero-Latency Decentralization:** The model executes locally in microseconds without bottlenecking the central FMS over lossy warehouse Wi-Fi.

