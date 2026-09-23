#!/usr/bin/env python3
"""
VOS Simulation — Fully Hardcoded Choreography
=============================================
This script runs the complete VOS demonstration as a scripted scene:
  1. AMR 3 navigates to Aisle B (the Dead Zone)
  2. AMR 3 pauses at the boundary, broadcasts its path to AMR 2
  3. AMR 3 enters the dead zone — signal goes dark
  4. AMR 2 switches to monitoring mode
  5. AMR 3 exits the dead zone — connection regained
  6. AMR 2 returns to normal operations
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import threading


class VOSSimulation(Node):
    def __init__(self):
        super().__init__('vos_simulation')

        # Dispatch publishers
        self.pub_amr1 = self.create_publisher(String, '/synapse_amr_1/amr_dispatch', 10)
        self.pub_amr2 = self.create_publisher(String, '/synapse_amr_2/amr_dispatch', 10)
        self.pub_amr3 = self.create_publisher(String, '/synapse_amr_3/amr_dispatch', 10)

        # VOS Status publisher — dashboard reads this
        self.vos_status_pub = self.create_publisher(String, '/vos/status', 10)

        # Wait for all AMR executors to connect
        self.get_logger().info("Waiting for all 3 AMR subscribers...")
        while (self.pub_amr1.get_subscription_count() == 0 or
               self.pub_amr2.get_subscription_count() == 0 or
               self.pub_amr3.get_subscription_count() == 0):
            time.sleep(0.5)
        self.get_logger().info("All 3 AMRs connected. Starting VOS Simulation...")

        # Run in a background thread so rclpy can keep spinning
        threading.Thread(target=self.run_simulation, daemon=True).start()

    def publish_status(self, event, robot, message, extra=None):
        """Publish a VOS status event to the dashboard."""
        payload = {
            "event": event,
            "robot": robot,
            "message": message,
            "timestamp": time.time()
        }
        if extra:
            payload.update(extra)
        msg = String()
        msg.data = json.dumps(payload)
        self.vos_status_pub.publish(msg)
        self.get_logger().info(f"[VOS STATUS] {event} — {message}")

    def dispatch(self, pub, load, dropoff):
        msg = String()
        msg.data = json.dumps({"target_load": load, "dropoff_location": dropoff})
        pub.publish(msg)

    def run_simulation(self):
        time.sleep(1.0)  # Brief settle time

        # ── SCENE 1: Announce simulation start ─────────────────────────────
        self.publish_status("SIMULATION_START", "system",
                            "VOS Simulation starting. AMR 3 is the Dead Zone subject.")
        time.sleep(2.0)

        # ── SCENE 2: Dispatch AMR 3 into Aisle B (Dead Zone) ───────────────
        self.publish_status("NAVIGATING", "synapse_amr_3",
                            "AMR 3 navigating toward Aisle B (Wi-Fi Dead Zone)...")
        self.dispatch(self.pub_amr3, "box_rack_b1_bay3", "Charging Dock")
        time.sleep(3.0)

        # ── SCENE 3: AMR 2 on standby ──────────────────────────────────────
        self.publish_status("STANDBY", "synapse_amr_2",
                            "AMR 2 on standby — ready to monitor AMR 3's VOS Shadow.")
        self.dispatch(self.pub_amr2, "box_rack_a1_bay1", "QC Station")
        time.sleep(2.0)

        # ── SCENE 4: AMR 1 dispatched ──────────────────────────────────────
        self.publish_status("AMR1_TASK", "synapse_amr_1",
                            "AMR 1 dispatched to Aisle A to demonstrate independent operations.")
        self.dispatch(self.pub_amr1, "box_rack_a2_bay2", "Outbound Staging")
        time.sleep(8.0) # Wait for AMR 3 to physically reach boundary

        # ── SCENE 5: Approaching Boundary ──────────────────────────────────
        self.publish_status("APPROACHING", "synapse_amr_3",
                            "AMR 3 approaching Dead Zone boundary — RSSI dropping...")
        time.sleep(1.0) # The 1 second pause requested in the script
        
        # ── SCENE 6: Broadcasting Path ─────────────────────────────────────
        # Hardcoded path coordinates for the Violet Line
        path_coords = [
            {"x": -7.5, "y": -5.9},
            {"x": -6.6, "y": -5.9},
            {"x": -6.6, "y": -3.6},
            {"x": -4.5, "y": -3.6},
            {"x": -1.5, "y": -3.6},
            {"x": 1.5, "y": -3.6},
            {"x": 1.5, "y": -2.2}
        ]
        self.publish_status("BROADCASTING_PATH", "synapse_amr_3",
                            "AMR 3 broadcasting full path to AMR 2 before entering dead zone...",
                            extra={"path": path_coords})
        time.sleep(1.0)
        
        # ── SCENE 7: AMR 2 Monitoring ──────────────────────────────────────
        self.publish_status("MONITORING", "synapse_amr_2",
                            "AMR 2 now monitoring AMR 3's VOS Shadow. Tracking last known position in Aisle B.")
        time.sleep(1.0)
        
        # ── SCENE 8: Entering Dead Zone ────────────────────────────────────
        self.publish_status("ENTERING_DEAD_ZONE", "synapse_amr_3",
                            "AMR 3 entering Dead Zone — signal going dark.")
        self.publish_status("SHADOW_ACTIVE", "synapse_amr_3",
                            "AMR 3 signal LOST inside Dead Zone. VOS Shadow blocking Aisle B for all peers.")
        time.sleep(18.0) # Time to physically cross the dead zone

        # ── SCENE 9: Regained Connection ───────────────────────────────────
        self.publish_status("CONNECTION_REGAINED", "synapse_amr_3",
                            "AMR 3 has exited the Dead Zone — connection regained! Shadow cleared.")
        time.sleep(1.0)
        self.publish_status("AMR2_RESUME", "synapse_amr_2",
                            "AMR 2 detected peer signal recovery. Returning to normal operations.")
        time.sleep(2.0)

        self.publish_status("SIMULATION_COMPLETE", "system",
                            "VOS Simulation complete! All 3 upgrade scenarios demonstrated successfully.")


def main():
    rclpy.init()
    node = VOSSimulation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

