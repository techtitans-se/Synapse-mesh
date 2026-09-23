#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

class STLeaseStressTest(Node):
    def __init__(self):
        super().__init__('st_lease_stress_test')
        
        self.pub_amr1 = self.create_publisher(String, '/synapse_amr_1/amr_dispatch', 10)
        self.pub_amr2 = self.create_publisher(String, '/synapse_amr_2/amr_dispatch', 10)
        self.pub_amr3 = self.create_publisher(String, '/synapse_amr_3/amr_dispatch', 10)
        
        self.get_logger().info("Waiting for subscribers...")
        # Wait until at least AMR 1 and 2 are subscribed
        while self.pub_amr1.get_subscription_count() == 0 or self.pub_amr2.get_subscription_count() == 0:
            time.sleep(0.5)
        self.get_logger().info("Subscribers connected!")
        
        self.get_logger().info("=== INITIATING ZERO-WAIT ST-LEASE DEMONSTRATION ===")
        self.get_logger().info("1. Dispatching AMR 2: Wins Lease for Aisle A -> Pick box_rack_a1_bay1 -> Deliver QC Station")
        self.dispatch(self.pub_amr2, "box_rack_a1_bay1", "QC Station")
        
        # Wait for AMR 2 to broadcast its ST-Lease heartbeat
        time.sleep(1.0)
        
        # Dispatch AMR 1
        self.get_logger().info("2. Dispatching AMR 1: Dynamically routes via West entrance to achieve Zero-Wait Handover")
        self.dispatch(self.pub_amr1, "box_rack_a2_bay2", "Outbound Staging")

        # Dispatch AMR 3 for show in Aisle B
        self.get_logger().info("3. Dispatching AMR 3: Operating in Aisle B (Just for show)")
        self.dispatch(self.pub_amr3, "box_rack_b1_bay3", "Charging Dock")
        
        self.get_logger().info("All AMRs dispatched! Watch AMR 1 & 2 handle ST-Lease while AMR 3 operates independently.")

    def dispatch(self, pub, load, dropoff):
        msg = String()
        msg.data = json.dumps({
            "target_load": load,
            "dropoff_location": dropoff
        })
        pub.publish(msg)

def main():
    rclpy.init()
    node = STLeaseStressTest()
    # allow time for messages to be sent
    time.sleep(1.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
