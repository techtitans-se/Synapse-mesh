#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from std_srvs.srv import Trigger
import json
import time

class FMSServer(Node):
    def __init__(self):
        super().__init__('fms_server')

        # State tracking for Dashboard
        self.current_state_payload = {
            "scenario": "IDLE",
            "status": "System Ready.",
            "robots": {
                "amr_1": {"state": "IDLE", "priority": 2.0, "aging": 0.0, "p_eff": 2.0},
                "amr_2": {"state": "IDLE", "priority": 1.0, "aging": 0.0, "p_eff": 1.0},
                "amr_3": {"state": "IDLE", "priority": 3.0, "aging": 0.0, "p_eff": 3.0}
            },
            "shadow": {"active": False, "x": 0.0, "y": 0.0, "radius": 0.0, "alpha": 0.0},
            "deadlock": {"active": False, "from": "", "to": ""},
            "safewait": {"active": False}
        }

        # Telemetry Publisher
        self.system_state_pub = self.create_publisher(String, '/fms/system_state', 10)
        self.bidding_pub = self.create_publisher(String, '/fms/st_lease_bidding', 10)

        # ST-Lease Node Listeners (to update the dashboard state from real autonomous data)
        self.create_subscription(String, '/amr_1/st_lease_status', lambda msg: self._update_robot_state('amr_1', msg), 10)
        self.create_subscription(String, '/amr_2/st_lease_status', lambda msg: self._update_robot_state('amr_2', msg), 10)
        self.create_subscription(String, '/amr_3/st_lease_status', lambda msg: self._update_robot_state('amr_3', msg), 10)

        # Timer for periodic telemetry
        self.create_timer(0.2, self._publish_telemetry)

        # Dashboard Command Listeners (converted to publishers so AMRs can listen)
        self.cmd_scenario_pub = self.create_publisher(String, '/fms/dashboard_cmd', 10)

        self.create_service(Trigger, '/fms/reset_home', self.trigger_reset_home)
        self.create_service(Trigger, '/fms/trigger_tie_breaker', self.trigger_tie_breaker)
        self.create_service(Trigger, '/fms/trigger_lease_timeout', self.trigger_lease_timeout)
        self.create_service(Trigger, '/fms/trigger_vos_breathing', self.trigger_vos_breathing)

        self.get_logger().info("Telemetry FMS Server running (Puppet-Master Disabled).")

    def _update_robot_state(self, robot_id, msg):
        try:
            data = json.loads(msg.data)
            self.current_state_payload["robots"][robot_id] = data
        except Exception as e:
            pass

    def _publish_telemetry(self):
        msg = String()
        msg.data = json.dumps(self.current_state_payload)
        self.system_state_pub.publish(msg)

    def _broadcast(self, msg: str):
        self.get_logger().info(msg)
        self.bidding_pub.publish(String(data=msg))

    def trigger_reset_home(self, req, res):
        self._broadcast("[FMS] Dashboard commanded RESET_HOME.")
        self.cmd_scenario_pub.publish(String(data="RESET_HOME"))
        res.success = True
        res.message = "Sent RESET_HOME command to all AMRs."
        return res

    def trigger_tie_breaker(self, req, res):
        self._broadcast("[FMS] Dashboard commanded FORCE_CROSSING (Tie-Breaker).")
        self.cmd_scenario_pub.publish(String(data="FORCE_CROSSING"))
        res.success = True
        res.message = "Triggered all AMRs to cross the conflict zone."
        return res

    def trigger_lease_timeout(self, req, res):
        self._broadcast("[FMS] Dashboard commanded FAULT_STALL (Lease Timeout).")
        self.cmd_scenario_pub.publish(String(data="FAULT_STALL_AMR1"))
        res.success = True
        res.message = "Injected Stall Fault on AMR 1."
        return res

    def trigger_vos_breathing(self, req, res):
        self._broadcast("[FMS] Dashboard commanded FAULT_NETWORK (Drop Mesh).")
        self.cmd_scenario_pub.publish(String(data="FAULT_NET_AMR2"))
        res.success = True
        res.message = "Dropped AMR 2 from Zenoh mesh."
        return res

def main(args=None):
    rclpy.init(args=args)
    server = FMSServer()
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
