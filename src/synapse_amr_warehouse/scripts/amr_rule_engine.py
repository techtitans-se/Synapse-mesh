#!/usr/bin/env python3
"""
Autonomous Mobile Robot (AMR) Warehouse Rule Engine & Dispatch Framework
Author: Synapse AMR Robotics
Description:
  Loads warehouse stations, inventory databases, and AMR specifications.
  Provides a plug-and-play rule-based architecture where users can add
  custom rules for dispatch, collision avoidance, battery management,
  and pick-and-place logic.
"""

import os
import json
import yaml
import math
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class AMRRuleEngine(Node):
    def __init__(self):
        super().__init__('amr_rule_engine')
        self.get_logger().info('Initializing AMR Warehouse Rule Engine...')

        # Load package data directory
        try:
            pkg_dir = get_package_share_directory('synapse_amr_warehouse')
            data_dir = os.path.join(pkg_dir, 'data')
        except Exception:
            # Fallback to local source directory if running directly
            current_dir = os.path.dirname(os.path.realpath(__file__))
            data_dir = os.path.abspath(os.path.join(current_dir, '..', 'data'))

        # 1. Load Preloaded Inventory Data
        inv_path = os.path.join(data_dir, 'inventory_data.json')
        with open(inv_path, 'r') as f:
            self.inventory_data = json.load(f)
        self.get_logger().info(f"Loaded {len(self.inventory_data.get('inventory', []))} SKUs from inventory_data.json")

        # 2. Load Preloaded Warehouse Stations
        stations_path = os.path.join(data_dir, 'warehouse_stations.yaml')
        with open(stations_path, 'r') as f:
            self.stations_data = yaml.safe_load(f).get('stations', {})
        self.get_logger().info(f"Loaded {len(self.stations_data)} waypoint stations from warehouse_stations.yaml")

        # 3. Load AMR Specifications
        specs_path = os.path.join(data_dir, 'amr_specs.json')
        with open(specs_path, 'r') as f:
            self.amr_specs = json.load(f)
        self.get_logger().info(f"AMR Spec: {self.amr_specs.get('robot_class')}, max payload {self.amr_specs['mechanical']['max_payload_kg']}kg")

        # State Variables
        self.current_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.min_front_scan_distance = float('inf')
        self.battery_level_pct = 95.0
        self.active_mission = None

        # ROS 2 Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # ROS 2 Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/amr/rule_engine_status', 10)

        # Periodic Rule Evaluation Loop (10 Hz)
        self.rule_timer = self.create_timer(0.1, self.evaluate_rules)
        self.status_timer = self.create_timer(2.0, self.publish_status)

        self.get_logger().info('AMR Warehouse Rule Engine is ACTIVE and ready for user rules!')

    def odom_callback(self, msg: Odometry):
        self.current_pose['x'] = msg.pose.pose.position.x
        self.current_pose['y'] = msg.pose.pose.position.y

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_pose['yaw'] = math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg: LaserScan):
        # Scan front sector (-30 to +30 degrees)
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        total_samples = len(msg.ranges)

        front_min_idx = int((-math.radians(30) - angle_min) / angle_inc)
        front_max_idx = int((math.radians(30) - angle_min) / angle_inc)
        front_min_idx = max(0, min(total_samples - 1, front_min_idx))
        front_max_idx = max(0, min(total_samples - 1, front_max_idx))

        if front_min_idx > front_max_idx:
            front_min_idx, front_max_idx = front_max_idx, front_min_idx

        front_ranges = [r for r in msg.ranges[front_min_idx:front_max_idx+1] if msg.range_min < r < msg.range_max]
        if front_ranges:
            self.min_front_scan_distance = min(front_ranges)
        else:
            self.min_front_scan_distance = float('inf')

    # =========================================================================
    # USER RULE HOOKS - Extend with your warehouse business rules below
    # =========================================================================

    def rule_safety_collision_check(self) -> bool:
        """Rule: Emergency stop or speed curtailment if obstacle is too close"""
        safety_stop_distance = 0.45 # meters
        if self.min_front_scan_distance < safety_stop_distance:
            self.get_logger().warn(
                f"[RULE TRIGGER] Obstacle proximity alert! Distance: {self.min_front_scan_distance:.2f}m",
                throttle_duration_sec=2.0
            )
            return False # Unsafe to proceed
        return True # Safe

    def rule_battery_management(self):
        """Rule: Monitor battery and return to charging dock when critical"""
        low_threshold = self.amr_specs['battery']['low_battery_threshold_pct']
        if self.battery_level_pct <= low_threshold and self.active_mission != 'RETURN_TO_CHARGING':
            self.get_logger().warn(
                f"[RULE TRIGGER] Battery below threshold ({self.battery_level_pct}% <= {low_threshold}%). Triggering return to dock!",
                throttle_duration_sec=5.0
            )
            self.active_mission = 'RETURN_TO_CHARGING'

    def rule_dispatch_by_priority(self):
        """Rule: Select pending SKU with highest priority (CRITICAL > HIGH > MEDIUM > LOW)"""
        if self.active_mission is None:
            priority_order = {"CRITICAL": 4, "HIGH": 3, "MEDIUM": 2, "LOW": 1}
            pending = [item for item in self.inventory_data.get('inventory', []) if item.get('status') == 'READY_FOR_PICK']
            if pending:
                pending.sort(key=lambda item: priority_order.get(item.get('priority', 'LOW'), 0), reverse=True)
                target_item = pending[0]
                self.active_mission = f"PICK_{target_item['sku']}"
                self.get_logger().info(f"[RULE TRIGGER] Dispatched mission: Pick {target_item['name']} ({target_item['sku']}) from {target_item['target_station']}")

    def evaluate_rules(self):
        """Central rule engine loop called at 10Hz"""
        # Execute active rules
        self.rule_battery_management()
        self.rule_dispatch_by_priority()
        is_safe = self.rule_safety_collision_check()

        # Simulated battery drain when operating
        if self.battery_level_pct > 1.0:
            self.battery_level_pct -= 0.0005

    def publish_status(self):
        status = {
            'amr_id': self.amr_specs.get('robot_id'),
            'pose': {
                'x': round(self.current_pose['x'], 2),
                'y': round(self.current_pose['y'], 2),
                'yaw_deg': round(math.degrees(self.current_pose['yaw']), 1)
            },
            'battery_pct': round(self.battery_level_pct, 1),
            'front_obstacle_dist_m': round(self.min_front_scan_distance, 2),
            'active_mission': self.active_mission,
            'loaded_stations': list(self.stations_data.keys())
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AMRRuleEngine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
