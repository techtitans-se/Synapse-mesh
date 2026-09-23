#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import time

class FMSDispatcher(Node):
    def __init__(self):
        super().__init__('fms_dispatcher')
        self.get_logger().info("FMS Dispatcher starting...")
        
        # Action clients for Nav2
        self.amr1_client = ActionClient(self, NavigateToPose, '/amr_1/navigate_to_pose')
        self.amr2_client = ActionClient(self, NavigateToPose, '/amr_2/navigate_to_pose')
        self.amr3_client = ActionClient(self, NavigateToPose, '/amr_3/navigate_to_pose')

    def wait_for_nav2(self):
        self.get_logger().info("Waiting for Nav2 action servers...")
        self.amr1_client.wait_for_server()
        self.amr2_client.wait_for_server()
        self.amr3_client.wait_for_server()
        self.get_logger().info("Nav2 servers ready!")

    def send_goal(self, client, x, y, yaw=0.0):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0 # simplified orientation
        
        self.get_logger().info(f"Sending goal ({x}, {y})")
        client.send_goal_async(goal_msg)

    def dispatch_conflict_scenario(self):
        # All three AMRs will be commanded to cross the central narrow aisle simultaneously
        
        self.get_logger().info("Dispatching 3-robot conflict scenario...")
        # AMR 1 goes South through the aisle
        self.send_goal(self.amr1_client, 0.0, -4.0)
        
        time.sleep(1.0)
        
        # AMR 2 goes North through the aisle
        self.send_goal(self.amr2_client, 0.0, 4.0)
        
        time.sleep(1.0)
        
        # AMR 3 goes West through the aisle (starts at x=4)
        self.send_goal(self.amr3_client, -4.0, 0.0)

def main(args=None):
    rclpy.init(args=args)
    dispatcher = FMSDispatcher()
    
    # In a real scenario, we would wait for Nav2 to be fully active.
    # For now, we assume it's running.
    # dispatcher.wait_for_nav2() 
    
    dispatcher.dispatch_conflict_scenario()
    
    try:
        rclpy.spin(dispatcher)
    except KeyboardInterrupt:
        pass
    finally:
        dispatcher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
