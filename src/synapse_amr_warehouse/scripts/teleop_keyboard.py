#!/usr/bin/env python3
"""
Interactive Keyboard Teleoperation for Synapse AMR in Gazebo
Controls:
  w / s : increase / decrease linear velocity (forward/backward)
  a / d : increase / decrease angular velocity (left/right turning)
  space / x : emergency stop / reset speed to zero
  q : quit
"""

import sys
import termios
import tty
import select
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

BANNER = """
======================================================
  SYNAPSE AMR - WAREHOUSE TELEOPERATION CONSOLE
======================================================
  Movement Controls:
        [w]
   [a]  [s]  [d]

   w : Forward (+linear velocity)
   s : Backward (-linear velocity)
   a : Turn Left (+angular velocity)
   d : Turn Right (-angular velocity)
   space / x : STOP
   q : Quit

  Linear Step : 0.15 m/s (Max: 1.5 m/s)
  Angular Step: 0.25 rad/s (Max: 1.8 rad/s)
======================================================
"""

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = Node('amr_teleop_keyboard')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    linear_vel = 0.0
    angular_vel = 0.0
    max_linear = 1.5
    max_angular = 1.8
    step_linear = 0.15
    step_angular = 0.25

    print(BANNER)
    try:
        while rclpy.ok():
            key = get_key(settings)
            if key == 'w':
                linear_vel = min(max_linear, linear_vel + step_linear)
            elif key == 's':
                linear_vel = max(-max_linear, linear_vel - step_linear)
            elif key == 'a':
                angular_vel = min(max_angular, angular_vel + step_angular)
            elif key == 'd':
                angular_vel = max(-max_angular, angular_vel - step_angular)
            elif key in (' ', 'x'):
                linear_vel = 0.0
                angular_vel = 0.0
            elif key == 'q':
                break

            twist = Twist()
            twist.linear.x = float(linear_vel)
            twist.angular.z = float(angular_vel)
            pub.publish(twist)

            if key != '':
                print(f"\rCurrent Velocity: Linear = {linear_vel:+.2f} m/s | Angular = {angular_vel:+.2f} rad/s    ", end='', flush=True)

    except Exception as e:
        print(f"\nError: {e}")
    finally:
        twist = Twist()
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
        print("\nTeleop stopped.")


if __name__ == '__main__':
    main()
