#!/bin/bash
source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || true
echo "Starting ROS Bridge Server on ws://localhost:9090..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
