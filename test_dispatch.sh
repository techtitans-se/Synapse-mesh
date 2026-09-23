source /opt/ros/humble/setup.bash
ros2 topic pub /amr/dispatch_command std_msgs/msg/String '{data: "{\"target_load\": \"box_rack_a2_bay2\", \"dropoff_location\": \"QC Station\"}"}' --once
