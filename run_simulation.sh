#!/bin/bash
# One-click script to launch Gazebo Warehouse AMR Simulation
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "${SCRIPT_DIR}/install/setup.bash"

echo "=========================================================="
echo " Starting Synapse AMR Warehouse Simulation in Gazebo..."
echo " World: Industrial Warehouse (Pallet Racks, Docks, Charging)"
echo " Robot: Industrial Differential Drive AMR (LiDAR, Camera, IMU)"
echo "=========================================================="

ros2 launch synapse_amr_warehouse warehouse_sim.launch.py "$@"
