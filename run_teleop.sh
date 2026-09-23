#!/bin/bash
# Teleoperation script to drive the AMR using keyboard
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "${SCRIPT_DIR}/install/setup.bash"

ros2 run synapse_amr_warehouse teleop_keyboard.py
